/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple VXD (Imagination MSVDX) video decoder driver
 *
 * Bring-up driver for the S8000/S8003 "VXD" block: PowerVR MSVDX with the
 * resident DEVA (MTX) firmware, reverse engineered from the iOS 16.7.10
 * AppleD5500 kext.  The driver follows the rung ladder in
 * docs/periph/ds-video-decoder.md:
 *
 *   R0  probe without firmware: power domain, MMIO, IRQ, core revision
 *   R1  boot the resident DEVA image on the MTX
 *   R3a host<->firmware mailbox: ring geometry, 0x80 init message, push,
 *       IRQ drain, fences, debugfs ping (ds-vxd-decode-protocol.md)
 *   R3b command buffers: 8 x 256 KiB pool, packet container, command
 *       emitters, GART-mapped V4L2 buffers (ds-vxd-cmdstream.md)
 *   R3c+ H.264 command-stream generator and the decode submit (pending)
 */
#ifndef __APPLE_VXD_H__
#define __APPLE_VXD_H__

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include "apple-vxd-cmd.h"
#include "apple-vxd-regs.h"

struct dentry;

#define APPLE_VXD_DRV_NAME		"apple-vxd"
#define APPLE_VXD_FW_NAME		"apple/vxd-deva.bin"
#define APPLE_VXD_FW_MAX_WORDS		0x10000

/*
 * DT advertises h264-playback-level = 42: H.264 level 4.2, 1920x1088 [E].
 * Tile decode exists (H264DecoderSupportsTileDecode = true) and could lift
 * the pixel bound, but that path is not explored yet.
 */
#define APPLE_VXD_MIN_PIC_W		16
#define APPLE_VXD_MIN_PIC_H		16
#define APPLE_VXD_MAX_PIC_W		1920
#define APPLE_VXD_MAX_PIC_H		1088

/* Output image formats.  P010/NV16 follow once HEVC/4:2:2 are proven. */
enum apple_vxd_image_fmt {
	APPLE_VXD_IMG_FMT_ANY = 0,
	APPLE_VXD_IMG_FMT_420_8BIT,
};

/* SoC variant data (compatible table). */
struct apple_vxd_variant {
	unsigned int vxd_version;	/* ADT "vxd-version" */
	u32 num_gart_pts;		/* mapped page tables (512 = 2 GiB) */
	bool hevc_supported;
};

/*
 * The core's own two-level "GART" (there is no DART on this node): a 1024
 * entry directory of 4 KiB page tables covering 4 GiB of device address
 * space, entries beyond the mapped page tables pointing at a dummy page.
 *
 * GART pages are always 4 KiB, whatever the kernel page size (the device
 * kernel runs 16 KiB pages).  Entries are ((PA >> 8) & 0xfffffff0) | 1,
 * computed on the full 40-bit physical address exactly as
 * AppleD5500::initialize_gart does [E]: DRAM sits at 0x8_0000_0000 here,
 * so truncating to 32 bits first (rounds 1-3) pointed every entry below
 * DRAM.
 *
 * The directory is 64 KiB aligned: Apple allocates it that way
 * (withOptions(0x403, 0x1000, alignment 0x10000)) because the decode
 * message carries (MMU_DIR_ADR value & ~0xff) | hwContextID in one word
 * (SubmitCommandBufferInternal, ds-vxd-decode-protocol.md correction).
 *
 * We identity-map the low 32 bits of every DMA address, so device address
 * == dma_addr & 0xffffffff for the whole 2 GiB of DRAM on this device, and
 * the mapped page tables (indices 0..num_gart_pts) cover it.
 */
#define VXD_GART_PAGE_SHIFT		12
#define VXD_GART_PAGE_SIZE		BIT(VXD_GART_PAGE_SHIFT)
#define VXD_GART_DIR_ENTRIES		1024
#define VXD_GART_PT_ENTRIES		1024
#define VXD_GART_PT_SIZE		(VXD_GART_PT_ENTRIES * sizeof(u32))
#define VXD_GART_DIR_ALIGN		SZ_64K
#define VXD_GART_PA_BITS		40
#define VXD_GART_PTE_VALID		BIT(0)
#define VXD_GART_PTE(phys)		(((u32)((u64)(phys) >> 8) & 0xfffffff0) | \
					 VXD_GART_PTE_VALID)

struct apple_vxd_mapping {
	struct list_head list;
	dma_addr_t dma;
	size_t size;
	u32 va;				/* device address, == low 32 of dma */
};

/*
 * A coherent DMA buffer whose bus address is aligned beyond PAGE_SIZE.
 * dma_alloc_coherent() makes no alignment promise past the page, so the
 * buffer is over-allocated when the first try is misaligned.
 */
struct apple_vxd_dma_buf {
	void *vaddr;			/* aligned CPU address */
	dma_addr_t dma;			/* aligned bus address */
	size_t size;			/* usable bytes from vaddr */
	void *raw_vaddr;		/* what dma_free_coherent() needs */
	dma_addr_t raw_dma;
	size_t raw_size;
};

/* A zeroed, GART-mapped buffer the core reads or writes (decode context). */
struct apple_vxd_dev_buf {
	struct apple_vxd_dma_buf mem;
	struct apple_vxd_mapping map;
	u32 va;				/* device address of mem.vaddr */
};

struct apple_vxd_gart {
	struct mutex lock;		/* serializes table updates */
	struct apple_vxd_dma_buf dir_buf; /* directory, 64 KiB aligned */
	u32 *dir;			/* directory table, VXD_GART_DIR_ENTRIES */
	dma_addr_t dir_dma;
	u32 *dummy;			/* unmapped VAs resolve here */
	dma_addr_t dummy_dma;
	u32 *pts;			/* num_gart_pts consecutive 4 KiB tables */
	dma_addr_t pts_dma;
	u32 num_pts;
	struct list_head mappings;	/* every live apple_vxd_mapping */
	/*
	 * Set by every map/unmap.  Apple rewrites 0x1008 = 0xf00 before the
	 * next decode submit when its GART changed (AppleD5500+0x27a0 [E]);
	 * the decode submit path consumes and clears it.
	 */
	bool dirty;
};

/*
 * One message ring as published by the firmware after the ready magic:
 * word i lives at VXD_RING_WINDOW + base + 4 * i.  Read once per boot,
 * validated against the MMIO window, never trusted beyond it.
 */
struct apple_vxd_ring {
	u32 base;			/* byte offset from VXD_RING_WINDOW */
	u32 cap;			/* capacity in words */
};

/* Where a pushed message sits in TO_MTX, for the consumption check. */
struct apple_vxd_msg_ticket {
	u32 start;			/* write index before the push */
	u32 nwords;
	u32 end;			/* write index after the push */
};

/*
 * Fence tracking.  The fence is the 16-bit tag in w0[31:16] of every
 * message; the firmware echoes it in the 0xc0 completion.  Apple builds it
 * as (submission << 4) | (packet * 2 & 0xe) | core (0xfffffff00628a8d4
 * [E]), and routes completions to core (fence & 1).  A slot is armed
 * before the push and signalled from the IRQ drain.
 */
#define APPLE_VXD_NUM_FENCES		16

enum apple_vxd_fence_state {
	APPLE_VXD_FENCE_FREE = 0,
	APPLE_VXD_FENCE_ARMED,		/* message pushed, completion pending */
	APPLE_VXD_FENCE_SIGNALLED,	/* 0xc0 seen (or failed by panic/teardown) */
};

struct apple_vxd_fence {
	enum apple_vxd_fence_state state;
	u16 id;				/* w0[31:16] */
	u8 type;			/* message type it tagged */
	bool error;			/* fault bits, panic or teardown */
	u32 status;			/* 0xc0 word 2 */
	struct apple_vxd_cmdbuf *cmdbuf; /* freed by a clean completion */
	bool job;			/* last packet of a decode job */
	struct completion done;
};

/*
 * R3b: command buffers.  Apple allocates eight 0x40000-byte register
 * programs and eight 0x10000-byte packet lists per H.264 decoder
 * (DecodeCreateH264VLD 0xfffffff0062c3d98, ds-vxd-cmdstream.md section
 * 16.5 [E]); a B slice with weight tables takes about 1.5 KiB, so 256 KiB
 * holds APPLE_VXD_JOB_MAX_PACKETS of the largest.  Each one is two things:
 *
 *   - the device-visible register program, GART-mapped, filled with the
 *     ds-vxd-cmdstream.md command words one 64-byte-aligned chunk per
 *     slice, and read by the core's command processor;
 *   - a host-only packet list (one {0x80000007, 0x78, 0xabcdefab} +
 *     payload per slice) that the submit path turns into one mailbox
 *     message each and never hands to the device.
 *
 * Ownership: FREE (pool) -> HOST (CPU writes it) -> DEVICE (submitted,
 * bound to the fence of its last message) -> FREE when that fence
 * completes.  A buffer whose fence was failed by a panic or a teardown
 * stays DEVICE: the core may still be reading it, so only
 * apple_vxd_cmdbufs_reclaim() after the firmware is stopped frees it.
 */
#define APPLE_VXD_NUM_CMDBUFS		8
#define APPLE_VXD_CMDBUF_SIZE		SZ_256K
#define APPLE_VXD_CMDBUF_ALIGN		SZ_64K
#define APPLE_VXD_PKTLIST_SIZE		SZ_32K	/* 128 x (12 + 0x78) bytes */

struct apple_vxd_cmdbuf_pool {
	struct apple_vxd_dma_buf mem;		/* NUM x SIZE, 64 KiB aligned */
	struct apple_vxd_mapping map;
	u8 *pkt_mem;
	struct apple_vxd_cmdbuf bufs[APPLE_VXD_NUM_CMDBUFS];
	bool ready;
};

/*
 * V4L2 buffer with its GART mapping: the 0xa load command takes the
 * bitstream by device address, and the render target / DPB registers take
 * the capture surfaces the same way (ds-vxd-cmdstream.md sections 3.3, 6).
 */
struct apple_vxd_buffer {
	struct v4l2_m2m_buffer base;
	struct apple_vxd_mapping map;
	u32 va;				/* device VA of plane 0's first byte */
	/*
	 * CAPTURE only: the per-picture side buffer the core keeps with the
	 * reconstructed surface (H.264: stored macroblock parameters, BE
	 * register 0x62ec), allocated with the buffer.
	 */
	struct apple_vxd_dev_buf aux;
};

struct apple_vxd_mbox_stats {
	u32 sent;			/* messages pushed to TO_MTX */
	u32 received;			/* messages drained from TO_HOST */
	u32 irqs;			/* handled interrupts */
	u32 spurious;			/* interrupts without 0x410 bit 20 */
	u32 completions;		/* 0xc0 */
	u32 unknown_fences;		/* 0xc0 for no armed fence */
	u32 faults;			/* 0xc0 with fault bits */
	u32 contiguity;			/* 0xc6 */
	u32 panics;			/* 0xc7 */
	u32 unknown;			/* anything else */
	u32 malformed;			/* bad length / index */
	u32 last_irq_status;		/* 0x410 at the last interrupt */
};

/*
 * Active decode job.  Submission is one picture: one stateless request, one
 * bitstream buffer, one capture surface.  Completion arrives from the
 * firmware through the TO_HOST ring as a 0xc0 message.
 */
struct apple_vxd_run {
	struct vb2_v4l2_buffer *src;
	struct vb2_v4l2_buffer *dst;
};

/*
 * One submission in flight: one picture, one command buffer, one message
 * per packet.  Every message carries a fence of the same submission,
 * (seq << 4) | (packet * 2 & 0xe) as Apple computes it, so the tags repeat
 * after eight packets.  Up to eight packets every fence is armed and a
 * 0xc0 for any of them is recognised (round 5 on the device: only the
 * 0x4004 message answers); past eight only the last packet's is, since
 * the firmware never answers a 0x2000 message.  The job completes on the
 * last packet's fence or on the watchdog.
 */
#define APPLE_VXD_JOB_MAX_FENCES	8
#define APPLE_VXD_JOB_MAX_PACKETS	128
#define APPLE_VXD_JOB_TIMEOUT_MS	1000

struct apple_vxd_job {
	struct apple_vxd_ctx *ctx;
	struct apple_vxd_fence *fences[APPLE_VXD_JOB_MAX_FENCES];
	unsigned int nfences;		/* armed; the last one is the job's */
	unsigned int npackets;
	u16 seq;
	ktime_t submitted;
};

struct apple_vxd_dev {
	struct device *dev;
	const struct apple_vxd_variant *variant;

	void __iomem *regs;
	resource_size_t regs_size;	/* bounds every ring access */
	int irq;

	/* Bring-up state (R0/R1). */
	struct mutex power_lock;	/* boot vs. runtime suspend */
	bool hw_ready;			/* firmware took its init message */
	bool selftest_failed;		/* protocol self-test: no boot */
	u32 core_id;
	u32 core_rev;
	u32 va_offset;			/* device-VA base offset (module param) */
	u32 num_entropy_pipes;
	u32 num_pixel_pipes;
	u32 mtx_mem_size;		/* KiB, from MTX_MEM_CFG */
	bool mmu_40bit;
	bool mmu_tiling_ctx;
	u32 pipe_status;		/* raw 0x40c0 after power-up */
	u32 pipe_status_cfg;		/* OR of per-pipe 0x40c0 (R1 discovery) */
	u32 pipe_status_2_max;		/* max per-pipe 0x40c4 & 3 */
	u32 cfg_flags;			/* InitializeCoreConfig core[0x54] */

	/* R1: firmware buffer + boot results. */
	bool fw_loaded;			/* MTX up: ready magic seen */
	void *fw_vaddr;
	dma_addr_t fw_dma;
	size_t fw_size;			/* PAGE_ALIGNed allocation */
	size_t fw_blob_size;		/* image bytes (init message pages) */
	u32 fw_dev_va;
	struct apple_vxd_mapping fw_map;	/* GART mapping of fw_dev_va */

	struct apple_vxd_gart gart;

	/*
	 * R3a: host<->MTX mailbox.  mbox_lock covers ring MMIO in both
	 * directions, the fence table and the stats; it is taken from the
	 * hard IRQ handler.  mbox_ready/hw_ready/irq_armed change only under
	 * power_lock.
	 */
	spinlock_t mbox_lock;
	struct apple_vxd_ring to_mtx;
	struct apple_vxd_ring to_host;
	bool mbox_ready;		/* geometry read and valid */
	bool irq_armed;			/* IRQ line enabled */
	bool irq_storm;			/* disabled by the storm guard */
	u32 irq_empty;			/* consecutive IRQs with nothing to drain */
	bool fw_panicked;		/* 0xc7 seen since boot */
	u16 init_seq;			/* InitMessageBuffers core[0x70] */
	u16 fence_seq;			/* submission counter, fence[15:4] */
	struct apple_vxd_fence fences[APPLE_VXD_NUM_FENCES];
	struct apple_vxd_mbox_stats stats;

	/* R3b: command buffers, allocated and mapped before the MTX starts. */
	struct apple_vxd_cmdbuf_pool cmdbufs;

	/* Active job (R3d; decode jobs fail until the command stream lands). */
	spinlock_t job_lock;
	bool job_active;
	struct apple_vxd_job job;	/* under job_lock while job_active */
	struct work_struct job_done_work;
	struct delayed_work watchdog_work;

	/* V4L2 stateless device (R3/R4 skeleton). */
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex vdev_lock;		/* serializes ioctls */
	bool v4l2_ready;		/* set last, once every node is up */
	/*
	 * Coded formats, filled by apple_vxd_h264_register() during probe
	 * before any V4L2 node exists.  Round 2 never filled them (the
	 * variant table had no entry), so the first open() of /dev/video0
	 * read coded_fmts[0] at address 0.
	 */
	const struct apple_vxd_coded_fmt_desc *coded_fmts;
	unsigned int num_coded_fmts;
	struct dentry *debugfs_dir;	/* dumps, boot trigger, ping */
};

struct apple_vxd_ctx {
	struct v4l2_fh fh;
	struct apple_vxd_dev *vxd;
	struct v4l2_format coded_fmt;
	struct v4l2_format decoded_fmt;
	const struct apple_vxd_coded_fmt_desc *coded_fmt_desc;
	struct v4l2_ctrl_handler ctrl_hdl;
	enum apple_vxd_image_fmt image_fmt;
	void *priv;
};

struct apple_vxd_coded_fmt_ops {
	int (*adjust_fmt)(struct apple_vxd_ctx *ctx, struct v4l2_format *f);
	int (*start)(struct apple_vxd_ctx *ctx);
	void (*stop)(struct apple_vxd_ctx *ctx);
	int (*run)(struct apple_vxd_ctx *ctx);
	int (*try_ctrl)(struct apple_vxd_ctx *ctx, struct v4l2_ctrl *ctrl);
	enum apple_vxd_image_fmt (*get_image_fmt)(struct apple_vxd_ctx *ctx,
						  struct v4l2_ctrl *ctrl);
	/* Bytes of per-CAPTURE side buffer, 0 for none. */
	size_t (*capture_aux_size)(struct apple_vxd_ctx *ctx);
};

struct apple_vxd_decoded_fmt_desc {
	u32 fourcc;
	enum apple_vxd_image_fmt image_fmt;
};

struct apple_vxd_ctrl_desc {
	struct v4l2_ctrl_config cfg;
};

struct apple_vxd_ctrls {
	const struct apple_vxd_ctrl_desc *ctrls;
	unsigned int num_ctrls;
};

struct apple_vxd_coded_fmt_desc {
	u32 fourcc;
	struct v4l2_frmsize_stepwise frmsize;
	const struct apple_vxd_ctrls *ctrls;
	const struct apple_vxd_coded_fmt_ops *ops;
	unsigned int num_decoded_fmts;
	const struct apple_vxd_decoded_fmt_desc *decoded_fmts;
};

/* Message header word: (fence << 16) | (type << 8) | nwords. */
static inline u32 apple_vxd_msg_hdr(u16 fence, u8 type, u32 nwords)
{
	return FIELD_PREP(VXD_MSG_FENCE, fence) |
	       FIELD_PREP(VXD_MSG_TYPE, type) |
	       FIELD_PREP(VXD_MSG_NWORDS, nwords);
}

/* Register access (CPVDECCore::ReadReg/WriteReg [E], native endian). */
static inline void vxd_write(struct apple_vxd_dev *vxd, u32 off, u32 val)
{
	writel(val, vxd->regs + off);
}

static inline u32 vxd_read(struct apple_vxd_dev *vxd, u32 off)
{
	return readl(vxd->regs + off);
}

/* core.c */
irqreturn_t apple_vxd_irq(int irq, void *data);
void apple_vxd_watchdog(struct work_struct *work);
void apple_vxd_job_done(struct work_struct *work);
void apple_vxd_job_finish(struct apple_vxd_ctx *ctx,
			  enum vb2_buffer_state result);
void apple_vxd_job_finish_no_pm(struct apple_vxd_ctx *ctx,
				enum vb2_buffer_state result);
void apple_vxd_run_preamble(struct apple_vxd_ctx *ctx,
			    struct apple_vxd_run *run);
void apple_vxd_run_postamble(struct apple_vxd_ctx *ctx,
			     struct apple_vxd_run *run);
void apple_vxd_log_pwrstate(struct apple_vxd_dev *vxd, const char *when);
__printf(2, 3) void vxd_step(const struct device *dev, const char *fmt, ...);
__printf(2, 3) void vxd_step_io(const struct device *dev, const char *fmt, ...);

/* gart.c */
int apple_vxd_dma_alloc_aligned(struct apple_vxd_dev *vxd, size_t size,
				size_t align, struct apple_vxd_dma_buf *buf);
void apple_vxd_dma_free_aligned(struct apple_vxd_dev *vxd,
				struct apple_vxd_dma_buf *buf);
int apple_vxd_gart_init(struct apple_vxd_dev *vxd);
void apple_vxd_gart_fini(struct apple_vxd_dev *vxd);
int apple_vxd_gart_map(struct apple_vxd_dev *vxd, dma_addr_t dma, size_t size,
		       struct apple_vxd_mapping *map);
int apple_vxd_gart_map_at(struct apple_vxd_dev *vxd, dma_addr_t dma,
			  size_t size, u32 va_base,
			  struct apple_vxd_mapping *map);
void apple_vxd_gart_unmap(struct apple_vxd_dev *vxd,
			  struct apple_vxd_mapping *map);
int apple_vxd_dev_buf_alloc(struct apple_vxd_dev *vxd, size_t size,
			    struct apple_vxd_dev_buf *buf);
void apple_vxd_dev_buf_free(struct apple_vxd_dev *vxd,
			    struct apple_vxd_dev_buf *buf);

/* fw.c */
int apple_vxd_boot_firmware(struct apple_vxd_dev *vxd);
void apple_vxd_fw_cleanup(struct apple_vxd_dev *vxd);

/* mbox.c */
int apple_vxd_mbox_init(struct apple_vxd_dev *vxd);
void apple_vxd_mbox_fini(struct apple_vxd_dev *vxd);
int apple_vxd_mbox_open(struct apple_vxd_dev *vxd);
int apple_vxd_mbox_start(struct apple_vxd_dev *vxd);
void apple_vxd_mbox_stop(struct apple_vxd_dev *vxd);
int apple_vxd_msg_push(struct apple_vxd_dev *vxd, const u32 *msg,
		       struct apple_vxd_msg_ticket *ticket);
int apple_vxd_msg_wait_consumed(struct apple_vxd_dev *vxd,
				const struct apple_vxd_msg_ticket *ticket,
				unsigned int timeout_us, unsigned int *waited_us);
unsigned int apple_vxd_mbox_drain(struct apple_vxd_dev *vxd);
struct apple_vxd_fence *apple_vxd_fence_arm(struct apple_vxd_dev *vxd,
					    unsigned int packet, u8 type);
void apple_vxd_fence_release(struct apple_vxd_dev *vxd,
			     struct apple_vxd_fence *fence);
void apple_vxd_build_init_msg(u32 msg[VXD_INIT_MSG_WORDS], u32 fw_va,
			      size_t fw_bytes, u16 seq);
int apple_vxd_mbox_selftest(struct apple_vxd_dev *vxd);
int apple_vxd_submit(struct apple_vxd_ctx *ctx, struct apple_vxd_cmdbuf *cb);

/* cmdbuf.c */
int apple_vxd_cmdbufs_init(struct apple_vxd_dev *vxd);
void apple_vxd_cmdbufs_fini(struct apple_vxd_dev *vxd);
void apple_vxd_cmdbufs_reclaim(struct apple_vxd_dev *vxd);
struct apple_vxd_cmdbuf *apple_vxd_cmdbuf_get(struct apple_vxd_dev *vxd);
void apple_vxd_cmdbuf_put(struct apple_vxd_dev *vxd,
			  struct apple_vxd_cmdbuf *cb);
int apple_vxd_cmdbuf_bind(struct apple_vxd_dev *vxd,
			  struct apple_vxd_cmdbuf *cb,
			  struct apple_vxd_fence *fence);
void apple_vxd_cmdbuf_retire_locked(struct apple_vxd_cmdbuf *cb);
u32 apple_vxd_mmu_ctx_word(struct apple_vxd_dev *vxd, u8 hw_ctx);
int apple_vxd_packet_msg(struct apple_vxd_dev *vxd, u32 *pkt, u16 fence,
			 u8 hw_ctx, bool last, u32 msg[VXD_DECODE_MSG_WORDS]);
int apple_vxd_cmdbuf_selftest(struct apple_vxd_dev *vxd);
void apple_vxd_cmdbufs_debugfs(struct apple_vxd_dev *vxd);

/* v4l2.c */
u32 apple_vxd_buf_va(struct vb2_v4l2_buffer *vbuf);
u32 apple_vxd_buf_aux_va(struct vb2_v4l2_buffer *vbuf);
int apple_vxd_v4l2_init(struct apple_vxd_dev *vxd);
void apple_vxd_v4l2_cleanup(struct apple_vxd_dev *vxd);
extern const struct v4l2_ctrl_ops apple_vxd_ctrl_ops;

/* h264.c */
int apple_vxd_h264_register(struct apple_vxd_dev *vxd);

#endif /* __APPLE_VXD_H__ */

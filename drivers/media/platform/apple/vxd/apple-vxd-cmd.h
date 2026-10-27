/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple VXD - command-stream encoders (R3b, split out for R3c)
 *
 * The containers and encodings of docs/periph/ds-vxd-cmdstream.md, with
 * no MMIO, no DMA API and no locking, so the H.264 builder on top of them
 * also compiles on the host (scripts/vdec/host/) and its output can be
 * diffed word for word against the reference generator.
 */
#ifndef __APPLE_VXD_CMD_H__
#define __APPLE_VXD_CMD_H__

#include <linux/bitfield.h>
#include <linux/types.h>

#include "apple-vxd-regs.h"

struct apple_vxd_fence;

enum apple_vxd_cmdbuf_state {
	APPLE_VXD_CMDBUF_FREE = 0,
	APPLE_VXD_CMDBUF_HOST,
	APPLE_VXD_CMDBUF_DEVICE,
};

/*
 * One command buffer: the device-visible register program plus the
 * host-only packet list that the submit path turns into mailbox messages
 * (see apple-vxd.h for the pool and the ownership rules).
 */
struct apple_vxd_cmdbuf {
	unsigned int index;
	enum apple_vxd_cmdbuf_state state;	/* under mbox_lock */
	struct apple_vxd_fence *fence;		/* releases it, while DEVICE */

	/* Register program (device-visible). */
	u32 *vaddr;			/* 64 KiB aligned */
	dma_addr_t dma;
	u32 va;				/* GART device VA */
	size_t size;
	size_t used;			/* byte cursor, 4-byte granular */
	size_t chunk_start;		/* current chunk, 64 B aligned */
	bool group_open;		/* inside a 0x1 register group */
	u32 group_flags;		/* OR-ed into every run header */
	size_t run_hdr;			/* open run header, or SIZE_MAX */
	u32 run_next_reg;		/* register that extends the run */
	unsigned int npairs;		/* 0x000f0000 writes in this chunk */
	bool error;			/* sticky: overflow or misuse */

	/* Packet list (host-only). */
	u8 *pkt;
	size_t pkt_size;
	size_t pkt_used;
	unsigned int npackets;
};

void apple_vxd_cmdbuf_reset(struct apple_vxd_cmdbuf *cb);
void apple_vxd_cmd_word(struct apple_vxd_cmdbuf *cb, u32 word);
void apple_vxd_cmd_group_open(struct apple_vxd_cmdbuf *cb, u32 flags);
void apple_vxd_cmd_reg(struct apple_vxd_cmdbuf *cb, u32 reg, u32 val);
void apple_vxd_cmd_group_close(struct apple_vxd_cmdbuf *cb);
void apple_vxd_cmd_reg1(struct apple_vxd_cmdbuf *cb, u32 reg, u32 val,
			u32 flags);
void apple_vxd_cmd_slice_block(struct apple_vxd_cmdbuf *cb);
void apple_vxd_cmd_ctrl_alloc(struct apple_vxd_cmdbuf *cb,
			      const u32 words[VXD_CMD_SLICE_BLOCK_WORDS - 1]);
void apple_vxd_cmd_block(struct apple_vxd_cmdbuf *cb, u32 reg, u32 flags,
			 const u32 *vals, unsigned int n);
void apple_vxd_cmd_dma(struct apple_vxd_cmdbuf *cb, u32 type, u32 bytes,
		       u32 va);
void apple_vxd_cmd_slice_desc(struct apple_vxd_cmdbuf *cb, u32 w1, u32 w2);
void apple_vxd_cmd_load(struct apple_vxd_cmdbuf *cb, u32 va, u32 bytes);
void apple_vxd_cmd_table_xfer(struct apple_vxd_cmdbuf *cb, u32 bytes,
			      u32 va, u32 dst);
void apple_vxd_cmd_end_slice(struct apple_vxd_cmdbuf *cb);
int apple_vxd_cmdbuf_chunk_end(struct apple_vxd_cmdbuf *cb, u32 *offset,
			       u32 *bytes);
int apple_vxd_cmdbuf_chunk_end_exact(struct apple_vxd_cmdbuf *cb,
				     u32 *offset, u32 *bytes);
int apple_vxd_cmdbuf_pair_chunk_end(struct apple_vxd_cmdbuf *cb, u32 *offset,
				    u32 *bytes);
u32 *apple_vxd_cmdbuf_add_packet(struct apple_vxd_cmdbuf *cb, u32 magic,
				 u32 size);
int apple_vxd_cmdbuf_validate(const struct apple_vxd_cmdbuf *cb);

#endif /* __APPLE_VXD_CMD_H__ */

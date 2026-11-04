/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_APPLE_G5P_DRM_H_
#define _UAPI_APPLE_G5P_DRM_H_

#include "drm.h"

/* G5P merged winsys ABI. Retired ioctl numbers remain reserved. */
#define DRM_G5P_UAPI_VERSION 1
#define DRM_G5P_CAP_BO_CPU (1ULL << 0)
#define DRM_G5P_CAP_VM_PREPARE (1ULL << 1)
#define DRM_G5P_CAP_SYNCOBJ (1ULL << 2)
#define DRM_G5P_CAP_RENDER (1ULL << 5)
#define DRM_G5P_CAP_PRIME (1ULL << 6)
#define DRM_G5P_CAP_CODE_BASE_VERIFIED (1ULL << 7)
#define DRM_G5P_CAP_VM_LIVE (1ULL << 8)
#define DRM_G5P_CAP_CONTEXT_LIVE (1ULL << 9)
#define DRM_G5P_CAP_JOB_STATUS (1ULL << 10)
#define DRM_G5P_CAP_CONTEXT_STATUS (1ULL << 11)
#define DRM_G5P_CAP_FAULT_INFO (1ULL << 13)

/* LIVE means the owned user catalogue has completed firmware attachment. */
#define DRM_G5P_CONTEXT_LIVE 3
#define DRM_G5P_CONTEXT_HOST_ONLY 0
#define DRM_G5P_CONTEXT_LOST 1

struct drm_g5p_get_context_status {
	__u32 context;
	__u32 flags;
	__u32 state;
	__u32 map_count;
	__u64 mapped_bytes;
	__u64 reserved;
};

/* First terminal firmware event, retained independently of last_event. */
#define DRM_G5P_FAULT_NONE 0
#define DRM_G5P_FAULT_RECOVERY 1
#define DRM_G5P_FAULT_PAGE_REQUEST 2
#define DRM_G5P_FAULT_CHANNEL 3
#define DRM_G5P_FAULT_UNKNOWN 4
#define DRM_G5P_FAULT_VALID_CHANNEL (1U << 0)
struct drm_g5p_get_fault_info {
	__u32 flags;
	__u32 kind;
	__u64 event_serial; /* One-based arrival order; zero if no fault. */
	__u32 valid;
	__u32 channel_class; /* Event8: 0 memory failure, 1 killed, others opaque. */
	__u32 channel_token; /* Firmware token, not a DRM context handle. */
	__u32 channel_stamp;
	__u32 raw[11]; /* Includes unknown fields; no decoded fault VA is claimed. */
	__u32 reserved;
};

struct drm_g5p_get_info {
	__u32 size;
	__u32 flags;
	__u32 version;
	__u32 gpu_page_shift;
	__u64 capabilities;
	__u64 va_start;
	__u64 va_end;
	__u64 reserved_start;
	__u64 reserved_end;
	__u64 code_base; /* Zero until CODE_BASE_VERIFIED. */
	__u64 max_bo_size;
	__u32 bvnc[4]; /* Static target, not a new MMIO measurement. */
	__u64 reserved[2];
};

/* GPU access roles; CPU initialization/mmap is permitted for every role. */
#define DRM_G5P_BO_READ (1U << 0)
#define DRM_G5P_BO_WRITE (1U << 1)
#define DRM_G5P_BO_EXEC (1U << 2)
struct drm_g5p_create_bo {
	__u64 size;
	__u32 flags;
	__u32 handle;
};

struct drm_g5p_bo_mmap_offset {
	__u32 handle;
	__u32 flags;
	__u64 offset;
};

#define DRM_G5P_BO_SYNC_TO_DEVICE 0
#define DRM_G5P_BO_SYNC_TO_CPU 1
struct drm_g5p_bo_sync {
	__u32 handle;
	__u32 op;
	__u64 offset;
	__u64 size;
};

struct drm_g5p_create_vm {
	__u32 flags;
	__u32 handle;
};

struct drm_g5p_destroy_vm {
	__u32 handle;
	__u32 flags;
};

#define DRM_G5P_VM_MAP 0
#define DRM_G5P_VM_UNMAP 1
struct drm_g5p_vm_bind {
	__u32 vm;
	__u32 bo;
	__u64 va;
	__u64 size;
	__u64 offset;
	__u32 op;
	__u32 flags;
};

struct drm_g5p_create_context {
	__u32 vm;
	__u32 type;
	__u32 flags;
	__u32 handle;
};

struct drm_g5p_destroy_context {
	__u32 handle;
	__u32 flags;
};

/* A point of zero uses a binary syncobj; nonzero uses a timeline point. */
#define DRM_G5P_SYNC_SIGNAL (1U << 0)
struct drm_g5p_sync {
	__u32 handle;
	__u32 flags;
	__u64 point;
};

/* Privileged one-shot kernel fixture; all payload/BO fields must be zero.
 * The empty user context authorizes ownership only; its VM is not attached.
 * Does not advertise general COMPUTE or attach user-provided GPU addresses.
 */

/* Geometry is all zero/NONE while CODE_BASE_VERIFIED is clear. */
#define DRM_G5P_BASE_NONE 0
#define DRM_G5P_BASE_DEVICE 1
#define DRM_G5P_BASE_VM 2
#define DRM_G5P_BASE_CONTEXT 3
struct drm_g5p_get_exec_info {
	__u32 size;
	__u32 flags;
	__u32 base_scope;
	__u32 pad;
	__u64 usc_base;
	__u64 pds_base;
	__u64 window_size;
	__u64 allocation_alignment;
	__u32 usc_offset_bits;
	__u32 pds_offset_bits;
	__u64 reserved[3];
};

#define DRM_G5P_JOB_PENDING 0
#define DRM_G5P_JOB_COMPLETE 1
struct drm_g5p_get_job_status {
	__u64 job_id;
	__u32 flags;
	__u32 state;
	__s32 error; /* -EINPROGRESS while pending; immutable after completion. */
	__u32 pad;
	__u64 reset_generation; /* Zero: no reset/recovery support. */
};

#define DRM_G5P_GET_INFO 0x00
#define DRM_G5P_CREATE_BO 0x01
#define DRM_G5P_BO_MMAP_OFFSET 0x02
#define DRM_G5P_BO_SYNC 0x03
#define DRM_G5P_CREATE_VM 0x04
#define DRM_G5P_DESTROY_VM 0x05
#define DRM_G5P_VM_BIND 0x06
#define DRM_G5P_CREATE_CONTEXT 0x07
#define DRM_G5P_DESTROY_CONTEXT 0x08
#define DRM_G5P_GET_EXEC_INFO 0x0b
#define DRM_G5P_GET_JOB_STATUS 0x0c
#define DRM_G5P_GET_CONTEXT_STATUS 0x0d
#define DRM_G5P_GET_FAULT_INFO 0x0e

#define DRM_IOCTL_G5P_GET_INFO \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_GET_INFO, struct drm_g5p_get_info)
#define DRM_IOCTL_G5P_CREATE_BO \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_CREATE_BO, struct drm_g5p_create_bo)
#define DRM_IOCTL_G5P_BO_MMAP_OFFSET                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_BO_MMAP_OFFSET, \
		 struct drm_g5p_bo_mmap_offset)
#define DRM_IOCTL_G5P_BO_SYNC \
	DRM_IOW(DRM_COMMAND_BASE + DRM_G5P_BO_SYNC, struct drm_g5p_bo_sync)
#define DRM_IOCTL_G5P_CREATE_VM \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_CREATE_VM, struct drm_g5p_create_vm)
#define DRM_IOCTL_G5P_DESTROY_VM                       \
	DRM_IOW(DRM_COMMAND_BASE + DRM_G5P_DESTROY_VM, \
		struct drm_g5p_destroy_vm)
#define DRM_IOCTL_G5P_VM_BIND \
	DRM_IOW(DRM_COMMAND_BASE + DRM_G5P_VM_BIND, struct drm_g5p_vm_bind)
#define DRM_IOCTL_G5P_CREATE_CONTEXT                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_CREATE_CONTEXT, \
		 struct drm_g5p_create_context)
#define DRM_IOCTL_G5P_DESTROY_CONTEXT                       \
	DRM_IOW(DRM_COMMAND_BASE + DRM_G5P_DESTROY_CONTEXT, \
		struct drm_g5p_destroy_context)

#define DRM_IOCTL_G5P_GET_EXEC_INFO                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_GET_EXEC_INFO, \
		 struct drm_g5p_get_exec_info)
#define DRM_IOCTL_G5P_GET_JOB_STATUS                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_GET_JOB_STATUS, \
		 struct drm_g5p_get_job_status)

#define DRM_IOCTL_G5P_GET_CONTEXT_STATUS                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_GET_CONTEXT_STATUS, \
		 struct drm_g5p_get_context_status)

#define DRM_IOCTL_G5P_GET_FAULT_INFO                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_GET_FAULT_INFO, \
		 struct drm_g5p_get_fault_info)

#define DRM_G5P_CAP_IDLE_VM_BIND (1ULL << 18)

#define DRM_G5P_CAP_BOUNDED_RENDER (1ULL << 19)
#define DRM_G5P_CAP_TYPED_RENDER (1ULL << 20)
#define DRM_G5P_CAP_RENDER_USER_GEOMETRY (1ULL << 21)
#define DRM_G5P_CAP_RENDER_KERNEL_GEOMETRY (1ULL << 22)
#define DRM_G5P_CAP_TYPED_RENDER_V2 (1ULL << 23)
/* Bit 24 is reserved for COMPUTE_BLIT_V1 and bit 25 is FIXED_DATA_B1 on
 * a9/release, so the next bit free in every branch is 26.
 */
#define DRM_G5P_CAP_TYPED_RENDER_V3 (1ULL << 26)

/**
 * DRM_G5P_CAP_RENDER_MULTI_CONTEXT - several render contexts share the GPU
 *
 * Reported together with TYPED_RENDER_V3.  Any number of render contexts, in any
 * number of files, may submit TYPED_V3 jobs.  Each context has its own VM and its
 * own firmware slot pair, and the jobs of all contexts run one at a time in
 * submission order.  Without this bit, the first context to submit owns the GPU
 * until reboot and every other context is refused.  The other render profiles
 * stay single-context.
 */
#define DRM_G5P_CAP_RENDER_MULTI_CONTEXT (1ULL << 27)

/**
 * DRM_G5P_CAP_RENDER_UNPRIVILEGED - TYPED_V3 render needs no CAP_SYS_RAWIO
 *
 * Reported together with RENDER_MULTI_CONTEXT.  Any opener of the render node may
 * create render contexts and submit TYPED_V3 jobs.  A render VM then maps none of
 * the firmware's memory beyond the kernel-built window the GPU's own requesters
 * need, and the contexts close cleanly: a closed context's firmware binding is
 * removed and its memory freed.  Pinned memory is limited device-wide.  The other
 * render profiles, compute and the diagnostic ioctls still need CAP_SYS_RAWIO.
 */
#define DRM_G5P_CAP_RENDER_UNPRIVILEGED (1ULL << 28)

/**
 * DRM_G5P_CAP_RENDER_PIXEL_OUTPUT_WIDTH - TYPED_V3 carries pixel_output_width
 *
 * Reported together with TYPED_RENDER_V3.  struct drm_g5p_render_pass_v3 has
 * pixel_output_width, so a TYPED_V3 pass in the V1 dialect (all-zero ext) can
 * write several colour targets.  Without this bit the field is reserved and
 * must be zero.
 */
#define DRM_G5P_CAP_RENDER_PIXEL_OUTPUT_WIDTH (1ULL << 29)

/* drm_g5p_create_context.type; flags must be zero. */
#define DRM_G5P_CONTEXT_RENDER 1

#define DRM_G5P_RENDER_PROFILE_TYPED_V3 4

/* LINEAR64_V1: kind is also the resource's position in the canonical array. */

#define DRM_G5P_RENDER_ACCESS_READ (1U << 0)
#define DRM_G5P_RENDER_ACCESS_WRITE (1U << 1)
#define DRM_G5P_RENDER_ACCESS_IMPLICIT (1U << 2)
#define DRM_G5P_RENDER_ACCESS_CACHE (1U << 3)

#define DRM_G5P_RENDER_MAX_RESOURCES 256
#define DRM_G5P_RENDER_MAX_SYNCS 17
#define DRM_G5P_RENDER_MAX_VDM_BYTES 0x10000

/* A GPU VA range [va, va + size) inside one existing mapping of handle's BO
 * in the context VM, at BO byte offset `offset`. Never a CPU pointer, PA or
 * META address. LINEAR64_V1 requires offset 0 and the exact whole BO.
 */
struct drm_g5p_render_resource {
	__u32 handle;
	__u32 kind;
	__u64 offset;
	__u64 size;
	__u64 va;
};

/**
 * DRM_G5P_CAP_RENDER_MSAA - 4x multisampled render targets and passes
 *
 * Reported with TYPED_RENDER_V3 and RENDER_KERNEL_GEOMETRY.
 * CREATE_RENDER_TARGET accepts samples = 4 (kernel-derived geometry; with
 * DRM_G5P_RT_USER_GEOMETRY userspace may only restate it), and a TYPED_V3
 * pass carries samples, msaa_flags and sample_positions in struct
 * drm_g5p_render_pass. Without this bit those fields are reserved.
 */
#define DRM_G5P_CAP_RENDER_MSAA (1ULL << 31)

/* drm_g5p_render_pass.msaa_flags: the end of tile averages each pixel's
 * samples into the (one-sample) surface the EOT's PBE words name.
 */
#define DRM_G5P_RENDER_MSAA_RESOLVE (1U << 0)

struct drm_g5p_render_pass {
	__u32 size; /* sizeof(struct drm_g5p_render_pass) */
	__u32 flags; /* zero */
	__u64 vdm_va; /* VDM control stream, READ, 4-byte aligned */
	__u32 vdm_size; /* bytes, multiple of 4, <= MAX_VDM_BYTES */
	__u32 vdm_control; /* V1/V2: TA W+0x88 value, multiple of 16; V3: zero */
	__u64 scratch_va; /* TA scratch block, WRITE, 16-byte aligned */
	__u64 scratch_size;
	__u64 scratch_ptr[6]; /* TA scratch/marker pointers, in a WRITE range */
	__u64 pool7_va; /* 3D allocator pool, WRITE, page aligned */
	__u64 pool7_size;
	__u64 pool8_va; /* 3D allocator pool, WRITE, page aligned */
	__u64 pool8_size;
	__u32 context_pds[2][6]; /* TA context store/load PDS descriptors */
	__u32 resume_pds[2][4]; /* TA resume PDS descriptors */
	__u32 ta_pds[4]; /* TA state PDS descriptor; word 3 zero */
	__u32 background[8]; /* 3D background (load/clear) object */
	__u32 eot[4]; /* 3D end-of-tile object; word 3 zero */
	__u32 eot_resume[4]; /* 3D partial-render EOT; word 3 zero */
	__u32 restore[4]; /* 3D partial-render restore words */
	/* DRM_G5P_CAP_RENDER_MSAA, TYPED_V3 only; zero in every other profile.
	 * samples: 0 or 1 with a one-sample target, 4 with a four-sample one.
	 * sample_positions: one byte per sample, x in [3:0] and y in [7:4] in
	 * 1/16 px (PPP_MULTISAMPLECTL); 0 takes the target's standard pattern.
	 * The kernel writes the AA mode, ISP_CTL and output-partition words
	 * for the sample count. The draws' ISPCTL.upass (0x7 at four samples,
	 * 0xf at one), the PBE/EOT words and the fragment programs are
	 * userspace's, as for one sample; see docs/gpu/experiments/msaa.md.
	 */
	__u32 samples;
	__u32 msaa_flags; /* DRM_G5P_RENDER_MSAA_* */
	__u64 sample_positions;
	__u64 reserved[6];
};

/*
 * TYPED_V2 additions. Each field is a 3D register image that the firmware
 * uploads from the normal-3D work object. The CR offsets are the firmware
 * store targets (its 3D upload puts W3c0 on ROGUE_CR_ISP_CTL 0xf38); the
 * Rogue names are inferred, graded in docs/gpu/render-uapi.md.
 */
#define DRM_G5P_RENDER_MAX_TABLE_ENTRIES 65536 /* ISPDBSC 16-bit indices */
#define DRM_G5P_RENDER_ZLS_REGS 6

struct drm_g5p_render_pass_ext {
	/* ISP_SCISSOR_BASE (CR fb0): IPF_SCISSOR_WORD_0/1 pairs, 8 bytes each,
	 * indexed by TA_STATE_ISPDBSC.scindex. READ, 4-byte aligned.
	 */
	__u64 scissor_va;
	__u32 scissor_count; /* entries, 1..MAX_TABLE_ENTRIES */
	__u32 dbias_count; /* entries, 1..MAX_TABLE_ENTRIES */
	/* ISP_DBIAS_BASE (CR fb8): {constant, slope, clamp} f32 triples,
	 * 12 bytes each, indexed by ISPDBSC.dbindex. READ, 4-byte aligned.
	 */
	__u64 dbias_va;
	/* PDS_BGRND1_BASE (CR 6a8): background object texture/varying data
	 * (B-relative, like background[]); LOAD_OP_LOAD's texture state.
	 */
	__u64 background_texture;
	__u32 bgobj_depth; /* ISP_BGOBJDEPTH (CR f80): depth clear, ZLS format */
	__u32 bgobj_vals; /* ISP_BGOBJVALS (CR f88): stencil [7:0], mask [8], bgtag [9] */
	/* Pixel output width (2, 4 or 8): the 6ca7ae0 inputs to ISP_CTL's
	 * partition fields and the six per-core output words. 2 for one
	 * 32-bpp colour attachment (the captured value).
	 */
	__u32 pixel_output_width;
	/* Bit i: zls[i] is an address ([39:4]; [63:40] zero; [3:0] flags)
	 * inside a listed READ|WRITE range. The kernel does not decode ZLS.
	 */
	__u32 zls_address_mask;
	/* Opaque ZLS register images, CR f48, f50, f58, f60, f68 and fd0:
	 * Rogue ISP_ZLSCTL, ZLOAD/ZSTORE and STENCIL_LOAD/STORE bases plus
	 * one more; positions proven, order unqualified. Zero: no ZLS.
	 */
	__u64 zls[DRM_G5P_RENDER_ZLS_REGS];
	__u64 reserved[4];
};

struct drm_g5p_render_pass_v2 {
	struct drm_g5p_render_pass base;
	struct drm_g5p_render_pass_ext ext;
};

#define DRM_G5P_RENDER_MAX_PROGRAM_BYTES 0x1000
#define DRM_G5P_RENDER_STREAM_TERMINATE 0xc0000000U

struct drm_g5p_render_pass_v3 {
	struct drm_g5p_render_pass base;
	struct drm_g5p_render_pass_ext ext;
	__u64 ta_program_va; /* -> W+0x88 / R0+0x470 as an offset from pds_base */
	__u32 ta_program_size; /* bytes, 16..MAX_PROGRAM_BYTES, multiple of 16 */
	__u32 vdm_end; /* bytes from vdm_va past STREAM_TERMINATE, <= vdm_size */
	__u32 pixel_output_width; /* V1 dialect: 0 (= 2), 2, 4 or 8 */
	__u32 pad; /* zero */
	__u64 reserved[3];
};

struct drm_g5p_submit_render_typed {
	__u32 context;
	__u32 flags; /* zero */
	__u32 profile;
	__u32 render_target; /* handle from CREATE_RENDER_TARGET */
	__u64 resources; /* struct drm_g5p_render_resource[] */
	__u32 resource_count;
	__u32 sync_count;
	__u64 syncs; /* struct drm_g5p_sync[] */
	__u64 pass; /* struct drm_g5p_render_pass */
	__u32 pass_size;
	__u32 pad;
	__u64 job_id; /* output */
	__u64 reserved[2];
};

/*
 * Render-target memory geometry: the AGXRenderTargetMemoryG5 arithmetic
 * outputs (firmware round16 section 3, R-offsets in comments). The kernel
 * computes them for its verified table (64x64x1), or with
 * RENDER_KERNEL_GEOMETRY for any size; with RENDER_USER_GEOMETRY userspace
 * supplies them and the kernel bounds the memory they imply (and, with
 * RENDER_KERNEL_GEOMETRY, requires its own result).
 */
struct drm_g5p_rt_geometry {
	__u32 header_bytes; /* R10 */
	__u32 region_bytes; /* R20 */
	__u64 packed_state; /* Rb0 */
	__u32 state_word; /* Rb8 */
	__u32 padded_dims; /* Rbc */
	__u64 scale[3]; /* Rc0, Rc8, Rd0 packed float pairs */
	__u64 ta_state[2]; /* Rf0, Rf8 */
	__u64 ta_tiling[2]; /* R100, R108 */
	__u32 ta_tiling_word; /* R110 */
	__u32 tile_count; /* R118 */
	__u64 secondary_bytes; /* R138 */
	__u64 secondary_align; /* R178 */
	__u64 reserved[2];
};

#define DRM_G5P_RT_USER_GEOMETRY (1U << 0)
struct drm_g5p_create_render_target {
	__u32 context; /* render context; RT memory lives in its VM */
	__u32 flags;
	__u32 width;
	__u32 height;
	__u32 samples; /* 1, or 4 with DRM_G5P_CAP_RENDER_MSAA */
	__u32 layers; /* 1 */
	__u32 handle; /* output */
	__u32 pad;
	__u64 main_va; /* output: kernel-private GPU VA */
	__u64 main_bytes; /* output */
	__u64 secondary_va; /* output */
	__u64 secondary_bytes; /* output: mapped bytes */
	struct drm_g5p_rt_geometry geometry; /* input if USER, else output */
	__u32 pm_blocks; /* output with RENDER_KERNEL_GEOMETRY: context PM pool */
	__u32 pad2;
	__u64 reserved;
};

struct drm_g5p_destroy_render_target {
	__u32 handle;
	__u32 flags;
};

/* GET_RENDER_INFO.flags, output only (input must be zero).
 * VARIANT_ACTIVE: a render_user_* hypothesis knob (stage, cache/mmu/timestamp
 * variant, 3d_after_ta, pm_policy, cache_repeat, wait_ms, pm/memory/rt/slot
 * variant, profile1_vdm_control) differs from its default, so a context
 * created now latches a non-default job behaviour. GET_RENDER_INFO has no
 * context: it reports what CREATE_CONTEXT would latch at this moment.
 */
#define DRM_G5P_RENDER_INFO_VARIANT_ACTIVE (1U << 0)

struct drm_g5p_get_render_info {
	__u32 size; /* sizeof, exact */
	__u32 flags; /* in: zero; out: RENDER_INFO_* */
	__u32 profiles; /* bit N: profile N accepted */
	__u32 max_render_targets;
	__u64 private_va_start; /* kernel window in render VMs */
	__u64 private_va_end;
	__u64 user_va_min; /* lowest mappable VA in a render VM */
	__u64 usc_base; /* common USC/PDS base, 32-bit offsets */
	__u64 pds_base;
	__u64 code_window; /* bytes above usc_base usable for code */
	__u32 pm_blocks; /* parameter buffer blocks per context */
	__u32 pm_block_bytes;
	__u32 max_queued_jobs;
	__u32 max_vdm_bytes;
	__u32 job_timeout_ms;
	__u32 retained_results;
	/* RENDER_KERNEL_GEOMETRY only, else zero (64x64 only). */
	__u32 max_render_width;
	__u32 max_render_height;
	__u32 max_pm_blocks; /* PM pool growth bound per context */
	__u32 pad;
	union {
		__u64 reserved[2]; /* in: zero */
		struct {
			/* TYPED_V3: every GPU VA the TA/3D read ends at or below
			 * this; 0 makes no statement. Out only.
			 */
			__u64 hw_va_limit;
			__u64 reserved1;
		};
	};
};

#define DRM_G5P_CREATE_RENDER_TARGET 0x11
#define DRM_G5P_DESTROY_RENDER_TARGET 0x12
#define DRM_G5P_GET_RENDER_INFO 0x13
#define DRM_G5P_SUBMIT_RENDER_TYPED 0x14

#define DRM_IOCTL_G5P_SUBMIT_RENDER_TYPED                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_SUBMIT_RENDER_TYPED, \
		 struct drm_g5p_submit_render_typed)
#define DRM_IOCTL_G5P_CREATE_RENDER_TARGET                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_CREATE_RENDER_TARGET, \
		 struct drm_g5p_create_render_target)
#define DRM_IOCTL_G5P_DESTROY_RENDER_TARGET                       \
	DRM_IOW(DRM_COMMAND_BASE + DRM_G5P_DESTROY_RENDER_TARGET, \
		struct drm_g5p_destroy_render_target)
#define DRM_IOCTL_G5P_GET_RENDER_INFO                        \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_G5P_GET_RENDER_INFO, \
		 struct drm_g5p_get_render_info)

#endif

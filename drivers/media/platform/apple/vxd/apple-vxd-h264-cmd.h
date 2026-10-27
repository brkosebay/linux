/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple VXD - H.264 (AVC) command-stream builder (R3c)
 *
 * Pure function of its inputs: no allocation, no MMIO, no V4L2 core, so
 * it builds unchanged in the host harness (scripts/vdec/host/) and is
 * diffed word for word against tools/ds-video/cmdstream.py --gen.
 */
#ifndef __APPLE_VXD_H264_CMD_H__
#define __APPLE_VXD_H264_CMD_H__

#include <linux/types.h>
#include <linux/v4l2-controls.h>

#include "apple-vxd-cmd.h"
#include "apple-vxd-h264-parse.h"

/* Hardware DPB slot of a reference list entry; slot == V4L2 dpb[] index. */
#define APPLE_VXD_H264_NO_REF		0xff
#define APPLE_VXD_H264_UNUSED_VA	0xdeadbeef

/* The CAVLC table travels by CMD_DMA, 0x440 bytes, every slice. */
#define APPLE_VXD_H264_VLC_BYTES	0x440

/* Operating mode for 8-bit 4:2:0 frames (CMDS 0x7008, payload +0x08). */
#define APPLE_VXD_H264_OPMODE_BASE	0x10010000

/*
 * Element order of the scaling lists in the 0xe000 block.  The kext
 * uploads raster order (its parser stores list[zigzag[j]]); the others are
 * a device-side check of that reading (module parameter scaling_order).
 */
enum apple_vxd_h264_scaling_order {
	APPLE_VXD_H264_SCALING_RASTER = 0,
	APPLE_VXD_H264_SCALING_ZIGZAG,
	APPLE_VXD_H264_SCALING_TRANSPOSED,
};

/* Device VAs of the decode context's own buffers (DecodeCreateH264VLD). */
struct apple_vxd_h264_bufs {
	u32 cavlc;			/* 0x1000, the 0x440-byte VLC table */
	u32 intra;			/* IntraBuffer */
	u32 stream_state;		/* StreamStateBuffer -> message w5 */
};

struct apple_vxd_h264_frame {
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_decode_params *dp;
	/*
	 * Used when the PPS has V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
	 * flat (16) lists otherwise.
	 */
	const struct v4l2_ctrl_h264_scaling_matrix *scaling;
	const struct apple_vxd_h264_slice *slices;
	unsigned int nslices;
	/* Per slice: final RefPicList0/1 as DPB slots, NO_REF past the end. */
	const u8 (*ref_slots)[2][V4L2_H264_REF_LIST_LEN];

	u32 src_va;			/* OUTPUT buffer, byte 0 */
	u32 dst_luma, dst_chroma;	/* CAPTURE planes */
	u32 dst_stride;			/* bytes, luma and chroma (NV12) */
	u32 colocated;			/* CAPTURE's stored-MB parameter buffer */
	u32 dpb_luma[V4L2_H264_NUM_DPB_ENTRIES];	/* 0: not resolvable */
	u32 dpb_chroma[V4L2_H264_NUM_DPB_ENTRIES];
	/* Each reference's stored-MB buffer: the B co-located picture's. */
	u32 dpb_colocated[V4L2_H264_NUM_DPB_ENTRIES];
	struct apple_vxd_h264_bufs bufs;
	bool first_picture;		/* first picture of the context */
	enum apple_vxd_h264_scaling_order scaling_order;
};

int apple_vxd_h264_validate(const struct v4l2_ctrl_h264_sps *sps,
			    const struct v4l2_ctrl_h264_pps *pps,
			    const struct v4l2_ctrl_h264_scaling_matrix *scaling,
			    const char **why);
int apple_vxd_h264_build(struct apple_vxd_cmdbuf *cb,
			 const struct apple_vxd_h264_frame *f,
			 const char **why);
u32 apple_vxd_h264_colocated_size(unsigned int width_mbs,
				  unsigned int height_mbs);

#endif /* __APPLE_VXD_H264_CMD_H__ */

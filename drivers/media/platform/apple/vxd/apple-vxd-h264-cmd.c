// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - H.264 (AVC) command-stream builder (R3c)
 *
 * The AVC path of the kext (CH264Decoder::HOST_H264DecodeFrame
 * 0xfffffff0062a15f0 -> DecodePictureInternal 0x62a0c00 ->
 * DecodeExecuteH264VLD 0x62c4c98, docs/periph/ds-vxd-cmdstream.md section
 * 16.3-16.6) builds one command chunk and one kind-1 packet (mailbox
 * 0x81) per slice, in Imagination's DXVA control-allocation format
 * (psb_video's dxva_fw_ctrl.h):
 *
 *   CTRL_ALLOC_HEADER        0x90000000 + 9 words (slice params, strides,
 *                            first/last MB, chroma format)
 *   CMD_DMA                  the 0x440-byte CAVLC table, type VLC
 *   REGVALPAIR               26 constant VLC table address registers
 *   SR_SETUP + BITSTREAM_DMA the slice NAL from the OUTPUT buffer, with
 *                            the core stripping emulation prevention
 *   REGVALPAIR               entropy front end: control, SPS0, PPS0, PIC0,
 *                            SLICE0, SLICE1, first MB
 *   RENDEC_BLOCK x n         entropy back end, scaling lists, B-slice POCs
 *                            and co-located picture, reference lists,
 *                            reference surfaces, prediction weights,
 *                            picture geometry, intra buffer, cache policy,
 *                            stride
 *   CMD_COMPLETION           0x60000000
 *
 * The chunks follow each other with no pad; there is no per-picture
 * (kind-4) chunk in the AVC path.  Apple's register numbers map onto the
 * public MSVDX layout as 0x6000 + off = VEC + off and 0x7000 + off =
 * CMDS + off (FE_SPS0 0x200, BE_FOC0 0x250, BE_SPS0 0x2d8,
 * BE_BASE_ADDR_CUR_PIC 0x2ec, BE_REF0 0x2f0, BE_COL_PIC0 0x2f4,
 * BE_LIST0 0x33c, H264_WEIGHTED_FACTORS_A 0x200, VLC_TABLE_ADDR0 0xec,
 * ENTDEC_FE/BE_CONTROL 0xc/0x10).
 *
 * Supported here, by apple_vxd_h264_validate() and the slice checks:
 * progressive frames, 8-bit 4:2:0, I, P and B slices, CAVLC and CABAC,
 * 8x8 transform and scaling matrices (High profile), explicit and implicit
 * weighted prediction.  Not supported: interlace (field pictures, MBAFF),
 * FMO/ASO, SP/SI slices, the second deblock pass, alternate output.
 */
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "apple-vxd-h264-cmd.h"

/* Entropy decoder (Apple 0x6000 bank = MSVDX VEC). */
#define VXD_H264_FE_CONTROL		0x600c
#define VXD_H264_BE_CONTROL		0x6010
#define VXD_H264_VLC_ADDR0		0x60ec	/* + 22 */
#define VXD_H264_VLC_ADDR_HI		0x61c0	/* + 2, the 4:2:2 tables */
#define VXD_H264_FE_SPS0		0x6200	/* SPS0 PPS0 PIC0 SLICE0 SLICE1 */
#define VXD_H264_FE_SLICE2		0x6218
#define VXD_H264_BE_FOC0		0x6250	/* current, then per DPB slot */
#define VXD_H264_BE_SPS0		0x62d8	/* ... SLICE1, CUR_PIC, REF0 */
#define VXD_H264_BE_COL_PIC0		0x62f4	/* COL_PIC, LIST0_INVERSE, LIST1 */
#define VXD_H264_BE_L0			0x633c
#define VXD_H264_SCALING		0xe000
/* Render commands (Apple 0x7000 bank = MSVDX CMDS). */
#define VXD_H264_CMDS_SIZE		0x7000	/* display, coded, mode, Y, UV */
#define VXD_H264_CMDS_INTRA		0x7018
#define VXD_H264_CMDS_CACHE		0x701c
#define VXD_H264_CMDS_WEIGHT_DENOM	0x7024
#define VXD_H264_CMDS_STRIDE		0x7040
#define VXD_H264_CMDS_REFS		0x7100	/* Y, UV per DPB slot */
#define VXD_H264_CMDS_CHROMA		0x71b4
#define VXD_H264_CMDS_WEIGHTS(list)	(0x7200 + 0x100 * (list))

/* Apple's forced profile 3 translates to 0x11 on both ends (0x62c6564). */
#define VXD_H264_ENTDEC_CONTROL		0x11
#define VXD_H264_CACHE_POLICY		0x013c000c	/* 0x62c4788 */
#define VXD_H264_WEIGHTS_DEFAULT	0x55		/* 5 / 5 */
#define VXD_H264_SCALING_WORDS		56		/* 2 x 8x8 + 6 x 4x4 */
#define VXD_H264_SCALING_FLAT		0x10
#define VXD_H264_FIELD_FRAME		2		/* progressive */
#define VXD_H264_WEIGHT_WORDS		64		/* 32 factors, 32 offsets */

/* Slice codes of the core: I 0, P 1, B 2 (0xfffffff007a6e7b4). */
#define VXD_H264_CODE_I			0
#define VXD_H264_CODE_P			1
#define VXD_H264_CODE_B			2

/* The 26 constant VLC table registers (0xfffffff005597148 [E]). */
static const u32 vxd_h264_vlc_addr[] = {
	0x00026000, 0x000738a0, 0x000828f4, 0x000a312d, 0x000b5959,
	0x000c517b, 0x000d1196, 0x000db1ad, 0x000e21be, 0x000e59c8,
	0x000e79cd, 0x000eb1d3, 0x000ed1d8, 0x000f09dd, 0x000f71e7,
	0x001041f6, 0x1256a4dd, 0x01489292, 0x11248050, 0x000022a2,
	0x00002a02, 0x8108282a, 0x00000012,
};

static const u32 vxd_h264_vlc_addr_hi[] = {
	0x0010920e, 0x0010d216, 0x0010f21c,
};

/*
 * The scaling-list block (0x62c66b0 L39756-39766 of the dump): Inter Y and
 * Intra Y 8x8, then the 4x4 lists in this order of the H.264 list index
 * (Intra Y, Inter Y, Inter Cb, Intra Cb, Inter Cr, Intra Cr), each in
 * raster order: AppleD5500's scaling_list() parser (0xfffffff00629c918)
 * stores list[zigzag[i]], and its defaults (0xfffffff005596170) are raster
 * tables, which is also V4L2's order.
 */
static const u8 vxd_h264_scaling_4x4_order[6] = { 0, 3, 4, 1, 5, 2 };

/* Raster position of the i-th coefficient in frame zig-zag scan. */
static const u8 vxd_h264_zigzag4[16] = {
	0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15,
};

static const u8 vxd_h264_zigzag8[64] = {
	0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* One raster list of @n x @n in the requested order. */
static void vxd_h264_scaling_list(u8 *dst, const u8 *raster, unsigned int n,
				  enum apple_vxd_h264_scaling_order order)
{
	const u8 *zz = n == 4 ? vxd_h264_zigzag4 : vxd_h264_zigzag8;
	unsigned int i;

	for (i = 0; i < n * n; i++) {
		switch (order) {
		case APPLE_VXD_H264_SCALING_ZIGZAG:
			dst[i] = raster[zz[i]];
			break;
		case APPLE_VXD_H264_SCALING_TRANSPOSED:
			dst[i] = raster[(i % n) * n + i / n];
			break;
		default:
			dst[i] = raster[i];
			break;
		}
	}
}

static int vxd_h264_slice_code(u8 slice_type)
{
	switch (slice_type) {
	case V4L2_H264_SLICE_TYPE_I:
		return VXD_H264_CODE_I;
	case V4L2_H264_SLICE_TYPE_P:
		return VXD_H264_CODE_P;
	case V4L2_H264_SLICE_TYPE_B:
		return VXD_H264_CODE_B;
	default:
		return -EINVAL;
	}
}

u32 apple_vxd_h264_colocated_size(unsigned int width_mbs,
				  unsigned int height_mbs)
{
	/* (W * H * 0x80 + 0x41ff) & ~0xfff, 0xfffffff0062c3b8c [E] */
	return (width_mbs * height_mbs * 0x80 + 0x41ff) & ~0xfffU;
}

static bool vxd_h264_has_scaling(const struct v4l2_ctrl_h264_pps *pps)
{
	return pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
}

/* A scaling factor is 1..255: a zero list means the control was never set. */
static bool vxd_h264_scaling_set(const struct v4l2_ctrl_h264_pps *pps,
				 const struct v4l2_ctrl_h264_scaling_matrix *m)
{
	unsigned int i, j;

	if (!m)
		return false;
	for (i = 0; i < 6; i++)
		for (j = 0; j < 16; j++)
			if (!m->scaling_list_4x4[i][j])
				return false;
	if (!(pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE))
		return true;
	for (i = 0; i < 2; i++)
		for (j = 0; j < 64; j++)
			if (!m->scaling_list_8x8[i][j])
				return false;
	return true;
}

int apple_vxd_h264_validate(const struct v4l2_ctrl_h264_sps *sps,
			    const struct v4l2_ctrl_h264_pps *pps,
			    const struct v4l2_ctrl_h264_scaling_matrix *scaling,
			    const char **why)
{
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		*why = "interlaced coding (frame_mbs_only_flag = 0)";
	else if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		*why = "separate colour planes";
	else if (V4L2_H264_SPS_HAS_CHROMA_FORMAT(sps) && sps->chroma_format_idc != 1)
		*why = "chroma format other than 4:2:0";
	else if (sps->bit_depth_luma_minus8 || sps->bit_depth_chroma_minus8)
		*why = "bit depth above 8";
	else if (sps->flags & V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS)
		*why = "lossless transform bypass";
	else if (pps->num_slice_groups_minus1)
		*why = "slice groups (FMO)";
	else if (pps->weighted_bipred_idc > 2)
		*why = "weighted_bipred_idc out of range";
	else if (vxd_h264_has_scaling(pps) && !vxd_h264_scaling_set(pps, scaling))
		*why = "scaling matrices signalled but the SCALING_MATRIX control is not set";
	else
		return 0;
	return -EINVAL;
}

/* Bits of the slice NAL before slice_data(), escaped, NAL header included. */
static u32 vxd_h264_header_bits(const struct v4l2_ctrl_h264_pps *pps,
				const struct apple_vxd_h264_slice *sl)
{
	u32 bits = 8 + sl->header_bits_raw;

	/* CABAC slice data starts after cabac_alignment_one_bit padding. */
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		bits = ALIGN(bits, 8);
	return bits;
}

/* POC registers: 17 bits, sign in bit 16 (0x62c66b0 L39779-39792). */
static u32 vxd_h264_poc(s32 poc)
{
	return ((u32)poc >> 15 & 0x10000) | ((u32)poc & 0xffff);
}

/* Explicit weights in the bitstream (the kext's 0x62be348 condition). */
static bool vxd_h264_explicit_weights(const struct v4l2_ctrl_h264_pps *pps,
				      int code)
{
	return ((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) &&
		code == VXD_H264_CODE_P) ||
	       (pps->weighted_bipred_idc == 1 && code == VXD_H264_CODE_B);
}

/* The weight tables go out for explicit and implicit modes (L40086). */
static bool vxd_h264_weight_tables(const struct v4l2_ctrl_h264_pps *pps,
				   int code)
{
	return ((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) &&
		code == VXD_H264_CODE_P) ||
	       (pps->weighted_bipred_idc && code == VXD_H264_CODE_B);
}

static void vxd_h264_scaling_words(const struct apple_vxd_h264_frame *f,
				   u32 *w)
{
	const struct v4l2_ctrl_h264_scaling_matrix *m = f->scaling;
	u8 bytes[4 * VXD_H264_SCALING_WORDS];
	unsigned int i;

	memset(bytes, VXD_H264_SCALING_FLAT, sizeof(bytes));
	if (vxd_h264_has_scaling(f->pps) && m) {
		/* 8x8 lists only with the 8x8 transform (0x62be538 L33469). */
		if (f->pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) {
			vxd_h264_scaling_list(&bytes[0], m->scaling_list_8x8[1], 8,
					      f->scaling_order);
			vxd_h264_scaling_list(&bytes[64], m->scaling_list_8x8[0], 8,
					      f->scaling_order);
		}
		for (i = 0; i < 6; i++)
			vxd_h264_scaling_list(&bytes[128 + 16 * i],
					      m->scaling_list_4x4[vxd_h264_scaling_4x4_order[i]],
					      4, f->scaling_order);
	}
	for (i = 0; i < VXD_H264_SCALING_WORDS; i++)
		w[i] = bytes[4 * i] | bytes[4 * i + 1] << 8 |
		       bytes[4 * i + 2] << 16 | (u32)bytes[4 * i + 3] << 24;
}

/* @n bytes of a reference list, four per word, low byte first. */
static unsigned int vxd_h264_pack_list(const u8 *list, unsigned int n, u32 *w)
{
	unsigned int i, words = DIV_ROUND_UP(n, 4);

	for (i = 0; i < words; i++)
		w[i] = 0;
	for (i = 0; i < 4 * words; i++)
		w[i / 4] |= (u32)list[i] << (8 * (i % 4));
	return words;
}

static bool vxd_h264_dpb_active(const struct v4l2_h264_dpb_entry *e)
{
	return (e->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) &&
	       (e->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE);
}

/*
 * B slices: the picture order counts (BE_FOC0/1, then TOP/BOT_FOC per
 * hardware DPB slot) and the co-located picture block (BE_COL_PIC0,
 * BE_BASE_ADDR_COL_PIC, BE_LIST0_INVERSE x 8, BE_LIST1), 0x62c66b0
 * L39776-39887.  The co-located picture is RefPicList1[0].
 */
static void vxd_h264_emit_b(struct apple_vxd_cmdbuf *cb,
			    const struct apple_vxd_h264_frame *f,
			    const u8 *l0, const u8 *l1, u8 l1_minus1,
			    unsigned int highwater, u32 *w)
{
	const struct v4l2_ctrl_h264_decode_params *dp = f->dp;
	u8 inverse[V4L2_H264_REF_LIST_LEN];
	unsigned int i, n;
	u8 col = l1[0];

	w[0] = vxd_h264_poc(dp->top_field_order_cnt);
	w[1] = vxd_h264_poc(dp->bottom_field_order_cnt);
	for (i = 0; i < highwater; i++) {
		const struct v4l2_h264_dpb_entry *e = &dp->dpb[i];
		bool active = vxd_h264_dpb_active(e);

		w[2 + 2 * i] = active ? vxd_h264_poc(e->top_field_order_cnt) : 0;
		w[3 + 2 * i] = active ? vxd_h264_poc(e->bottom_field_order_cnt) : 0;
	}
	apple_vxd_cmd_block(cb, VXD_H264_BE_FOC0, 0, w, 2 + 2 * highwater);

	if (col < V4L2_H264_NUM_DPB_ENTRIES && f->dpb_colocated[col]) {
		const struct v4l2_h264_dpb_entry *e = &dp->dpb[col];
		s32 cur = min(dp->top_field_order_cnt, dp->bottom_field_order_cnt);
		s64 dtop = abs((s64)cur - e->top_field_order_cnt);
		s64 dbot = abs((s64)cur - e->bottom_field_order_cnt);

		/*
		 * Frame co-located picture: COL_NOTFRAME and COL_MBAFF clear,
		 * bottom field flag by POC distance, as psb_video and the kext.
		 */
		w[0] = dbot <= dtop;
		w[1] = f->dpb_colocated[col];
	} else {
		w[0] = APPLE_VXD_H264_UNUSED_VA;
		w[1] = APPLE_VXD_H264_UNUSED_VA;
	}

	/* Inverse L0: first position of each slot (bottom fields at +16). */
	memset(inverse, APPLE_VXD_H264_NO_REF, sizeof(inverse));
	for (i = V4L2_H264_REF_LIST_LEN; i--; )
		if (l0[i] != APPLE_VXD_H264_NO_REF)
			inverse[(l0[i] >> 3 & 0x10) | (l0[i] & 0x1f)] = i;
	vxd_h264_pack_list(inverse, V4L2_H264_REF_LIST_LEN, &w[2]);
	n = vxd_h264_pack_list(l1, l1_minus1 + 1, &w[10]);
	apple_vxd_cmd_block(cb, VXD_H264_BE_COL_PIC0, 0, w, 10 + n);
}

/*
 * H264_WEIGHTED_FACTORS/OFFSET A (list 0) and B (list 1), 32 entries each
 * (0x62c66b0 L40086-40134, field layout as psb_video's msvdx_cmds_io2.h).
 * Implicit bi-prediction leaves the kext's tables unrefreshed (zero in a
 * fresh context); the core derives those weights from the POCs.
 */
static void vxd_h264_emit_weights(struct apple_vxd_cmdbuf *cb,
				  const struct apple_vxd_h264_frame *f,
				  const struct apple_vxd_h264_slice *sl,
				  const u8 *ref_slots, unsigned int list,
				  u8 n_minus1, int code, u32 *w)
{
	bool expl = vxd_h264_explicit_weights(f->pps, code);
	unsigned int i;

	memset(w, 0, VXD_H264_WEIGHT_WORDS * sizeof(*w));
	for (i = 0; expl && i <= n_minus1; i++) {
		const struct apple_vxd_h264_weight *wt = &sl->weights[list][i];

		if (ref_slots[i] == APPLE_VXD_H264_NO_REF)
			continue;
		w[i] = ((u32)wt->weight[2] & 0x1ff) << 18 |
		       ((u32)wt->weight[1] & 0x1ff) << 9 |
		       ((u32)wt->weight[0] & 0x1ff);
		w[32 + i] = (u32)(u8)wt->offset[2] << 16 |
			    (u32)(u8)wt->offset[1] << 8 | (u8)wt->offset[0];
	}
	apple_vxd_cmd_block(cb, VXD_H264_CMDS_WEIGHTS(list), 0, w,
			    VXD_H264_WEIGHT_WORDS);
}

static void vxd_h264_emit_slice(struct apple_vxd_cmdbuf *cb,
				const struct apple_vxd_h264_frame *f,
				unsigned int idx, int code)
{
	const struct v4l2_ctrl_h264_sps *sps = f->sps;
	const struct v4l2_ctrl_h264_pps *pps = f->pps;
	const struct apple_vxd_h264_slice *sl = &f->slices[idx];
	const u8 *l0 = f->ref_slots[idx][0];
	const u8 *l1 = f->ref_slots[idx][1];
	u32 w = sps->pic_width_in_mbs_minus1 + 1;
	u32 h = sps->pic_height_in_map_units_minus1 + 1;
	u32 mb_x = sl->first_mb_in_slice % w, mb_y = sl->first_mb_in_slice / w;
	u32 chroma = V4L2_H264_SPS_HAS_CHROMA_FORMAT(sps) ? sps->chroma_format_idc : 1;
	u32 wmode = !!(pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) |
		    pps->weighted_bipred_idc << 1;
	u32 hdr[VXD_CMD_SLICE_BLOCK_WORDS - 1] = { };
	u32 sps0, pps0, pic0, slice0, slice1, skip, bits, longterm = 0;
	unsigned int i, n, highwater = 0;
	bool min_bipred8, used[V4L2_H264_NUM_DPB_ENTRIES] = { };
	/*
	 * I slices zero both counts; P slices keep the parsed L1 count (the
	 * PPS default, 0x62a0058) as the kext does (0x62c5fb8 L39477-39483).
	 */
	u8 l0_minus1 = code ? sl->num_ref_idx_l0_active_minus1 : 0;
	u8 l1_minus1 = code ? sl->num_ref_idx_l1_active_minus1 : 0;
	/* Scratch for one block at a time: the largest is the weight table. */
	u32 buf[VXD_H264_WEIGHT_WORDS];

	min_bipred8 = w > 80 && sps->level_idc > 30;
	sps0 = ((chroma >> 1) & 1) << 28 | sps->bit_depth_chroma_minus8 << 25 |
	       sps->bit_depth_luma_minus8 << 22 | 1 << 14 | 3 << 12 |
	       min_bipred8 << 11 |
	       !!(sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE) << 10 |
	       (chroma & 1) << 9 |
	       !!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) << 8 | (w - 1);
	pps0 = ((u8)pps->second_chroma_qp_index_offset & 31) << 24 |
	       ((u8)pps->chroma_qp_index_offset & 31) << 16 |
	       pps->weighted_bipred_idc << 8 |
	       pps->num_slice_groups_minus1 << 4 |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) << 2 |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 1 |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE);
	pic0 = (w * h - 1) << 16 | (h - 1) << 8 | !!f->dp->nal_ref_idc << 3;
	slice0 = sl->direct_spatial_mv_pred << 31 |
		 sl->disable_deblocking_filter_idc << 28 |
		 ((u8)sl->slice_alpha_c0_offset_div2 & 15) << 24 |
		 ((u8)sl->slice_beta_offset_div2 & 15) << 20 |
		 VXD_H264_FIELD_FRAME << 18 | code << 16 |
		 sl->cabac_init_idc << 14 | (idx & 0x3fff);
	slice1 = mb_y << 24 | mb_x << 16 |
		 ((26 + pps->pic_init_qp_minus26 + sl->slice_qp_delta) & 63) << 10 |
		 (l1_minus1 & 31) << 5 | (l0_minus1 & 31);

	/* CTRL_ALLOC_HEADER; MODE_CONFIG = weighted_pred | bipred_idc << 1. */
	hdr[0] = !!(pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 23 |
		 wmode << 17 |
		 sl->disable_deblocking_filter_idc << 12 |
		 ((u8)sl->slice_alpha_c0_offset_div2 & 15) << 8 |
		 ((u8)sl->slice_beta_offset_div2 & 15) << 4 |
		 VXD_H264_FIELD_FRAME << 2 | code;
	hdr[2] = f->dst_stride | f->dst_stride << 16;
	hdr[3] = mb_x | mb_y << 16;
	hdr[4] = (w - 1) | (h - 1) << 16;
	hdr[8] = sps->bit_depth_chroma_minus8 << 12 |
		 sps->bit_depth_luma_minus8 << 8 | chroma;
	apple_vxd_cmd_ctrl_alloc(cb, hdr);

	/* CAVLC table + its address registers, every slice. */
	apple_vxd_cmd_dma(cb, VXD_CMD_DMA_TYPE_VLC, APPLE_VXD_H264_VLC_BYTES,
			  f->bufs.cavlc);
	apple_vxd_cmd_group_open(cb, 0);
	for (i = 0; i < ARRAY_SIZE(vxd_h264_vlc_addr); i++)
		apple_vxd_cmd_reg(cb, VXD_H264_VLC_ADDR0 + 4 * i,
				  vxd_h264_vlc_addr[i]);
	for (i = 0; i < ARRAY_SIZE(vxd_h264_vlc_addr_hi); i++)
		apple_vxd_cmd_reg(cb, VXD_H264_VLC_ADDR_HI + 4 * i,
				  vxd_h264_vlc_addr_hi[i]);
	apple_vxd_cmd_group_close(cb);

	/*
	 * The slice NAL from its header byte: whole 32-bit words of header go
	 * by DMA offset, the rest by the SR bit offset (0x62c5b18 [E]).
	 */
	bits = vxd_h264_header_bits(pps, sl);
	skip = (bits & 0xffe0) >> 3;
	apple_vxd_cmd_slice_desc(cb, bits & 31, sl->nal_size - skip);
	apple_vxd_cmd_load(cb, f->src_va + sl->nal_offset + skip,
			   sl->nal_size - skip);

	/* Entropy front end. */
	apple_vxd_cmd_group_open(cb, 0);
	apple_vxd_cmd_reg(cb, VXD_H264_FE_CONTROL, VXD_H264_ENTDEC_CONTROL);
	apple_vxd_cmd_reg(cb, VXD_H264_FE_SPS0, sps0);
	apple_vxd_cmd_reg(cb, VXD_H264_FE_SPS0 + 4, pps0);
	apple_vxd_cmd_reg(cb, VXD_H264_FE_SPS0 + 8, pic0);
	apple_vxd_cmd_reg(cb, VXD_H264_FE_SPS0 + 12, slice0);
	apple_vxd_cmd_reg(cb, VXD_H264_FE_SPS0 + 16, slice1);
	apple_vxd_cmd_reg(cb, VXD_H264_FE_SLICE2, sl->first_mb_in_slice << 16);
	apple_vxd_cmd_group_close(cb);

	/* Entropy back end. */
	buf[0] = VXD_H264_ENTDEC_CONTROL;
	apple_vxd_cmd_block(cb, VXD_H264_BE_CONTROL, 0, buf, 1);

	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		const struct v4l2_h264_dpb_entry *e = &f->dp->dpb[i];

		if (!vxd_h264_dpb_active(e))
			continue;
		highwater = i + 1;
		if (e->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
			longterm |= BIT(i);
	}
	buf[0] = sps0;
	buf[1] = pps0;
	buf[2] = pic0;
	buf[3] = slice0;
	buf[4] = slice1;
	buf[5] = f->colocated;
	buf[6] = longterm;
	apple_vxd_cmd_block(cb, VXD_H264_BE_SPS0, 0, buf, 7);

	vxd_h264_scaling_words(f, buf);
	apple_vxd_cmd_block(cb, VXD_H264_SCALING, 0, buf,
			    VXD_H264_SCALING_WORDS);

	if (code == VXD_H264_CODE_B)
		vxd_h264_emit_b(cb, f, l0, l1, l1_minus1, highwater, buf);

	if (code) {
		n = vxd_h264_pack_list(l0, l0_minus1 + 1, buf);
		apple_vxd_cmd_block(cb, VXD_H264_BE_L0, 0, buf, n);
	}

	/* Surfaces only for the references this slice's lists use. */
	for (i = 0; code && i <= l0_minus1; i++)
		if (l0[i] < V4L2_H264_NUM_DPB_ENTRIES)
			used[l0[i]] = true;
	for (i = 0; code == VXD_H264_CODE_B && i <= l1_minus1; i++)
		if (l1[i] < V4L2_H264_NUM_DPB_ENTRIES)
			used[l1[i]] = true;
	if (highwater) {
		for (i = 0; i < highwater; i++) {
			bool real = used[i] && f->dpb_luma[i];

			buf[2 * i] = real ? f->dpb_luma[i] : APPLE_VXD_H264_UNUSED_VA;
			buf[2 * i + 1] = real ? f->dpb_chroma[i] :
						APPLE_VXD_H264_UNUSED_VA;
		}
		apple_vxd_cmd_block(cb, VXD_H264_CMDS_REFS, VXD_CMD_BLOCK_FLAG_REFS,
				    buf, 2 * highwater);
	}

	if (vxd_h264_weight_tables(pps, code)) {
		vxd_h264_emit_weights(cb, f, sl, l0, 0, l0_minus1, code, buf);
		if (code == VXD_H264_CODE_B)
			vxd_h264_emit_weights(cb, f, sl, l1, 1, l1_minus1, code,
					      buf);
	}

	/* Picture geometry, operating mode, output. */
	buf[0] = (16 * h - 1) << 12 | (16 * w - 1);
	buf[1] = buf[0];
	buf[2] = APPLE_VXD_H264_OPMODE_BASE | 2 << 20 | (chroma != 0) << 12;
	buf[3] = f->dst_luma;
	buf[4] = f->dst_chroma;
	apple_vxd_cmd_block(cb, VXD_H264_CMDS_SIZE, 0, buf, 5);
	buf[0] = f->bufs.intra;
	apple_vxd_cmd_block(cb, VXD_H264_CMDS_INTRA, 0, buf, 1);
	/* Denominators: the slice's when explicit, 5 / 5 otherwise. */
	buf[0] = vxd_h264_explicit_weights(pps, code) ?
		 sl->chroma_log2_weight_denom << 4 | sl->luma_log2_weight_denom :
		 VXD_H264_WEIGHTS_DEFAULT;
	apple_vxd_cmd_block(cb, VXD_H264_CMDS_WEIGHT_DENOM, 0, buf, 1);
	buf[0] = VXD_H264_CACHE_POLICY;
	apple_vxd_cmd_block(cb, VXD_H264_CMDS_CACHE, 0, buf, 1);
	buf[0] = f->dst_stride & ~63U;
	apple_vxd_cmd_block(cb, VXD_H264_CMDS_STRIDE, 0, buf, 1);
	buf[0] = chroma;
	apple_vxd_cmd_block(cb, VXD_H264_CMDS_CHROMA, 0, buf, 1);

	apple_vxd_cmd_end_slice(cb);
}

/*
 * One picture: per slice, the chunk and its kind-1 packet (payload as
 * 0x62c4c98 fills it, L39392-39429 of the dump).  Returns 0, or -EINVAL /
 * -ENOSPC with @why set; the buffer is then unusable for this picture.
 */
int apple_vxd_h264_build(struct apple_vxd_cmdbuf *cb,
			 const struct apple_vxd_h264_frame *f,
			 const char **why)
{
	u32 opmode = APPLE_VXD_H264_OPMODE_BASE | 2 << 20 | 1 << 12;
	unsigned int i;
	int ret;

	*why = NULL;
	ret = apple_vxd_h264_validate(f->sps, f->pps, f->scaling, why);
	if (ret)
		return ret;
	if (!f->nslices) {
		*why = "no slices";
		return -EINVAL;
	}

	for (i = 0; i < f->nslices; i++) {
		const struct apple_vxd_h264_slice *sl = &f->slices[i];
		int code = vxd_h264_slice_code(sl->slice_type);
		u32 off, bytes, *p;

		if (code < 0) {
			*why = "SP/SI slices are not supported";
			return -EINVAL;
		}
		if (vxd_h264_header_bits(f->pps, sl) >= 0x10000 ||
		    (vxd_h264_header_bits(f->pps, sl) & 0xffe0) >> 3 >= sl->nal_size) {
			*why = "slice header longer than the NAL";
			return -EINVAL;
		}

		vxd_h264_emit_slice(cb, f, i, code);
		ret = apple_vxd_cmdbuf_chunk_end_exact(cb, &off, &bytes);
		if (ret) {
			*why = "command buffer full or misused";
			return ret;
		}

		p = apple_vxd_cmdbuf_add_packet(cb, VXD_PKT_MAGIC0,
						VXD_PKT_H264_PAYLOAD);
		if (!p) {
			*why = "packet list full";
			return -ENOSPC;
		}
		p[VXD_PKT_FLAGS / 4] = (i ? 0 : 1) |
				       (!i && f->first_picture ? 0x10 : 0);
		p[VXD_PKT_W4 / 4] = opmode;
		p[VXD_PKT_CMD_BYTES / 4] = bytes;
		p[VXD_PKT_W2 / 4] = cb->va + off;
		p[VXD_PKT_W5 / 4] = f->bufs.stream_state;
		p[VXD_PKT_KIND / 4] = VXD_PKT_KIND_PICTURE;
		p[VXD_PKT_NOT_REF / 4] = !f->dp->nal_ref_idc;
	}
	return 0;
}

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple VXD - H.264 slice finder and slice-header parser (R3c)
 *
 * The device runs in V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED with
 * Annex-B start codes: the OUTPUT buffer holds a whole picture and the
 * driver itself locates the slices, as AppleD5500's host NAL walk does
 * (HOST_H264DecodeFrame, 0xfffffff0062965f4).  Every value the command
 * stream needs per slice comes from here; the picture-invariant values
 * come from the SPS/PPS/DECODE_PARAMS controls.
 *
 * Host-buildable: no allocation, no locking, no MMIO.
 */
#ifndef __APPLE_VXD_H264_PARSE_H__
#define __APPLE_VXD_H264_PARSE_H__

#include <linux/types.h>
#include <linux/v4l2-controls.h>

#define APPLE_VXD_H264_MAX_SLICES	128

#define APPLE_VXD_H264_NAL_SLICE	1
#define APPLE_VXD_H264_NAL_IDR		5

struct apple_vxd_h264_ref_mod {
	u8 idc;				/* modification_of_pic_nums_idc */
	u32 val;			/* abs_diff_pic_num_minus1 or long_term_pic_num */
};

/*
 * One pred_weight_table() entry with the values clause 7.4.3.2 infers for
 * absent flags (weight 1 << denominator, offset 0), the way AppleD5500's
 * copy (0xfffffff0062b0e9c) fills its per-reference arrays.
 */
struct apple_vxd_h264_weight {
	s16 weight[3];			/* Y, Cb, Cr */
	s8 offset[3];
};

struct apple_vxd_h264_slice {
	/* Location in the OUTPUT buffer. */
	u32 sc_offset;			/* first byte of the start code */
	u32 nal_offset;			/* the NAL header byte */
	u32 nal_size;			/* NAL header .. last non-zero byte */
	/*
	 * slice_header() length, counted from the first bit after the NAL
	 * header byte: in RBSP bits, and in bits of the escaped byte stream
	 * (emulation prevention bytes inside the header included).
	 */
	u32 header_bits;
	u32 header_bits_raw;

	u8 nal_ref_idc;
	u8 nal_unit_type;
	u32 first_mb_in_slice;
	u8 slice_type;			/* 0..4 (V4L2_H264_SLICE_TYPE_*) */
	u8 pps_id;
	u16 frame_num;
	u16 idr_pic_id;
	u16 pic_order_cnt_lsb;
	s32 delta_pic_order_cnt_bottom;
	s32 delta_pic_order_cnt[2];
	u8 redundant_pic_cnt;
	bool direct_spatial_mv_pred;
	bool num_ref_idx_override;
	u8 num_ref_idx_l0_active_minus1;
	u8 num_ref_idx_l1_active_minus1;
	u8 num_mods[2];
	struct apple_vxd_h264_ref_mod mods[2][V4L2_H264_REF_LIST_LEN + 1];
	bool has_pred_weight_table;
	u8 luma_log2_weight_denom;
	u8 chroma_log2_weight_denom;
	struct apple_vxd_h264_weight weights[2][V4L2_H264_REF_LIST_LEN];
	bool no_output_of_prior_pics;
	bool long_term_reference;
	bool adaptive_ref_pic_marking;
	u8 cabac_init_idc;
	s8 slice_qp_delta;
	bool sp_for_switch;
	s8 slice_qs_delta;
	u8 disable_deblocking_filter_idc;
	s8 slice_alpha_c0_offset_div2;
	s8 slice_beta_offset_div2;
};

int apple_vxd_h264_find_slices(const u8 *buf, size_t len,
			       const struct v4l2_ctrl_h264_sps *sps,
			       const struct v4l2_ctrl_h264_pps *pps,
			       struct apple_vxd_h264_slice *slices,
			       unsigned int max, unsigned int *count,
			       const char **why);

int apple_vxd_h264_modify_ref_list(const struct apple_vxd_h264_slice *sl,
				   unsigned int list,
				   const struct v4l2_ctrl_h264_sps *sps,
				   const struct v4l2_ctrl_h264_decode_params *dp,
				   struct v4l2_h264_reference *reflist,
				   unsigned int len);

#endif /* __APPLE_VXD_H264_PARSE_H__ */

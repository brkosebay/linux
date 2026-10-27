// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - H.264 slice finder and slice-header parser (R3c)
 *
 * Frame-based decode with Annex-B start codes: the OUTPUT buffer carries
 * the whole access unit and V4L2_CID_STATELESS_H264_SLICE_PARAMS is not
 * used (uapi: "When this mode is selected, ... SLICE_PARAMS is not
 * used").  The MSVDX command stream still needs per-slice values: where
 * the slice bytes are (the 0xa load), how many header bits precede the
 * slice data (the 0xb descriptor), the slice type, first MB, QP and
 * deblocking parameters, and the reference-list modifications.  Apple's
 * kext gets them from its own NAL walk; so does this file.
 *
 * Scope: progressive frames (frame_mbs_only), one slice group (no FMO),
 * no separate colour planes, no data partitioning.  Everything else of
 * clause 7.3.3 is parsed so the header length is exact.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "apple-vxd-h264-parse.h"

struct vxd_bits {
	const u8 *buf;			/* escaped bytes after the NAL header */
	size_t len;
	size_t byte;			/* raw index of the current byte */
	unsigned int bit;		/* bits of buf[byte] already read */
	unsigned int zeros;		/* zero bytes right before buf[byte] */
	u32 rbsp_bits;			/* bits read, emulation bytes excluded */
	bool overrun;
};

static unsigned int vxd_bit(struct vxd_bits *b)
{
	unsigned int v;

	if (!b->bit && b->zeros >= 2 && b->byte < b->len &&
	    b->buf[b->byte] == 0x03) {
		/* emulation_prevention_three_byte */
		b->byte++;
		b->zeros = 0;
	}
	if (b->byte >= b->len) {
		b->overrun = true;
		return 0;
	}

	v = (b->buf[b->byte] >> (7 - b->bit)) & 1;
	b->rbsp_bits++;
	if (++b->bit == 8) {
		b->zeros = b->buf[b->byte] ? 0 : b->zeros + 1;
		b->byte++;
		b->bit = 0;
	}
	return v;
}

static u32 vxd_bits_u(struct vxd_bits *b, unsigned int n)
{
	u32 v = 0;

	while (n--)
		v = (v << 1) | vxd_bit(b);
	return v;
}

static u32 vxd_bits_ue(struct vxd_bits *b)
{
	unsigned int zeros = 0;

	while (!vxd_bit(b)) {
		if (b->overrun || ++zeros > 31) {
			b->overrun = true;
			return 0;
		}
	}
	if (!zeros)
		return 0;
	return (u32)((1ULL << zeros) - 1) + vxd_bits_u(b, zeros);
}

static s32 vxd_bits_se(struct vxd_bits *b)
{
	u32 k = vxd_bits_ue(b);

	return (k & 1) ? (s32)((k + 1) / 2) : -(s32)(k / 2);
}

static bool vxd_slice_is(u8 slice_type, u8 want)
{
	return slice_type == want;
}

/* ChromaArrayType; profiles without chroma_format_idc in the SPS imply 4:2:0. */
static unsigned int vxd_chroma_array_type(const struct v4l2_ctrl_h264_sps *sps)
{
	if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		return 0;
	if (!V4L2_H264_SPS_HAS_CHROMA_FORMAT(sps))
		return 1;
	return sps->chroma_format_idc;
}

static int vxd_parse_mods(struct vxd_bits *b, struct apple_vxd_h264_slice *sl,
			  unsigned int list)
{
	unsigned int n = 0;
	u32 idc;

	if (!vxd_bit(b))		/* ref_pic_list_modification_flag_lX */
		return 0;

	do {
		idc = vxd_bits_ue(b);
		if (idc > 3 || n >= ARRAY_SIZE(sl->mods[list]) || b->overrun)
			return -EINVAL;
		sl->mods[list][n].idc = idc;
		sl->mods[list][n].val = idc < 3 ? vxd_bits_ue(b) : 0;
		if (idc == 3)
			break;
		n++;
	} while (1);
	sl->num_mods[list] = n;
	return 0;
}

/* luma/chroma_weight and _offset: se(v) in -128..127 (7.4.3.2, 8 bits). */
static bool vxd_weight_ok(s32 v)
{
	return v >= -128 && v <= 127;
}

/*
 * Clause 7.3.3.2, with the inferred weights and offsets filled in.  The
 * command stream carries the table for every active reference.
 */
static int vxd_parse_pred_weight_table(struct vxd_bits *b,
				       const struct v4l2_ctrl_h264_sps *sps,
				       struct apple_vxd_h264_slice *sl)
{
	unsigned int chroma = vxd_chroma_array_type(sps);
	unsigned int list, i, c, n;
	u32 v;

	v = vxd_bits_ue(b);
	if (v > 7)
		return -EINVAL;
	sl->luma_log2_weight_denom = v;
	if (chroma) {
		v = vxd_bits_ue(b);
		if (v > 7)
			return -EINVAL;
		sl->chroma_log2_weight_denom = v;
	}

	for (list = 0; list < 2; list++) {
		if (list && !vxd_slice_is(sl->slice_type, V4L2_H264_SLICE_TYPE_B))
			break;
		n = list ? sl->num_ref_idx_l1_active_minus1 :
			   sl->num_ref_idx_l0_active_minus1;
		for (i = 0; i <= n && !b->overrun; i++) {
			struct apple_vxd_h264_weight *w = &sl->weights[list][i];
			s32 wt, off;

			w->weight[0] = 1 << sl->luma_log2_weight_denom;
			if (vxd_bit(b)) {	/* luma_weight_lX_flag */
				wt = vxd_bits_se(b);
				off = vxd_bits_se(b);
				if (!vxd_weight_ok(wt) || !vxd_weight_ok(off))
					return -EINVAL;
				w->weight[0] = wt;
				w->offset[0] = off;
			}
			if (!chroma)
				continue;
			for (c = 1; c < 3; c++)
				w->weight[c] = 1 << sl->chroma_log2_weight_denom;
			if (!vxd_bit(b))	/* chroma_weight_lX_flag */
				continue;
			for (c = 1; c < 3; c++) {
				wt = vxd_bits_se(b);
				off = vxd_bits_se(b);
				if (!vxd_weight_ok(wt) || !vxd_weight_ok(off))
					return -EINVAL;
				w->weight[c] = wt;
				w->offset[c] = off;
			}
		}
	}
	return 0;
}

static int vxd_skip_dec_ref_pic_marking(struct vxd_bits *b,
					struct apple_vxd_h264_slice *sl)
{
	unsigned int n = 0;
	u32 mmco;

	if (sl->nal_unit_type == APPLE_VXD_H264_NAL_IDR) {
		sl->no_output_of_prior_pics = vxd_bit(b);
		sl->long_term_reference = vxd_bit(b);
		return 0;
	}

	sl->adaptive_ref_pic_marking = vxd_bit(b);
	if (!sl->adaptive_ref_pic_marking)
		return 0;

	do {
		mmco = vxd_bits_ue(b);
		if (mmco > 6 || ++n > 66 || b->overrun)
			return -EINVAL;
		if (mmco == 1 || mmco == 3)
			vxd_bits_ue(b);	/* difference_of_pic_nums_minus1 */
		if (mmco == 2)
			vxd_bits_ue(b);	/* long_term_pic_num */
		if (mmco == 3 || mmco == 6)
			vxd_bits_ue(b);	/* long_term_frame_idx */
		if (mmco == 4)
			vxd_bits_ue(b);	/* max_long_term_frame_idx_plus1 */
	} while (mmco);
	return 0;
}

/* Clause 7.3.3, for one NAL whose header byte is @nal[0]. */
static int vxd_parse_slice_header(const u8 *nal, size_t len,
				  const struct v4l2_ctrl_h264_sps *sps,
				  const struct v4l2_ctrl_h264_pps *pps,
				  struct apple_vxd_h264_slice *sl,
				  const char **why)
{
	bool p, b_slice, sp, si, i_slice;
	struct vxd_bits b = {
		.buf = nal + 1,
		.len = len - 1,
	};
	u32 v;

	sl->nal_ref_idc = (nal[0] >> 5) & 3;
	sl->nal_unit_type = nal[0] & 0x1f;

	sl->first_mb_in_slice = vxd_bits_ue(&b);
	v = vxd_bits_ue(&b);
	if (v > 9) {
		*why = "slice_type out of range";
		return -EINVAL;
	}
	sl->slice_type = v % 5;
	v = vxd_bits_ue(&b);
	if (v != pps->pic_parameter_set_id) {
		*why = "slice references another PPS";
		return -EINVAL;
	}
	sl->pps_id = v;

	p = vxd_slice_is(sl->slice_type, V4L2_H264_SLICE_TYPE_P);
	b_slice = vxd_slice_is(sl->slice_type, V4L2_H264_SLICE_TYPE_B);
	sp = vxd_slice_is(sl->slice_type, V4L2_H264_SLICE_TYPE_SP);
	si = vxd_slice_is(sl->slice_type, V4L2_H264_SLICE_TYPE_SI);
	i_slice = vxd_slice_is(sl->slice_type, V4L2_H264_SLICE_TYPE_I);

	sl->frame_num = vxd_bits_u(&b, sps->log2_max_frame_num_minus4 + 4);
	/* frame_mbs_only_flag is enforced by try_ctrl: no field_pic_flag. */
	if (sl->nal_unit_type == APPLE_VXD_H264_NAL_IDR)
		sl->idr_pic_id = vxd_bits_ue(&b);
	if (sps->pic_order_cnt_type == 0) {
		sl->pic_order_cnt_lsb =
			vxd_bits_u(&b, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT)
			sl->delta_pic_order_cnt_bottom = vxd_bits_se(&b);
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		sl->delta_pic_order_cnt[0] = vxd_bits_se(&b);
		if (pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT)
			sl->delta_pic_order_cnt[1] = vxd_bits_se(&b);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		sl->redundant_pic_cnt = vxd_bits_ue(&b);
	if (b_slice)
		sl->direct_spatial_mv_pred = vxd_bit(&b);

	sl->num_ref_idx_l0_active_minus1 = pps->num_ref_idx_l0_default_active_minus1;
	sl->num_ref_idx_l1_active_minus1 = pps->num_ref_idx_l1_default_active_minus1;
	if (p || sp || b_slice) {
		sl->num_ref_idx_override = vxd_bit(&b);
		if (sl->num_ref_idx_override) {
			v = vxd_bits_ue(&b);
			if (v > 31) {
				*why = "num_ref_idx_l0_active_minus1 > 31";
				return -EINVAL;
			}
			sl->num_ref_idx_l0_active_minus1 = v;
			if (b_slice) {
				v = vxd_bits_ue(&b);
				if (v > 31) {
					*why = "num_ref_idx_l1_active_minus1 > 31";
					return -EINVAL;
				}
				sl->num_ref_idx_l1_active_minus1 = v;
			}
		}
	}

	if (!i_slice && !si) {
		if (vxd_parse_mods(&b, sl, 0)) {
			*why = "bad ref_pic_list_modification (l0)";
			return -EINVAL;
		}
		if (b_slice && vxd_parse_mods(&b, sl, 1)) {
			*why = "bad ref_pic_list_modification (l1)";
			return -EINVAL;
		}
	}

	if (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) && (p || sp)) ||
	    (pps->weighted_bipred_idc == 1 && b_slice)) {
		sl->has_pred_weight_table = true;
		if (vxd_parse_pred_weight_table(&b, sps, sl)) {
			*why = "bad pred_weight_table";
			return -EINVAL;
		}
	}

	if (sl->nal_ref_idc && vxd_skip_dec_ref_pic_marking(&b, sl)) {
		*why = "bad dec_ref_pic_marking";
		return -EINVAL;
	}

	if ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) && !i_slice && !si)
		sl->cabac_init_idc = vxd_bits_ue(&b);
	sl->slice_qp_delta = vxd_bits_se(&b);
	if (sp || si) {
		if (sp)
			sl->sp_for_switch = vxd_bit(&b);
		sl->slice_qs_delta = vxd_bits_se(&b);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		sl->disable_deblocking_filter_idc = vxd_bits_ue(&b);
		if (sl->disable_deblocking_filter_idc != 1) {
			sl->slice_alpha_c0_offset_div2 = vxd_bits_se(&b);
			sl->slice_beta_offset_div2 = vxd_bits_se(&b);
		}
	}
	/* num_slice_groups_minus1 == 0 (try_ctrl): no slice_group_change_cycle. */

	if (b.overrun) {
		*why = "slice header runs past the NAL unit";
		return -EINVAL;
	}
	sl->header_bits = b.rbsp_bits;
	sl->header_bits_raw = b.byte * 8 + b.bit;
	return 0;
}

/* Next 0x000001 at or after @from, or @len. */
static size_t vxd_next_start_code(const u8 *buf, size_t len, size_t from)
{
	size_t i;

	for (i = from; i + 3 <= len; i++)
		if (!buf[i] && !buf[i + 1] && buf[i + 2] == 1)
			return i;
	return len;
}

/*
 * Walk the Annex-B access unit and parse every slice NAL (types 1 and 5).
 * Parameter sets, SEI, AUD and the like may be present and are skipped;
 * redundant slices (redundant_pic_cnt > 0) are dropped as decoders may.
 * The slices of one picture must agree on the picture-level syntax and
 * come in increasing first_mb_in_slice order.
 */
int apple_vxd_h264_find_slices(const u8 *buf, size_t len,
			       const struct v4l2_ctrl_h264_sps *sps,
			       const struct v4l2_ctrl_h264_pps *pps,
			       struct apple_vxd_h264_slice *slices,
			       unsigned int max, unsigned int *count,
			       const char **why)
{
	size_t sc = vxd_next_start_code(buf, len, 0);
	unsigned int n = 0;
	int ret;

	*count = 0;
	*why = NULL;
	while (sc < len) {
		size_t nal = sc + 3, next, end;
		struct apple_vxd_h264_slice *sl;
		u8 type;

		next = vxd_next_start_code(buf, len, nal);
		end = next;
		while (end > nal && !buf[end - 1])	/* trailing_zero_8bits */
			end--;
		if (sc && !buf[sc - 1])			/* 4-byte start code */
			sc--;
		if (end <= nal)
			goto next_nal;

		if (buf[nal] & 0x80) {
			*why = "forbidden_zero_bit set";
			return -EINVAL;
		}
		type = buf[nal] & 0x1f;
		if (type >= 2 && type <= 4) {
			*why = "data partitioning is not supported";
			return -EINVAL;
		}
		if (type != APPLE_VXD_H264_NAL_SLICE &&
		    type != APPLE_VXD_H264_NAL_IDR)
			goto next_nal;

		if (n == max) {
			*why = "too many slices in one picture";
			return -E2BIG;
		}
		sl = &slices[n];
		memset(sl, 0, sizeof(*sl));
		sl->sc_offset = sc;
		sl->nal_offset = nal;
		sl->nal_size = end - nal;
		ret = vxd_parse_slice_header(buf + nal, end - nal, sps, pps, sl, why);
		if (ret)
			return ret;
		if (sl->redundant_pic_cnt)
			goto next_nal;

		if (!n && sl->first_mb_in_slice) {
			*why = "the first slice does not start at macroblock 0";
			return -EINVAL;
		}
		if (n && (sl->first_mb_in_slice <= slices[n - 1].first_mb_in_slice ||
			  sl->frame_num != slices[0].frame_num ||
			  sl->nal_unit_type != slices[0].nal_unit_type ||
			  sl->idr_pic_id != slices[0].idr_pic_id ||
			  sl->pic_order_cnt_lsb != slices[0].pic_order_cnt_lsb)) {
			*why = "slices disagree on the picture or are out of order (ASO)";
			return -EINVAL;
		}
		n++;
next_nal:
		sc = next;
	}

	if (!n) {
		*why = "no slice NAL unit in the buffer";
		return -ENOENT;
	}
	*count = n;
	return 0;
}

/* FrameNumWrap of a short-term DPB entry (clause 8.2.4.1, frames). */
static s32 vxd_pic_num(const struct v4l2_h264_dpb_entry *e, u32 cur_frame_num,
		       u32 max_frame_num)
{
	return e->frame_num > cur_frame_num ? (s32)e->frame_num - (s32)max_frame_num :
					      (s32)e->frame_num;
}

static bool vxd_dpb_is_ref(const struct v4l2_h264_dpb_entry *e)
{
	return (e->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) &&
	       (e->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID);
}

/*
 * Clause 8.2.4.3, frames only: apply the slice's modifications to the
 * initial list (indices into dp->dpb, as built by the v4l2-h264 helper).
 * @len is num_ref_idx_lX_active_minus1 + 1; @reflist holds at least
 * V4L2_H264_REF_LIST_LEN entries.
 */
int apple_vxd_h264_modify_ref_list(const struct apple_vxd_h264_slice *sl,
				   unsigned int list,
				   const struct v4l2_ctrl_h264_sps *sps,
				   const struct v4l2_ctrl_h264_decode_params *dp,
				   struct v4l2_h264_reference *reflist,
				   unsigned int len)
{
	struct v4l2_h264_reference tmp[V4L2_H264_REF_LIST_LEN + 1];
	u32 max_frame_num = 1U << (sps->log2_max_frame_num_minus4 + 4);
	s32 cur = sl->frame_num, pred = cur;
	unsigned int m, ref_idx = 0;

	if (!sl->num_mods[list])
		return 0;
	if (!len || len > V4L2_H264_REF_LIST_LEN)
		return -EINVAL;

	memcpy(tmp, reflist, len * sizeof(*tmp));
	for (m = 0; m < sl->num_mods[list]; m++) {
		const struct apple_vxd_h264_ref_mod *mod = &sl->mods[list][m];
		unsigned int i, c, nidx;
		bool long_term = mod->idc == 2;
		s32 pic_num = 0;
		int found = -1;

		if (!long_term) {
			s32 no_wrap, diff = mod->val + 1;

			if (diff > (s32)max_frame_num)
				return -EINVAL;
			if (mod->idc == 0) {
				no_wrap = pred - diff;
				if (no_wrap < 0)
					no_wrap += max_frame_num;
			} else {
				no_wrap = pred + diff;
				if (no_wrap >= (s32)max_frame_num)
					no_wrap -= max_frame_num;
			}
			pred = no_wrap;
			pic_num = no_wrap > cur ? no_wrap - (s32)max_frame_num : no_wrap;
		}

		for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
			const struct v4l2_h264_dpb_entry *e = &dp->dpb[i];
			bool lt = e->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;

			if (!vxd_dpb_is_ref(e) || lt != long_term)
				continue;
			if (long_term ? e->pic_num == mod->val :
					vxd_pic_num(e, cur, max_frame_num) == pic_num) {
				found = i;
				break;
			}
		}
		if (found < 0)
			return -ENOENT;

		for (c = len; c > ref_idx; c--)
			tmp[c] = tmp[c - 1];
		tmp[ref_idx].index = found;
		tmp[ref_idx].fields = V4L2_H264_FRAME_REF;
		ref_idx++;
		nidx = ref_idx;
		for (c = ref_idx; c <= len; c++)
			if (tmp[c].index != found)
				tmp[nidx++] = tmp[c];
	}
	memcpy(reflist, tmp, len * sizeof(*tmp));
	return 0;
}

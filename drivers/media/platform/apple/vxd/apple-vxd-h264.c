// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - H.264 backend (V4L2 stateless, frame based)
 *
 * Userspace parses the parameter sets and the DPB (V4L2 stateless
 * contract); the OUTPUT buffer carries a whole Annex-B access unit.  Per
 * picture this file
 *
 *   1. validates SPS/PPS against what the AVC recipe covers,
 *   2. finds and parses the slices itself (frame-based mode has no
 *      SLICE_PARAMS, apple-vxd-h264-parse.c),
 *   3. builds RefPicList0/1 (v4l2-h264 helper + the slice's
 *      modifications) and resolves the DPB to CAPTURE device addresses
 *      (and, for B slices, stored-MB buffers) by timestamp,
 *   4. builds the command stream (apple-vxd-h264-cmd.c) into a pooled
 *      command buffer and submits one 0x81 message per slice.
 *
 * The job completes from the last packet's 0xc0 (apple_vxd_job_done()).
 *
 * The context owns the buffers DecodeCreateH264VLD allocates for the
 * AVC path (ds-vxd-cmdstream.md section 16.5): the packed CAVLC table
 * (0x1000, the 0x440-byte constant), the IntraBuffer and the
 * StreamStateBuffer, each sized for the larger of Apple's two platform
 * multipliers.  Each CAPTURE buffer carries its stored-macroblock
 * parameter buffer (apple_vxd_buffer.aux).
 */
#include <linux/crc32.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/sizes.h>

#include <media/v4l2-h264.h>

#include "apple-vxd.h"
#include "apple-vxd-h264-cmd.h"
#include "apple-vxd-h264-parse.h"
#include "apple-vxd-h264-tables.h"

static unsigned int apple_vxd_scaling_order;
module_param_named(scaling_order, apple_vxd_scaling_order, uint, 0644);
MODULE_PARM_DESC(scaling_order,
		 "debug: scaling-list element order, 0 raster (AppleD5500, default), 1 zig-zag, 2 transposed");

/* DecodeCreateH264VLD sizes with Apple's multiplier M = 2. */
#define APPLE_VXD_H264_CAVLC_BUF	SZ_4K
#define APPLE_VXD_H264_INTRA_BUF	(2 << 16)
#define APPLE_VXD_H264_STREAM_STATE_BUF	(2 << 12)

struct apple_vxd_h264_ctx {
	struct apple_vxd_dev_buf cavlc;
	struct apple_vxd_dev_buf intra;
	struct apple_vxd_dev_buf stream_state;
	bool first_picture;
	struct apple_vxd_h264_slice slices[APPLE_VXD_H264_MAX_SLICES];
	u8 ref_slots[APPLE_VXD_H264_MAX_SLICES][2][V4L2_H264_REF_LIST_LEN];
};

static_assert(sizeof(apple_vxd_h264_vlc_table) == APPLE_VXD_H264_VLC_BYTES);

static int apple_vxd_h264_validate_sps(struct apple_vxd_ctx *ctx,
				       const struct v4l2_ctrl_h264_sps *sps)
{
	const struct apple_vxd_dev *vxd = ctx->vxd;

	/*
	 * Reject the profiles whose tools the 8-bit 4:2:0 decoder lacks;
	 * High 10/4:2:2/4:4:4 are caught by the chroma/bit-depth checks below.
	 * No allow-list: the control's default (profile_idc 0, before
	 * userspace sets an SPS) must pass, or TRY_EXT_CTRLS on the current
	 * values fails (v4l2-compliance, round 3 QEMU smoke).  What the AVC
	 * command recipe does not cover (interlace, FMO) is refused per
	 * picture.
	 */
	switch (sps->profile_idc) {
	case 88: /* extended: data partitioning */
	case 83: /* scalable baseline */
	case 86: /* scalable high */
	case 118: /* multiview high */
	case 128: /* stereo high */
	case 134: /* MFC high */
	case 135: /* MFC depth high */
	case 138: /* multiview depth high */
	case 139: /* enhanced multiview depth high */
		dev_dbg(vxd->dev, "sps: unsupported profile %u\n",
			sps->profile_idc);
		return -EINVAL;
	default:
		break;
	}

	if (sps->chroma_format_idc > 1) {
		dev_dbg(vxd->dev, "sps: chroma format 4:2:2/4:4:4 not supported\n");
		return -EINVAL;
	}

	if (sps->bit_depth_luma_minus8 > 0 || sps->bit_depth_chroma_minus8 > 0) {
		dev_dbg(vxd->dev, "sps: 10-bit not supported\n");
		return -EINVAL;
	}

	/*
	 * DT advertises h264-playback-level = 42: 1920x1088 [E].  Tile
	 * decode could lift this, but is not explored yet.
	 */
	if (sps->pic_width_in_mbs_minus1 > (APPLE_VXD_MAX_PIC_W / 16 - 1) ||
	    sps->pic_height_in_map_units_minus1 > (APPLE_VXD_MAX_PIC_H / 16 - 1)) {
		dev_dbg(vxd->dev, "sps: %ux%u exceeds level 4.2\n",
			(sps->pic_width_in_mbs_minus1 + 1) * 16,
			(sps->pic_height_in_map_units_minus1 + 1) * 16);
		return -EINVAL;
	}

	if (sps->max_num_ref_frames > V4L2_H264_NUM_DPB_ENTRIES) {
		dev_dbg(vxd->dev, "sps: max_num_ref_frames %u too many\n",
			sps->max_num_ref_frames);
		return -EINVAL;
	}

	return 0;
}

static int apple_vxd_h264_try_ctrl(struct apple_vxd_ctx *ctx,
				   struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_H264_SPS)
		return apple_vxd_h264_validate_sps(ctx, ctrl->p_new.p_h264_sps);

	return 0;
}

static int apple_vxd_h264_start(struct apple_vxd_ctx *ctx)
{
	struct apple_vxd_dev *vxd = ctx->vxd;
	struct apple_vxd_h264_ctx *h;
	int ret;

	h = kvzalloc_obj(*h);
	if (!h)
		return -ENOMEM;

	ret = apple_vxd_dev_buf_alloc(vxd, APPLE_VXD_H264_CAVLC_BUF, &h->cavlc);
	if (ret)
		goto err_free;
	ret = apple_vxd_dev_buf_alloc(vxd, APPLE_VXD_H264_INTRA_BUF, &h->intra);
	if (ret)
		goto err_cavlc;
	ret = apple_vxd_dev_buf_alloc(vxd, APPLE_VXD_H264_STREAM_STATE_BUF,
				      &h->stream_state);
	if (ret)
		goto err_intra;

	memcpy(h->cavlc.mem.vaddr, apple_vxd_h264_vlc_table,
	       sizeof(apple_vxd_h264_vlc_table));
	if ((crc32_le(~0U, h->cavlc.mem.vaddr, APPLE_VXD_H264_VLC_BYTES) ^ ~0U) !=
	    APPLE_VXD_H264_VLC_CRC32) {
		dev_err(vxd->dev, "h264: CAVLC table CRC mismatch\n");
		ret = -EIO;
		goto err_state;
	}
	h->first_picture = true;
	ctx->priv = h;

	vxd_step_io(vxd->dev,
		    "h264: context buffers: cavlc %#x, intra %#x (%u bytes), stream state %#x (%u bytes)",
		    h->cavlc.va, h->intra.va, APPLE_VXD_H264_INTRA_BUF,
		    h->stream_state.va, APPLE_VXD_H264_STREAM_STATE_BUF);
	return 0;

err_state:
	apple_vxd_dev_buf_free(vxd, &h->stream_state);
err_intra:
	apple_vxd_dev_buf_free(vxd, &h->intra);
err_cavlc:
	apple_vxd_dev_buf_free(vxd, &h->cavlc);
err_free:
	kvfree(h);
	return ret;
}

static void apple_vxd_h264_stop(struct apple_vxd_ctx *ctx)
{
	struct apple_vxd_h264_ctx *h = ctx->priv;

	if (!h)
		return;
	apple_vxd_dev_buf_free(ctx->vxd, &h->stream_state);
	apple_vxd_dev_buf_free(ctx->vxd, &h->intra);
	apple_vxd_dev_buf_free(ctx->vxd, &h->cavlc);
	kvfree(h);
	ctx->priv = NULL;
}

static enum apple_vxd_image_fmt
apple_vxd_h264_get_image_fmt(struct apple_vxd_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id != V4L2_CID_STATELESS_H264_SPS)
		return APPLE_VXD_IMG_FMT_ANY;

	return APPLE_VXD_IMG_FMT_420_8BIT;
}

static size_t apple_vxd_h264_capture_aux_size(struct apple_vxd_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->decoded_fmt.fmt.pix_mp;

	return apple_vxd_h264_colocated_size(DIV_ROUND_UP(pix->width, 16),
					     DIV_ROUND_UP(pix->height, 16));
}

static const void *apple_vxd_h264_ctrl(struct apple_vxd_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	return ctrl ? ctrl->p_cur.p : NULL;
}

/*
 * RefPicList0/1 per slice as DPB slots: the initial lists (8.2.4.2.1 for
 * P, 8.2.4.2.3 for B, built once per picture by the v4l2-h264 helper),
 * truncated to num_ref_idx_lX_active, then the slice's modifications
 * (8.2.4.3).  Entries past the available references are NO_REF.
 */
enum { VXD_LIST_P, VXD_LIST_B0, VXD_LIST_B1, VXD_LISTS };

static int apple_vxd_h264_ref_lists(struct apple_vxd_h264_ctx *h,
				    unsigned int nslices,
				    const struct v4l2_ctrl_h264_sps *sps,
				    const struct v4l2_ctrl_h264_decode_params *dp)
{
	struct v4l2_h264_reference init[VXD_LISTS][V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference list[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reflist_builder b;
	bool have_p = false, have_b = false;
	unsigned int i, k, lst;
	int ret;

	v4l2_h264_init_reflist_builder(&b, dp, sps, dp->dpb);
	for (i = 0; i < nslices; i++) {
		const struct apple_vxd_h264_slice *sl = &h->slices[i];
		bool bslice = sl->slice_type == V4L2_H264_SLICE_TYPE_B;

		memset(h->ref_slots[i], APPLE_VXD_H264_NO_REF,
		       sizeof(h->ref_slots[i]));
		if (sl->slice_type != V4L2_H264_SLICE_TYPE_P && !bslice)
			continue;

		if (!bslice && !have_p) {
			v4l2_h264_build_p_ref_list(&b, init[VXD_LIST_P]);
			have_p = true;
		}
		if (bslice && !have_b) {
			v4l2_h264_build_b_ref_lists(&b, init[VXD_LIST_B0],
						    init[VXD_LIST_B1]);
			have_b = true;
		}
		for (lst = 0; lst < (bslice ? 2 : 1); lst++) {
			const struct v4l2_h264_reference *src =
				init[bslice ? VXD_LIST_B0 + lst : VXD_LIST_P];
			unsigned int len = 1 + (lst ? sl->num_ref_idx_l1_active_minus1 :
						      sl->num_ref_idx_l0_active_minus1);

			for (k = 0; k < V4L2_H264_REF_LIST_LEN; k++) {
				list[k] = src[k];
				if (k >= b.num_valid)
					list[k].index = APPLE_VXD_H264_NO_REF;
			}
			ret = apple_vxd_h264_modify_ref_list(sl, lst, sps, dp,
							     list, len);
			if (ret)
				return ret;
			for (k = 0; k < len; k++)
				h->ref_slots[i][lst][k] = list[k].index;
		}
	}
	return 0;
}

/* Every reference a slice names must resolve to a CAPTURE buffer. */
static const char *apple_vxd_h264_check_refs(const struct apple_vxd_h264_ctx *h,
					     unsigned int nslices,
					     const struct apple_vxd_h264_frame *f)
{
	unsigned int i, k, lst;

	for (i = 0; i < nslices; i++) {
		const struct apple_vxd_h264_slice *sl = &h->slices[i];
		bool bslice = sl->slice_type == V4L2_H264_SLICE_TYPE_B;

		if (sl->slice_type != V4L2_H264_SLICE_TYPE_P && !bslice)
			continue;
		for (lst = 0; lst < (bslice ? 2 : 1); lst++) {
			unsigned int len = 1 + (lst ? sl->num_ref_idx_l1_active_minus1 :
						      sl->num_ref_idx_l0_active_minus1);

			for (k = 0; k < len; k++) {
				u8 slot = h->ref_slots[i][lst][k];

				if (slot < V4L2_H264_NUM_DPB_ENTRIES &&
				    !f->dpb_luma[slot])
					return "a reference picture has no CAPTURE buffer";
			}
		}
		if (bslice) {
			u8 col = h->ref_slots[i][1][0];

			if (col >= V4L2_H264_NUM_DPB_ENTRIES ||
			    !f->dpb_colocated[col])
				return "co-located picture (RefPicList1[0]) unresolved";
		}
	}
	return NULL;
}

static int apple_vxd_h264_run(struct apple_vxd_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->decoded_fmt.fmt.pix_mp;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling;
	const struct v4l2_ctrl_h264_decode_params *dp;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	struct apple_vxd_h264_ctx *h = ctx->priv;
	struct apple_vxd_dev *vxd = ctx->vxd;
	struct apple_vxd_h264_frame f = { };
	struct apple_vxd_cmdbuf *cb = NULL;
	struct apple_vxd_run run;
	const char *why = NULL;
	struct vb2_queue *cap_q;
	unsigned int i, nslices;
	const u8 *src;
	size_t src_len;
	u32 stride;
	int ret;

	apple_vxd_run_preamble(ctx, &run);

	if (!vxd->hw_ready || vxd->fw_panicked) {
		dev_dbg(vxd->dev,
			"decode job with no usable firmware (not booted, init message not taken, or panicked)\n");
		ret = -EAGAIN;
		goto out;
	}
	if (!h) {
		ret = -EINVAL;
		goto out;
	}

	sps = apple_vxd_h264_ctrl(ctx, V4L2_CID_STATELESS_H264_SPS);
	pps = apple_vxd_h264_ctrl(ctx, V4L2_CID_STATELESS_H264_PPS);
	dp = apple_vxd_h264_ctrl(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	scaling = apple_vxd_h264_ctrl(ctx, V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	if (!sps || !pps || !dp) {
		ret = -EINVAL;
		goto out;
	}
	ret = apple_vxd_h264_validate(sps, pps, scaling, &why);
	if (ret)
		goto out_why;
	if (dp->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC) {
		why = "field pictures";
		ret = -EINVAL;
		goto out_why;
	}
	if ((sps->pic_width_in_mbs_minus1 + 1) * 16 > pix->width ||
	    (sps->pic_height_in_map_units_minus1 + 1) * 16 > pix->height) {
		why = "picture larger than the CAPTURE format";
		ret = -EINVAL;
		goto out_why;
	}

	src_len = vb2_get_plane_payload(&run.src->vb2_buf, 0);
	src = vb2_plane_vaddr(&run.src->vb2_buf, 0);
	if (!src) {
		why = "OUTPUT buffer has no kernel mapping";
		ret = -EINVAL;
		goto out_why;
	}
	ret = apple_vxd_h264_find_slices(src, src_len, sps, pps, h->slices,
					 APPLE_VXD_H264_MAX_SLICES, &nslices,
					 &why);
	if (ret)
		goto out_why;
	if (nslices > APPLE_VXD_JOB_MAX_PACKETS) {
		why = "more slices in one picture than packets in a job";
		ret = -EINVAL;
		goto out_why;
	}
	ret = apple_vxd_h264_ref_lists(h, nslices, sps, dp);
	if (ret) {
		why = "a reference-list modification names a picture not in the DPB";
		goto out_why;
	}

	stride = pix->plane_fmt[0].bytesperline;
	f.sps = sps;
	f.pps = pps;
	f.dp = dp;
	f.scaling = scaling;
	f.slices = h->slices;
	f.nslices = nslices;
	f.ref_slots = (const u8 (*)[2][V4L2_H264_REF_LIST_LEN])h->ref_slots;
	f.src_va = apple_vxd_buf_va(run.src);
	f.dst_luma = apple_vxd_buf_va(run.dst);
	f.dst_chroma = f.dst_luma + stride * pix->height;
	f.dst_stride = stride;
	f.colocated = apple_vxd_buf_aux_va(run.dst);
	f.bufs.cavlc = h->cavlc.va;
	f.bufs.intra = h->intra.va;
	f.bufs.stream_state = h->stream_state.va;
	f.first_picture = h->first_picture;
	f.scaling_order = min_t(unsigned int, apple_vxd_scaling_order,
				APPLE_VXD_H264_SCALING_TRANSPOSED);

	cap_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		const struct v4l2_h264_dpb_entry *e = &dp->dpb[i];
		struct vb2_buffer *ref;

		if (!(e->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		ref = vb2_find_buffer(cap_q, e->reference_ts);
		if (!ref) {
			dev_dbg(vxd->dev, "h264: dpb[%u] ts %llu has no CAPTURE buffer\n",
				i, e->reference_ts);
			continue;
		}
		f.dpb_luma[i] = apple_vxd_buf_va(to_vb2_v4l2_buffer(ref));
		f.dpb_chroma[i] = f.dpb_luma[i] + stride * pix->height;
		f.dpb_colocated[i] = apple_vxd_buf_aux_va(to_vb2_v4l2_buffer(ref));
	}
	why = apple_vxd_h264_check_refs(h, nslices, &f);
	if (why) {
		ret = -EINVAL;
		goto out_why;
	}

	cb = apple_vxd_cmdbuf_get(vxd);
	if (!cb) {
		why = "no free command buffer";
		ret = -EBUSY;
		goto out_why;
	}
	ret = apple_vxd_h264_build(cb, &f, &why);
	if (ret)
		goto out_why;

	vxd_step_io(vxd->dev,
		    "decode: src %u va %#x (%zu bytes, %u slices, first %c NAL %#x+%u, header %u bits), dst %u Y %#x UV %#x stride %u, %zu command bytes in buf %u (va %#x), %u packets",
		    run.src->vb2_buf.index, f.src_va, src_len, nslices,
		    "PBIps"[h->slices[0].slice_type % 5],
		    h->slices[0].nal_offset, h->slices[0].nal_size,
		    8 + h->slices[0].header_bits_raw, run.dst->vb2_buf.index,
		    f.dst_luma, f.dst_chroma, stride, cb->used, cb->index,
		    cb->va, cb->npackets);

	/*
	 * The controls were only read while building: complete them before
	 * the submit, after which the job may finish (and the request with
	 * it) at any moment.
	 */
	apple_vxd_run_postamble(ctx, &run);
	ret = apple_vxd_submit(ctx, cb);
	if (ret) {
		dev_warn_ratelimited(vxd->dev, "decode: submit failed: %d\n", ret);
		if (cb->state == APPLE_VXD_CMDBUF_HOST)
			apple_vxd_cmdbuf_put(vxd, cb);
		return ret;
	}
	h->first_picture = false;
	return 0;

out_why:
	dev_warn_ratelimited(vxd->dev, "decode: picture refused (%d): %s\n",
			     ret, why ? why : "?");
out:
	if (cb)
		apple_vxd_cmdbuf_put(vxd, cb);
	apple_vxd_run_postamble(ctx, &run);
	return ret;
}

static const struct apple_vxd_ctrl_desc apple_vxd_h264_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SPS,
		.cfg.ops = &apple_vxd_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_PPS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SLICE_PARAMS,
		.cfg.flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.cfg.max = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.cfg.def = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_START_CODE,
		.cfg.min = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.cfg.def = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.cfg.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
		.cfg.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA,
		.cfg.menu_skip_mask =
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED) |
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE),
		.cfg.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.cfg.max = V4L2_MPEG_VIDEO_H264_LEVEL_4_2,
		.cfg.def = V4L2_MPEG_VIDEO_H264_LEVEL_4_2,
	},
};

static const struct apple_vxd_ctrls apple_vxd_h264_ctrls = {
	.ctrls = apple_vxd_h264_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(apple_vxd_h264_ctrl_descs),
};

static const struct apple_vxd_decoded_fmt_desc
apple_vxd_h264_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = APPLE_VXD_IMG_FMT_420_8BIT,
	},
};

static const struct apple_vxd_coded_fmt_desc apple_vxd_h264_fmt = {
	.fourcc = V4L2_PIX_FMT_H264_SLICE,
	.frmsize = {
		.min_width = APPLE_VXD_MIN_PIC_W,
		.max_width = APPLE_VXD_MAX_PIC_W,
		.step_width = 16,
		.min_height = APPLE_VXD_MIN_PIC_H,
		.max_height = APPLE_VXD_MAX_PIC_H,
		.step_height = 16,
	},
	.ctrls = &apple_vxd_h264_ctrls,
	.ops = &(const struct apple_vxd_coded_fmt_ops){
		.adjust_fmt = NULL,
		.start = apple_vxd_h264_start,
		.stop = apple_vxd_h264_stop,
		.run = apple_vxd_h264_run,
		.try_ctrl = apple_vxd_h264_try_ctrl,
		.get_image_fmt = apple_vxd_h264_get_image_fmt,
		.capture_aux_size = apple_vxd_h264_capture_aux_size,
	},
	.num_decoded_fmts = ARRAY_SIZE(apple_vxd_h264_decoded_fmts),
	.decoded_fmts = apple_vxd_h264_decoded_fmts,
};

static const struct apple_vxd_coded_fmt_desc apple_vxd_coded_fmts[] = {
	apple_vxd_h264_fmt,
};

/*
 * Fills the device's coded-format table.  Called during probe, before any
 * V4L2 node exists: apple_vxd_v4l2_init() and open() both refuse to run
 * without it (round 2 had no such fill and oopsed in the first open()).
 */
int apple_vxd_h264_register(struct apple_vxd_dev *vxd)
{
	vxd->coded_fmts = apple_vxd_coded_fmts;
	vxd->num_coded_fmts = ARRAY_SIZE(apple_vxd_coded_fmts);
	return 0;
}

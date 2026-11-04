// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - V4L2 stateless m2m device (R3/R4 skeleton)
 *
 * Structure follows the rkvdec stateless template: one /dev/video node,
 * media controller entity, mplane m2m queues with the request API
 * mandatory on the output queue, and the stateless H.264 control set.
 * The hardware submission layer behind the H.264 backend refuses jobs
 * until the command-stream generator (R3c) lands; that failure path is
 * part of the bring-up contract (test plan T6).
 *
 * Round 3: every handler body takes the context, not the file, so the
 * register-time self-test runs the same code as open(), the format ioctls
 * and release() on a private context before the node is exposed.  Every
 * V4L2 entry point announces itself with a flushed STEP line; nothing here
 * touches state that only the firmware boot creates (the only consumer of
 * hw_ready is the H.264 job path, which fails the job instead).
 */
#include <linux/pm_runtime.h>
#include <linux/sched.h>

#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "apple-vxd.h"

/* Smallest bitstream buffer handed to userspace. */
#define APPLE_VXD_MIN_BITSTREAM_SIZE	SZ_1M

/*
 * The VXD is not DMA-coherent, so coherent dma-contig buffers are mapped
 * uncached for the CPU.  Every consumer that reads decoded pictures on the
 * CPU (FFmpeg's hwframe download, GStreamer's videoconvert, Chromium's
 * NV12->ARGB converter) then reads uncached memory.  capture_cached=1
 * makes the CAPTURE queues of contexts opened afterwards allocate MMAP
 * buffers non-coherent (cacheable): vb2 cleans them at QBUF and
 * invalidates them at DQBUF, and userspace only reads a picture after
 * DQBUF.  They stay physically contiguous, so the GART path is unchanged.
 */
static bool apple_vxd_capture_cached;
module_param_named(capture_cached, apple_vxd_capture_cached, bool, 0644);
MODULE_PARM_DESC(capture_cached,
		 "cacheable MMAP CAPTURE buffers for contexts opened afterwards (vb2 syncs at QBUF/DQBUF)");

static struct apple_vxd_ctx *file_to_vxd_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct apple_vxd_ctx, fh);
}

static void apple_vxd_reset_fmt(struct apple_vxd_ctx *ctx,
				struct v4l2_format *f, u32 fourcc)
{
	memset(f, 0, sizeof(*f));
	f->fmt.pix_mp.pixelformat = fourcc;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

/*
 * Every MSVDX surface base register (luma and chroma reconstruction,
 * references) drops the low 12 bits, and the extended row stride counts
 * 64-byte units.  With single-plane NV12 the chroma plane starts at
 * bytesperline * height, so the luma stride is rounded up to 256 bytes:
 * with the height a multiple of 16 that keeps the chroma plane 4 KiB
 * aligned behind a page-aligned buffer.
 */
#define APPLE_VXD_NV12_STRIDE_ALIGN	256

static void apple_vxd_fill_decoded_pixfmt(struct apple_vxd_ctx *ctx,
					  struct v4l2_pix_format_mplane *pix_mp)
{
	struct v4l2_plane_pix_format *plane = &pix_mp->plane_fmt[0];

	v4l2_fill_pixfmt_mp(pix_mp, pix_mp->pixelformat,
			    pix_mp->width, pix_mp->height);
	if (pix_mp->pixelformat == V4L2_PIX_FMT_NV12) {
		plane->bytesperline = ALIGN(pix_mp->width,
					    APPLE_VXD_NV12_STRIDE_ALIGN);
		plane->sizeimage = plane->bytesperline * pix_mp->height * 3 / 2;
	}
}

static const struct apple_vxd_decoded_fmt_desc *
apple_vxd_find_decoded_fmt(const struct apple_vxd_coded_fmt_desc *desc,
			   enum apple_vxd_image_fmt image_fmt)
{
	unsigned int i;

	for (i = 0; i < desc->num_decoded_fmts; i++)
		if (desc->decoded_fmts[i].image_fmt == image_fmt)
			return &desc->decoded_fmts[i];

	return NULL;
}

static u32 apple_vxd_enum_decoded_fmt(struct apple_vxd_ctx *ctx, int index,
				      enum apple_vxd_image_fmt image_fmt)
{
	const struct apple_vxd_decoded_fmt_desc *dfmt;

	if (image_fmt == APPLE_VXD_IMG_FMT_ANY)
		image_fmt = APPLE_VXD_IMG_FMT_420_8BIT;
	dfmt = apple_vxd_find_decoded_fmt(ctx->coded_fmt_desc, image_fmt);
	if (dfmt && !index)
		return dfmt->fourcc;
	return 0;
}

static bool apple_vxd_is_valid_fmt(struct apple_vxd_ctx *ctx, u32 fourcc,
				   enum apple_vxd_image_fmt image_fmt)
{
	return apple_vxd_enum_decoded_fmt(ctx, 0, image_fmt) == fourcc;
}

static const struct apple_vxd_coded_fmt_desc *
apple_vxd_find_coded_fmt_desc(struct apple_vxd_ctx *ctx, u32 fourcc)
{
	struct apple_vxd_dev *vxd = ctx->vxd;
	unsigned int i;

	for (i = 0; i < vxd->num_coded_fmts; i++)
		if (vxd->coded_fmts[i].fourcc == fourcc)
			return &vxd->coded_fmts[i];
	return NULL;
}

static void apple_vxd_reset_coded_fmt(struct apple_vxd_ctx *ctx)
{
	struct v4l2_format *f = &ctx->coded_fmt;

	/* apple_vxd_ctx_init() checked vxd->coded_fmts before calling us. */
	ctx->coded_fmt_desc = &ctx->vxd->coded_fmts[0];
	apple_vxd_reset_fmt(ctx, f, ctx->coded_fmt_desc->fourcc);
	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f->fmt.pix_mp.width = ctx->coded_fmt_desc->frmsize.min_width;
	f->fmt.pix_mp.height = ctx->coded_fmt_desc->frmsize.min_height;
	/* Same normalization as TRY_FMT: one plane, a usable bitstream size. */
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].sizeimage = APPLE_VXD_MIN_BITSTREAM_SIZE;

	if (ctx->coded_fmt_desc->ops->adjust_fmt)
		ctx->coded_fmt_desc->ops->adjust_fmt(ctx, f);
}

static void apple_vxd_reset_decoded_fmt(struct apple_vxd_ctx *ctx)
{
	struct v4l2_format *f = &ctx->decoded_fmt;
	u32 fourcc = apple_vxd_enum_decoded_fmt(ctx, 0, ctx->image_fmt);

	apple_vxd_reset_fmt(ctx, f, fourcc);
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->fmt.pix_mp.width = ctx->coded_fmt.fmt.pix_mp.width;
	f->fmt.pix_mp.height = ctx->coded_fmt.fmt.pix_mp.height;
	apple_vxd_fill_decoded_pixfmt(ctx, &f->fmt.pix_mp);
}

static int apple_vxd_image_fmt_changed(struct apple_vxd_ctx *ctx,
				       enum apple_vxd_image_fmt image_fmt)
{
	if (image_fmt == APPLE_VXD_IMG_FMT_ANY)
		return ctx->image_fmt != image_fmt;

	return apple_vxd_find_decoded_fmt(ctx->coded_fmt_desc, image_fmt) !=
	       apple_vxd_find_decoded_fmt(ctx->coded_fmt_desc, ctx->image_fmt);
}

static int apple_vxd_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct apple_vxd_ctx *ctx =
		container_of(ctrl->handler, struct apple_vxd_ctx, ctrl_hdl);
	const struct apple_vxd_coded_fmt_desc *desc = ctx->coded_fmt_desc;

	if (desc->ops->try_ctrl)
		return desc->ops->try_ctrl(ctx, ctrl);
	return 0;
}

/*
 * Also runs from v4l2_ctrl_handler_setup() inside apple_vxd_ctx_init(), so
 * the m2m context (and its queues) must exist before the controls do.
 */
static int apple_vxd_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct apple_vxd_ctx *ctx =
		container_of(ctrl->handler, struct apple_vxd_ctx, ctrl_hdl);
	const struct apple_vxd_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	enum apple_vxd_image_fmt image_fmt;
	struct vb2_queue *vq;

	if (!desc->ops->get_image_fmt)
		return 0;

	image_fmt = desc->ops->get_image_fmt(ctx, ctrl);
	if (apple_vxd_image_fmt_changed(ctx, image_fmt)) {
		vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		if (vb2_is_busy(vq))
			return -EBUSY;

		ctx->image_fmt = image_fmt;
		apple_vxd_reset_decoded_fmt(ctx);
	}

	return 0;
}

const struct v4l2_ctrl_ops apple_vxd_ctrl_ops = {
	.try_ctrl = apple_vxd_try_ctrl,
	.s_ctrl = apple_vxd_s_ctrl,
};

static int apple_vxd_add_ctrls(struct apple_vxd_ctx *ctx,
			       const struct apple_vxd_ctrls *ctrls)
{
	unsigned int i;

	for (i = 0; i < ctrls->num_ctrls; i++) {
		v4l2_ctrl_new_custom(&ctx->ctrl_hdl, &ctrls->ctrls[i].cfg, ctx);
		if (ctx->ctrl_hdl.error)
			return ctx->ctrl_hdl.error;
	}

	return 0;
}

static int apple_vxd_init_ctrls(struct apple_vxd_ctx *ctx)
{
	struct apple_vxd_dev *vxd = ctx->vxd;
	unsigned int i, nctrls = 0;
	int ret;

	for (i = 0; i < vxd->num_coded_fmts; i++)
		nctrls += vxd->coded_fmts[i].ctrls->num_ctrls;

	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, nctrls);

	for (i = 0; i < vxd->num_coded_fmts; i++) {
		ret = apple_vxd_add_ctrls(ctx, vxd->coded_fmts[i].ctrls);
		if (ret)
			goto err_free_handler;
	}

	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);
	if (ret)
		goto err_free_handler;

	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	return ret;
}

/* --- handler bodies (context based; shared with the self-test) --------- */

static int apple_vxd_do_enum_framesizes(struct apple_vxd_ctx *ctx,
					struct v4l2_frmsizeenum *fsize)
{
	const struct apple_vxd_coded_fmt_desc *desc;

	if (fsize->index != 0)
		return -EINVAL;

	desc = apple_vxd_find_coded_fmt_desc(ctx, fsize->pixel_format);
	if (!desc)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = 1;
	fsize->stepwise.max_width = desc->frmsize.max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = desc->frmsize.max_height;
	fsize->stepwise.step_height = 1;

	return 0;
}

/*
 * The V4L2 core fills device_caps, capabilities and bus_info (from the
 * node's parent device, "platform:207f00000.vxd", matching the media
 * device's default) before calling the driver.
 */
static void apple_vxd_fill_querycap(struct apple_vxd_dev *vxd,
				    struct v4l2_capability *cap)
{
	strscpy(cap->driver, APPLE_VXD_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, APPLE_VXD_DRV_NAME, sizeof(cap->card));
}

static int apple_vxd_do_try_capture_fmt(struct apple_vxd_ctx *ctx,
					struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct apple_vxd_coded_fmt_desc *coded_desc = ctx->coded_fmt_desc;

	if (!apple_vxd_is_valid_fmt(ctx, pix_mp->pixelformat, ctx->image_fmt))
		pix_mp->pixelformat =
			apple_vxd_enum_decoded_fmt(ctx, 0, ctx->image_fmt);

	/* Always apply the frmsize constraint of the coded end. */
	pix_mp->width = max(pix_mp->width, ctx->coded_fmt.fmt.pix_mp.width);
	pix_mp->height = max(pix_mp->height, ctx->coded_fmt.fmt.pix_mp.height);
	v4l2_apply_frmsize_constraints(&pix_mp->width, &pix_mp->height,
				       &coded_desc->frmsize);

	apple_vxd_fill_decoded_pixfmt(ctx, pix_mp);
	pix_mp->field = V4L2_FIELD_NONE;

	return 0;
}

static int apple_vxd_do_try_output_fmt(struct apple_vxd_ctx *ctx,
				       struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct apple_vxd_coded_fmt_desc *desc;

	desc = apple_vxd_find_coded_fmt_desc(ctx, pix_mp->pixelformat);
	if (!desc) {
		desc = &ctx->vxd->coded_fmts[0];
		pix_mp->pixelformat = desc->fourcc;
	}

	v4l2_apply_frmsize_constraints(&pix_mp->width, &pix_mp->height,
				       &desc->frmsize);

	pix_mp->field = V4L2_FIELD_NONE;
	/* All coded formats are considered single planar for now. */
	pix_mp->num_planes = 1;

	if (desc->ops->adjust_fmt) {
		int ret;

		ret = desc->ops->adjust_fmt(ctx, f);
		if (ret)
			return ret;
	}

	/*
	 * One access unit per buffer (frame-based decode): one byte per
	 * pixel covers a level 4.2 1080p intra picture (about 2 MiB),
	 * userspace such as GStreamer leaves sizeimage to the driver.
	 */
	pix_mp->plane_fmt[0].sizeimage =
		max3(pix_mp->plane_fmt[0].sizeimage,
		     (u32)APPLE_VXD_MIN_BITSTREAM_SIZE,
		     (u32)PAGE_ALIGN(pix_mp->width * pix_mp->height));
	return 0;
}

static int apple_vxd_do_s_capture_fmt(struct apple_vxd_ctx *ctx,
				      struct v4l2_format *f)
{
	struct vb2_queue *vq;
	int ret;

	/* Change not allowed if queue is busy */
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = apple_vxd_do_try_capture_fmt(ctx, f);
	if (ret)
		return ret;

	ctx->decoded_fmt = *f;
	return 0;
}

static int apple_vxd_do_s_output_fmt(struct apple_vxd_ctx *ctx,
				     struct v4l2_format *f)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	const struct apple_vxd_coded_fmt_desc *desc;
	struct v4l2_format *cap_fmt;
	struct vb2_queue *peer_vq, *vq;
	int ret;

	/*
	 * The iOS parser is one-stream-configuration-per-instance ("SPS
	 * changes not supported!"); resolution changes only within the
	 * same pixelformat.  Never while streaming.
	 */
	vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (vb2_is_streaming(vq) ||
	    (vb2_is_busy(vq) &&
	     f->fmt.pix_mp.pixelformat != ctx->coded_fmt.fmt.pix_mp.pixelformat))
		return -EBUSY;

	peer_vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(peer_vq))
		return -EBUSY;

	ret = apple_vxd_do_try_output_fmt(ctx, f);
	if (ret)
		return ret;

	desc = apple_vxd_find_coded_fmt_desc(ctx, f->fmt.pix_mp.pixelformat);
	if (!desc)
		return -EINVAL;
	ctx->coded_fmt_desc = desc;
	ctx->coded_fmt = *f;

	/*
	 * Reset the decoded format; the user is mandated to set it again
	 * after we return.
	 */
	ctx->image_fmt = APPLE_VXD_IMG_FMT_ANY;
	apple_vxd_reset_decoded_fmt(ctx);

	/* Propagate colorspace information to capture. */
	cap_fmt = &ctx->decoded_fmt;
	cap_fmt->fmt.pix_mp.colorspace = f->fmt.pix_mp.colorspace;
	cap_fmt->fmt.pix_mp.xfer_func = f->fmt.pix_mp.xfer_func;
	cap_fmt->fmt.pix_mp.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	cap_fmt->fmt.pix_mp.quantization = f->fmt.pix_mp.quantization;

	return 0;
}

static int apple_vxd_do_enum_output_fmt(struct apple_vxd_ctx *ctx,
					struct v4l2_fmtdesc *f)
{
	if (f->index >= ctx->vxd->num_coded_fmts)
		return -EINVAL;
	f->pixelformat = ctx->vxd->coded_fmts[f->index].fourcc;
	return 0;
}

static int apple_vxd_do_enum_capture_fmt(struct apple_vxd_ctx *ctx,
					 struct v4l2_fmtdesc *f)
{
	u32 fourcc;

	fourcc = apple_vxd_enum_decoded_fmt(ctx, f->index, ctx->image_fmt);
	if (!fourcc)
		return -EINVAL;
	f->pixelformat = fourcc;
	return 0;
}

/* --- ioctl entry points ------------------------------------------------ */

/*
 * Driver-side STEP lines: the ioctl-entry line (apple_vxd_ioctl) names the
 * command before the V4L2 core runs; these name the driver handler, so a
 * crash between the two is in core code.
 */
static void apple_vxd_step_handler(struct file *file, const char *what)
{
	struct apple_vxd_dev *vxd = video_drvdata(file);

	vxd_step_io(vxd->dev, "handler: %s", what);
}

/*
 * Only a cached CAPTURE queue has allow_cache_hints set (queue_init); ask
 * vb2 for non-coherent memory on it whatever the application passed.
 */
static bool apple_vxd_cached_queue(struct file *file, u32 type, u32 memory)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(file_to_vxd_ctx(file)->fh.m2m_ctx,
					       type);

	return vq->allow_cache_hints && memory == V4L2_MEMORY_MMAP;
}

static int apple_vxd_reqbufs(struct file *file, void *priv,
			     struct v4l2_requestbuffers *rb)
{
	if (apple_vxd_cached_queue(file, rb->type, rb->memory))
		rb->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return v4l2_m2m_ioctl_reqbufs(file, priv, rb);
}

static int apple_vxd_create_bufs(struct file *file, void *priv,
				 struct v4l2_create_buffers *create)
{
	if (apple_vxd_cached_queue(file, create->format.type, create->memory))
		create->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
	return v4l2_m2m_ioctl_create_bufs(file, priv, create);
}

static int apple_vxd_enum_framesizes(struct file *file, void *priv,
				     struct v4l2_frmsizeenum *fsize)
{
	apple_vxd_step_handler(file, "enum_framesizes");
	return apple_vxd_do_enum_framesizes(file_to_vxd_ctx(file), fsize);
}

static int apple_vxd_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	apple_vxd_step_handler(file, "querycap");
	apple_vxd_fill_querycap(video_drvdata(file), cap);
	return 0;
}

static int apple_vxd_try_capture_fmt(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	apple_vxd_step_handler(file, "try_fmt capture");
	return apple_vxd_do_try_capture_fmt(file_to_vxd_ctx(file), f);
}

static int apple_vxd_try_output_fmt(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	apple_vxd_step_handler(file, "try_fmt output");
	return apple_vxd_do_try_output_fmt(file_to_vxd_ctx(file), f);
}

static int apple_vxd_s_capture_fmt(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	apple_vxd_step_handler(file, "s_fmt capture");
	return apple_vxd_do_s_capture_fmt(file_to_vxd_ctx(file), f);
}

static int apple_vxd_s_output_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	apple_vxd_step_handler(file, "s_fmt output");
	return apple_vxd_do_s_output_fmt(file_to_vxd_ctx(file), f);
}

static int apple_vxd_g_output_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	apple_vxd_step_handler(file, "g_fmt output");
	*f = file_to_vxd_ctx(file)->coded_fmt;
	return 0;
}

static int apple_vxd_g_capture_fmt(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	apple_vxd_step_handler(file, "g_fmt capture");
	*f = file_to_vxd_ctx(file)->decoded_fmt;
	return 0;
}

static int apple_vxd_enum_output_fmt(struct file *file, void *priv,
				     struct v4l2_fmtdesc *f)
{
	apple_vxd_step_handler(file, "enum_fmt output");
	return apple_vxd_do_enum_output_fmt(file_to_vxd_ctx(file), f);
}

static int apple_vxd_enum_capture_fmt(struct file *file, void *priv,
				      struct v4l2_fmtdesc *f)
{
	apple_vxd_step_handler(file, "enum_fmt capture");
	return apple_vxd_do_enum_capture_fmt(file_to_vxd_ctx(file), f);
}

static const struct v4l2_ioctl_ops apple_vxd_ioctl_ops = {
	.vidioc_querycap = apple_vxd_querycap,
	.vidioc_enum_framesizes = apple_vxd_enum_framesizes,

	.vidioc_try_fmt_vid_cap_mplane = apple_vxd_try_capture_fmt,
	.vidioc_try_fmt_vid_out_mplane = apple_vxd_try_output_fmt,
	.vidioc_s_fmt_vid_cap_mplane = apple_vxd_s_capture_fmt,
	.vidioc_s_fmt_vid_out_mplane = apple_vxd_s_output_fmt,
	.vidioc_g_fmt_vid_out_mplane = apple_vxd_g_output_fmt,
	.vidioc_g_fmt_vid_cap_mplane = apple_vxd_g_capture_fmt,
	.vidioc_enum_fmt_vid_out = apple_vxd_enum_output_fmt,
	.vidioc_enum_fmt_vid_cap = apple_vxd_enum_capture_fmt,

	.vidioc_reqbufs = apple_vxd_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = apple_vxd_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
};

static const struct {
	unsigned int cmd;
	const char *name;
} apple_vxd_ioctl_names[] = {
	{ VIDIOC_QUERYCAP, "VIDIOC_QUERYCAP" },
	{ VIDIOC_ENUM_FMT, "VIDIOC_ENUM_FMT" },
	{ VIDIOC_G_FMT, "VIDIOC_G_FMT" },
	{ VIDIOC_S_FMT, "VIDIOC_S_FMT" },
	{ VIDIOC_TRY_FMT, "VIDIOC_TRY_FMT" },
	{ VIDIOC_ENUM_FRAMESIZES, "VIDIOC_ENUM_FRAMESIZES" },
	{ VIDIOC_ENUM_FRAMEINTERVALS, "VIDIOC_ENUM_FRAMEINTERVALS" },
	{ VIDIOC_REQBUFS, "VIDIOC_REQBUFS" },
	{ VIDIOC_QUERYBUF, "VIDIOC_QUERYBUF" },
	{ VIDIOC_QBUF, "VIDIOC_QBUF" },
	{ VIDIOC_DQBUF, "VIDIOC_DQBUF" },
	{ VIDIOC_PREPARE_BUF, "VIDIOC_PREPARE_BUF" },
	{ VIDIOC_CREATE_BUFS, "VIDIOC_CREATE_BUFS" },
	{ VIDIOC_EXPBUF, "VIDIOC_EXPBUF" },
	{ VIDIOC_STREAMON, "VIDIOC_STREAMON" },
	{ VIDIOC_STREAMOFF, "VIDIOC_STREAMOFF" },
	{ VIDIOC_QUERYCTRL, "VIDIOC_QUERYCTRL" },
	{ VIDIOC_QUERY_EXT_CTRL, "VIDIOC_QUERY_EXT_CTRL" },
	{ VIDIOC_QUERYMENU, "VIDIOC_QUERYMENU" },
	{ VIDIOC_G_CTRL, "VIDIOC_G_CTRL" },
	{ VIDIOC_S_CTRL, "VIDIOC_S_CTRL" },
	{ VIDIOC_G_EXT_CTRLS, "VIDIOC_G_EXT_CTRLS" },
	{ VIDIOC_S_EXT_CTRLS, "VIDIOC_S_EXT_CTRLS" },
	{ VIDIOC_TRY_EXT_CTRLS, "VIDIOC_TRY_EXT_CTRLS" },
	{ VIDIOC_SUBSCRIBE_EVENT, "VIDIOC_SUBSCRIBE_EVENT" },
	{ VIDIOC_UNSUBSCRIBE_EVENT, "VIDIOC_UNSUBSCRIBE_EVENT" },
	{ VIDIOC_DQEVENT, "VIDIOC_DQEVENT" },
	{ VIDIOC_DECODER_CMD, "VIDIOC_DECODER_CMD" },
	{ VIDIOC_TRY_DECODER_CMD, "VIDIOC_TRY_DECODER_CMD" },
	{ VIDIOC_G_PRIORITY, "VIDIOC_G_PRIORITY" },
	{ VIDIOC_S_PRIORITY, "VIDIOC_S_PRIORITY" },
	{ VIDIOC_ENUMINPUT, "VIDIOC_ENUMINPUT" },
	{ VIDIOC_ENUMOUTPUT, "VIDIOC_ENUMOUTPUT" },
	{ VIDIOC_G_INPUT, "VIDIOC_G_INPUT" },
	{ VIDIOC_G_OUTPUT, "VIDIOC_G_OUTPUT" },
	{ VIDIOC_G_PARM, "VIDIOC_G_PARM" },
	{ VIDIOC_LOG_STATUS, "VIDIOC_LOG_STATUS" },
};

static const char *apple_vxd_ioctl_name(unsigned int cmd)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(apple_vxd_ioctl_names); i++)
		if (apple_vxd_ioctl_names[i].cmd == cmd)
			return apple_vxd_ioctl_names[i].name;
	return "unknown";
}

/*
 * Every ioctl on the video node passes here first: the STEP line names
 * the command (and the caller) before the V4L2 core or a handler runs.
 * The device comes from the node, not the file handle, so the line is
 * printed even if the per-file context is broken.
 */
static long apple_vxd_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct apple_vxd_dev *vxd = video_drvdata(file);

	vxd_step_io(vxd->dev, "ioctl %s (%#x) by %s[%d]",
		    apple_vxd_ioctl_name(cmd), cmd, current->comm,
		    task_pid_nr(current));
	return video_ioctl2(file, cmd, arg);
}

/* --- vb2 / m2m / media ops --------------------------------------------- */

static int apple_vxd_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
				 unsigned int *num_planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f;
	unsigned int i;

	f = V4L2_TYPE_IS_OUTPUT(vq->type) ? &ctx->coded_fmt : &ctx->decoded_fmt;

	if (*num_planes) {
		if (*num_planes != f->fmt.pix_mp.num_planes)
			return -EINVAL;

		for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
			if (sizes[i] < f->fmt.pix_mp.plane_fmt[i].sizeimage)
				return -EINVAL;
		}
	} else {
		*num_planes = f->fmt.pix_mp.num_planes;
		for (i = 0; i < f->fmt.pix_mp.num_planes; i++)
			sizes[i] = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	return 0;
}

static int apple_vxd_buf_prepare(struct vb2_buffer *vb)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_format *f;
	unsigned int i;

	f = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ? &ctx->coded_fmt :
		&ctx->decoded_fmt;

	for (i = 0; i < f->fmt.pix_mp.num_planes; ++i) {
		u32 sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb, i) < sizeimage)
			return -EINVAL;
	}

	/*
	 * Buffer's bytesused must be written by driver for CAPTURE buffers.
	 * (for OUTPUT buffers, if userspace passes 0 bytesused, v4l2-core
	 * sets it to buffer length).
	 */
	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0,
				      f->fmt.pix_mp.plane_fmt[0].sizeimage);

	return 0;
}

static struct apple_vxd_buffer *vxd_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct apple_vxd_buffer, base.vb);
}

/*
 * R3b: every buffer gets a GART mapping for its whole life.  The command
 * stream names the bitstream by device address (the 0xa load command,
 * ds-vxd-cmdstream.md section 3.3) and the render target / DPB surfaces
 * the same way (section 6).  The mapping is the identity window
 * (low 32 bits of the DMA address + va_offset), 4 KiB granular; vb2
 * dma-contig buffers are physically contiguous here (no IOMMU on this
 * node) and start page aligned.  Only page-table memory is written: no
 * register access, so this is safe with the firmware down.
 */
static int apple_vxd_buf_init(struct vb2_buffer *vb)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct apple_vxd_buffer *buf = vxd_buf(to_vb2_v4l2_buffer(vb));
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	dma_addr_t start = ALIGN_DOWN(dma, VXD_GART_PAGE_SIZE);
	size_t len = vb2_plane_size(vb, 0);
	int ret;

	ret = apple_vxd_gart_map(ctx->vxd, start,
				 ALIGN(dma + len, VXD_GART_PAGE_SIZE) - start,
				 &buf->map);
	if (ret) {
		dev_err(ctx->vxd->dev,
			"%s buffer %u: GART map of %pad (+%zu) failed: %d\n",
			V4L2_TYPE_IS_OUTPUT(vb->type) ? "output" : "capture",
			vb->index, &dma, len, ret);
		return ret;
	}
	buf->va = buf->map.va + (u32)(dma - start);
	vxd_step_io(ctx->vxd->dev,
		    "vb2: %s buffer %u GART-mapped: dma %pad, %zu bytes -> device VA %#x",
		    V4L2_TYPE_IS_OUTPUT(vb->type) ? "output" : "capture",
		    vb->index, &dma, len, buf->va);

	if (V4L2_TYPE_IS_CAPTURE(vb->type) &&
	    ctx->coded_fmt_desc->ops->capture_aux_size) {
		size_t aux = ctx->coded_fmt_desc->ops->capture_aux_size(ctx);

		if (aux) {
			ret = apple_vxd_dev_buf_alloc(ctx->vxd, aux, &buf->aux);
			if (ret) {
				dev_err(ctx->vxd->dev,
					"capture buffer %u: %zu-byte side buffer: %d\n",
					vb->index, aux, ret);
				apple_vxd_gart_unmap(ctx->vxd, &buf->map);
				buf->va = 0;
				return ret;
			}
			vxd_step_io(ctx->vxd->dev,
				    "vb2: capture buffer %u side buffer %zu bytes at device VA %#x",
				    vb->index, aux, buf->aux.va);
		}
	}
	return 0;
}

static void apple_vxd_buf_cleanup(struct vb2_buffer *vb)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct apple_vxd_buffer *buf = vxd_buf(to_vb2_v4l2_buffer(vb));

	apple_vxd_dev_buf_free(ctx->vxd, &buf->aux);
	apple_vxd_gart_unmap(ctx->vxd, &buf->map);
	buf->va = 0;
}

/* Device VA of plane 0 of a queued buffer (valid between buf_init and cleanup). */
u32 apple_vxd_buf_va(struct vb2_v4l2_buffer *vbuf)
{
	return vxd_buf(vbuf)->va;
}

/* Device VA of a CAPTURE buffer's side buffer, 0 if it has none. */
u32 apple_vxd_buf_aux_va(struct vb2_v4l2_buffer *vbuf)
{
	return vxd_buf(vbuf)->aux.va;
}

static void apple_vxd_buf_queue(struct vb2_buffer *vb)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int apple_vxd_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static void apple_vxd_buf_request_complete(struct vb2_buffer *vb)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
}

static int apple_vxd_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(q);
	int ret;

	vxd_step_io(ctx->vxd->dev, "vb2: start_streaming %s",
		    V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture");
	if (V4L2_TYPE_IS_CAPTURE(q->type))
		return 0;

	if (!ctx->coded_fmt_desc->ops->start)
		return 0;

	ret = ctx->coded_fmt_desc->ops->start(ctx);
	if (ret) {
		dev_err(ctx->vxd->dev,
			"failed to start decode context: %d\n", ret);
		return ret;
	}

	return 0;
}

static void apple_vxd_queue_cleanup(struct vb2_queue *vq, u32 state)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(vq);

	while (true) {
		struct vb2_v4l2_buffer *vbuf;

		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vbuf)
			break;

		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req,
					   &ctx->ctrl_hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static void apple_vxd_stop_streaming(struct vb2_queue *q)
{
	struct apple_vxd_ctx *ctx = vb2_get_drv_priv(q);

	vxd_step_io(ctx->vxd->dev, "vb2: stop_streaming %s",
		    V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture");
	if (V4L2_TYPE_IS_OUTPUT(q->type) && ctx->coded_fmt_desc->ops->stop)
		ctx->coded_fmt_desc->ops->stop(ctx);

	apple_vxd_queue_cleanup(q, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops apple_vxd_queue_ops = {
	.queue_setup = apple_vxd_queue_setup,
	.buf_init = apple_vxd_buf_init,
	.buf_cleanup = apple_vxd_buf_cleanup,
	.buf_prepare = apple_vxd_buf_prepare,
	.buf_queue = apple_vxd_buf_queue,
	.buf_out_validate = apple_vxd_buf_out_validate,
	.buf_request_complete = apple_vxd_buf_request_complete,
	.start_streaming = apple_vxd_start_streaming,
	.stop_streaming = apple_vxd_stop_streaming,
};

/* The media node's only driver hooks are the request ops. */
static int apple_vxd_request_validate(struct media_request *req)
{
	struct apple_vxd_dev *vxd = container_of(req->mdev, struct apple_vxd_dev,
						 mdev);
	unsigned int count;

	vxd_step_io(vxd->dev, "media: request validate");
	count = vb2_request_buffer_cnt(req);
	if (!count)
		return -ENOENT;
	else if (count > 1)
		return -EINVAL;

	return vb2_request_validate(req);
}

static void apple_vxd_request_queue(struct media_request *req)
{
	struct apple_vxd_dev *vxd = container_of(req->mdev, struct apple_vxd_dev,
						 mdev);

	vxd_step_io(vxd->dev, "media: request queue");
	v4l2_m2m_request_queue(req);
}

static const struct media_device_ops apple_vxd_media_ops = {
	.req_validate = apple_vxd_request_validate,
	.req_queue = apple_vxd_request_queue,
};

static void apple_vxd_device_run(void *priv)
{
	struct apple_vxd_ctx *ctx = priv;
	struct apple_vxd_dev *vxd = ctx->vxd;
	const struct apple_vxd_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	int ret;

	vxd_step_io(vxd->dev, "m2m: device_run (firmware %s)",
		    vxd->hw_ready ? "alive" : "not booted");
	if (WARN_ON(!desc))
		return;

	ret = pm_runtime_resume_and_get(vxd->dev);
	if (ret < 0) {
		apple_vxd_job_finish_no_pm(ctx, VB2_BUF_STATE_ERROR);
		return;
	}

	ret = desc->ops->run(ctx);
	if (ret)
		apple_vxd_job_finish(ctx, VB2_BUF_STATE_ERROR);
}

static const struct v4l2_m2m_ops apple_vxd_m2m_ops = {
	.device_run = apple_vxd_device_run,
};

static int apple_vxd_queue_init(void *priv, struct vb2_queue *src_vq,
				struct vb2_queue *dst_vq)
{
	struct apple_vxd_ctx *ctx = priv;
	struct apple_vxd_dev *vxd = ctx->vxd;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &apple_vxd_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	/*
	 * Frame-based decode: the driver walks the access unit and parses
	 * the slice headers itself, so the bitstream needs a kernel mapping.
	 */
	src_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES;
	src_vq->buf_struct_size = sizeof(struct apple_vxd_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &vxd->vdev_lock;
	src_vq->dev = vxd->v4l2_dev.dev;
	src_vq->supports_requests = true;
	src_vq->requires_requests = true;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->bidirectional = true;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	if (READ_ONCE(apple_vxd_capture_cached)) {
		/*
		 * dma_alloc_noncontiguous() takes no other attribute; a
		 * non-coherent buffer has no kernel mapping until vb2 asks.
		 */
		dst_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES;
		dst_vq->allow_cache_hints = 1;
	} else {
		dst_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES |
				    DMA_ATTR_NO_KERNEL_MAPPING;
	}
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &apple_vxd_queue_ops;
	dst_vq->buf_struct_size = sizeof(struct apple_vxd_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &vxd->vdev_lock;
	dst_vq->dev = vxd->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

/* --- open / release ---------------------------------------------------- */

/*
 * The driver half of open(): everything except the v4l2_fh bookkeeping,
 * which is V4L2 core and needs a real file.  @who prefixes the STEP lines
 * ("open" or "self-test").  Order matters: the control handler setup runs
 * s_ctrl, which inspects the m2m queues, so the m2m context comes first.
 */
static int apple_vxd_ctx_init(struct apple_vxd_ctx *ctx, const char *who)
{
	struct apple_vxd_dev *vxd = ctx->vxd;
	int ret;

	/*
	 * Round 2's oops: the coded-format table was never filled and the
	 * first open() (udev's v4l_id) read coded_fmts[0] at address 0.
	 */
	if (!vxd->coded_fmts || !vxd->num_coded_fmts || !vxd->m2m_dev) {
		dev_err(vxd->dev, "%s: V4L2 state missing (coded fmts %u, m2m %s)\n",
			who, vxd->num_coded_fmts, vxd->m2m_dev ? "ok" : "none");
		return -ENODEV;
	}

	vxd_step(vxd->dev, "%s: format defaults", who);
	apple_vxd_reset_coded_fmt(ctx);
	apple_vxd_reset_decoded_fmt(ctx);

	vxd_step(vxd->dev, "%s: m2m context + vb2 queues", who);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vxd->m2m_dev, ctx,
					    apple_vxd_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		ctx->fh.m2m_ctx = NULL;
		return ret;
	}

	vxd_step(vxd->dev, "%s: control handler + setup", who);
	ret = apple_vxd_init_ctrls(ctx);
	if (ret) {
		v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
		ctx->fh.m2m_ctx = NULL;
		return ret;
	}

	return 0;
}

/* The driver half of release(). */
static void apple_vxd_ctx_fini(struct apple_vxd_ctx *ctx)
{
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
}

static int apple_vxd_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct apple_vxd_dev *vxd = video_drvdata(filp);
	struct apple_vxd_ctx *ctx;
	int ret;

	vxd_step(vxd->dev, "open: /dev/%s by %s[%d]",
		 video_device_node_name(vdev), current->comm,
		 task_pid_nr(current));

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	ctx->vxd = vxd;
	v4l2_fh_init(&ctx->fh, vdev);

	ret = apple_vxd_ctx_init(ctx, "open");
	if (ret) {
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}

	v4l2_fh_add(&ctx->fh, filp);
	return 0;
}

static int apple_vxd_release(struct file *filp)
{
	struct apple_vxd_dev *vxd = video_drvdata(filp);
	struct apple_vxd_ctx *ctx = file_to_vxd_ctx(filp);

	vxd_step(vxd->dev, "release: /dev/%s by %s[%d]",
		 video_device_node_name(&vxd->vdev), current->comm,
		 task_pid_nr(current));

	v4l2_fh_del(&ctx->fh, filp);
	apple_vxd_ctx_fini(ctx);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static int apple_vxd_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct apple_vxd_dev *vxd = video_drvdata(file);

	vxd_step_io(vxd->dev, "mmap: offset %#lx", vma->vm_pgoff << PAGE_SHIFT);
	return v4l2_m2m_fop_mmap(file, vma);
}

static const struct v4l2_file_operations apple_vxd_fops = {
	.owner = THIS_MODULE,
	.open = apple_vxd_open,
	.release = apple_vxd_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = apple_vxd_ioctl,
	.mmap = apple_vxd_mmap,
};

/* --- register-time self-test ------------------------------------------- */

static int apple_vxd_selftest_check(struct apple_vxd_dev *vxd, int ret,
				    bool ok, const char *what)
{
	if (!ret && ok)
		return 0;

	dev_err(vxd->dev, "v4l2 self-test FAILED at %s (ret %d)\n", what, ret);
	return ret ? ret : -EINVAL;
}

/*
 * Runs the driver half of open(), every format/enum/framesize handler
 * body, the querycap fill and the driver half of release() on a private
 * context, BEFORE any node is exposed.  The round-2 class of bug (state
 * that some later step was supposed to create) then fails the V4L2
 * registration with a log line naming the check, instead of oopsing under
 * udev.  The v4l2_fh bookkeeping is V4L2 core and is not run:
 * v4l2_fh_init() on a not yet registered node would also mark its
 * priority ioctls as disabled.
 */
static int apple_vxd_v4l2_selftest(struct apple_vxd_dev *vxd)
{
	const struct apple_vxd_coded_fmt_desc *desc;
	struct v4l2_frmsizeenum fsize = { };
	struct v4l2_capability cap = { };
	struct v4l2_fmtdesc fdesc = { };
	struct v4l2_format f = { };
	struct apple_vxd_ctx *ctx;
	unsigned int i;
	int ret = 0;

	vxd_step(vxd->dev, "self-test: static tables + node wiring");
	for (i = 0; i < vxd->num_coded_fmts; i++) {
		desc = &vxd->coded_fmts[i];
		ret = apple_vxd_selftest_check(vxd, 0,
					       desc->ops && desc->ops->run &&
					       desc->ctrls && desc->num_decoded_fmts &&
					       desc->decoded_fmts,
					       "coded format descriptor");
		if (ret)
			return ret;
	}
	ret = apple_vxd_selftest_check(vxd, 0,
				       vxd->v4l2_dev.dev && vxd->vdev.v4l2_dev &&
				       vxd->vdev.fops && vxd->vdev.ioctl_ops &&
				       vxd->vdev.device_caps,
				       "video node wiring");
	if (ret)
		return ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->vxd = vxd;

	/* open() */
	ret = apple_vxd_ctx_init(ctx, "self-test");
	if (ret) {
		dev_err(vxd->dev, "v4l2 self-test FAILED at open (ret %d)\n", ret);
		kfree(ctx);
		return ret;
	}

	/* VIDIOC_QUERYCAP (the driver part; the core fills caps/bus_info) */
	vxd_step(vxd->dev, "self-test: querycap");
	apple_vxd_fill_querycap(vxd, &cap);
	ret = apple_vxd_selftest_check(vxd, 0, cap.driver[0] && cap.card[0],
				       "querycap");
	if (ret)
		goto out;

	/* VIDIOC_ENUM_FMT */
	vxd_step(vxd->dev, "self-test: enum_fmt");
	fdesc.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	ret = apple_vxd_do_enum_output_fmt(ctx, &fdesc);
	ret = apple_vxd_selftest_check(vxd, ret,
				       fdesc.pixelformat == vxd->coded_fmts[0].fourcc,
				       "enum_fmt output 0");
	if (ret)
		goto out;
	fdesc.index = vxd->num_coded_fmts;
	ret = apple_vxd_selftest_check(vxd, 0,
				       apple_vxd_do_enum_output_fmt(ctx, &fdesc) == -EINVAL,
				       "enum_fmt output end");
	if (ret)
		goto out;
	memset(&fdesc, 0, sizeof(fdesc));
	fdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	ret = apple_vxd_do_enum_capture_fmt(ctx, &fdesc);
	ret = apple_vxd_selftest_check(vxd, ret, fdesc.pixelformat,
				       "enum_fmt capture 0");
	if (ret)
		goto out;

	/* VIDIOC_G_FMT */
	vxd_step(vxd->dev, "self-test: g_fmt");
	ret = apple_vxd_selftest_check(vxd, 0,
				       ctx->coded_fmt.fmt.pix_mp.num_planes == 1 &&
				       ctx->coded_fmt.fmt.pix_mp.plane_fmt[0].sizeimage,
				       "g_fmt output");
	if (ret)
		goto out;
	ret = apple_vxd_selftest_check(vxd, 0,
				       ctx->decoded_fmt.fmt.pix_mp.num_planes &&
				       ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].sizeimage,
				       "g_fmt capture");
	if (ret)
		goto out;

	/* VIDIOC_TRY_FMT: oversized, unknown fourcc -> clamped coded fmt 0 */
	vxd_step(vxd->dev, "self-test: try_fmt");
	f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f.fmt.pix_mp.pixelformat = 0;
	f.fmt.pix_mp.width = 8192;
	f.fmt.pix_mp.height = 8192;
	ret = apple_vxd_do_try_output_fmt(ctx, &f);
	ret = apple_vxd_selftest_check(vxd, ret,
				       f.fmt.pix_mp.pixelformat == vxd->coded_fmts[0].fourcc &&
				       f.fmt.pix_mp.width <= APPLE_VXD_MAX_PIC_W &&
				       f.fmt.pix_mp.height <= APPLE_VXD_MAX_PIC_H,
				       "try_fmt output");
	if (ret)
		goto out;
	f = ctx->decoded_fmt;
	ret = apple_vxd_do_try_capture_fmt(ctx, &f);
	ret = apple_vxd_selftest_check(vxd, ret,
				       f.fmt.pix_mp.plane_fmt[0].sizeimage,
				       "try_fmt capture");
	if (ret)
		goto out;

	/* VIDIOC_S_FMT: 1080p bitstream, then the matching NV12 surface */
	vxd_step(vxd->dev, "self-test: s_fmt");
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f.fmt.pix_mp.pixelformat = vxd->coded_fmts[0].fourcc;
	f.fmt.pix_mp.width = 1920;
	f.fmt.pix_mp.height = 1080;
	ret = apple_vxd_do_s_output_fmt(ctx, &f);
	ret = apple_vxd_selftest_check(vxd, ret,
				       ctx->decoded_fmt.fmt.pix_mp.width >= 1920 &&
				       ctx->decoded_fmt.fmt.pix_mp.height >= 1080,
				       "s_fmt output");
	if (ret)
		goto out;
	f = ctx->decoded_fmt;
	ret = apple_vxd_do_s_capture_fmt(ctx, &f);
	ret = apple_vxd_selftest_check(vxd, ret,
				       ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].sizeimage,
				       "s_fmt capture");
	if (ret)
		goto out;

	/* VIDIOC_ENUM_FRAMESIZES */
	vxd_step(vxd->dev, "self-test: enum_framesizes");
	fsize.pixel_format = vxd->coded_fmts[0].fourcc;
	ret = apple_vxd_do_enum_framesizes(ctx, &fsize);
	ret = apple_vxd_selftest_check(vxd, ret,
				       fsize.stepwise.max_width == APPLE_VXD_MAX_PIC_W,
				       "enum_framesizes");

out:
	/* release() */
	vxd_step(vxd->dev, "self-test: release");
	apple_vxd_ctx_fini(ctx);
	kfree(ctx);
	if (!ret)
		dev_info(vxd->dev,
			 "v4l2 self-test ok: open/querycap/enum/g/try/s_fmt/release\n");
	return ret;
}

/* --- registration ------------------------------------------------------ */

int apple_vxd_v4l2_init(struct apple_vxd_dev *vxd)
{
	int ret;

	if (!vxd->coded_fmts || !vxd->num_coded_fmts) {
		dev_err(vxd->dev, "v4l2: no coded formats registered, not exposing a node\n");
		return -ENODEV;
	}

	vxd_step(vxd->dev, "v4l2: register v4l2 device");
	ret = v4l2_device_register(vxd->dev, &vxd->v4l2_dev);
	if (ret) {
		dev_err(vxd->dev, "failed to register v4l2 device\n");
		return ret;
	}

	vxd_step(vxd->dev, "v4l2: init mem2mem device");
	vxd->m2m_dev = v4l2_m2m_init(&apple_vxd_m2m_ops);
	if (IS_ERR(vxd->m2m_dev)) {
		ret = PTR_ERR(vxd->m2m_dev);
		vxd->m2m_dev = NULL;
		v4l2_err(&vxd->v4l2_dev, "failed to init mem2mem device\n");
		goto err_unregister_v4l2;
	}

	vxd->vdev.lock = &vxd->vdev_lock;
	vxd->vdev.v4l2_dev = &vxd->v4l2_dev;
	vxd->vdev.fops = &apple_vxd_fops;
	vxd->vdev.release = video_device_release_empty;
	vxd->vdev.vfl_dir = VFL_DIR_M2M;
	vxd->vdev.device_caps = V4L2_CAP_STREAMING |
				V4L2_CAP_VIDEO_M2M_MPLANE;
	vxd->vdev.ioctl_ops = &apple_vxd_ioctl_ops;
	video_set_drvdata(&vxd->vdev, vxd);
	strscpy(vxd->vdev.name, APPLE_VXD_DRV_NAME, sizeof(vxd->vdev.name));

	/* Nothing is exposed yet: fail here, not under udev. */
	ret = apple_vxd_v4l2_selftest(vxd);
	if (ret)
		goto err_release_m2m;

	/* bus_info stays empty: media_device_init() derives it from mdev.dev. */
	vxd->mdev.dev = vxd->dev;
	strscpy(vxd->mdev.model, APPLE_VXD_DRV_NAME, sizeof(vxd->mdev.model));
	media_device_init(&vxd->mdev);
	vxd->mdev.ops = &apple_vxd_media_ops;
	vxd->v4l2_dev.mdev = &vxd->mdev;

	vxd_step(vxd->dev, "v4l2: register video device");
	ret = video_register_device(&vxd->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&vxd->v4l2_dev, "failed to register video device\n");
		goto err_cleanup_mc;
	}
	dev_info(vxd->dev, "v4l2: video device /dev/%s registered\n",
		 video_device_node_name(&vxd->vdev));

	vxd_step(vxd->dev, "v4l2: register m2m media controller");
	ret = v4l2_m2m_register_media_controller(vxd->m2m_dev, &vxd->vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		v4l2_err(&vxd->v4l2_dev,
			 "failed to initialize v4l2 m2m media controller\n");
		goto err_unregister_vdev;
	}

	vxd_step(vxd->dev, "v4l2: register media device");
	ret = media_device_register(&vxd->mdev);
	if (ret) {
		v4l2_err(&vxd->v4l2_dev, "failed to register media device\n");
		goto err_unregister_mc;
	}
	dev_info(vxd->dev, "v4l2: media device /dev/media%d registered (model %s, bus %s)\n",
		 vxd->mdev.devnode->minor, vxd->mdev.model, vxd->mdev.bus_info);

	/* Only now does remove() own a full teardown. */
	vxd->v4l2_ready = true;
	return 0;

err_unregister_mc:
	v4l2_m2m_unregister_media_controller(vxd->m2m_dev);
err_unregister_vdev:
	video_unregister_device(&vxd->vdev);
err_cleanup_mc:
	vxd->v4l2_dev.mdev = NULL;
	media_device_cleanup(&vxd->mdev);
err_release_m2m:
	v4l2_m2m_release(vxd->m2m_dev);
	vxd->m2m_dev = NULL;
err_unregister_v4l2:
	v4l2_device_unregister(&vxd->v4l2_dev);
	return ret;
}

void apple_vxd_v4l2_cleanup(struct apple_vxd_dev *vxd)
{
	vxd->v4l2_ready = false;
	media_device_unregister(&vxd->mdev);
	v4l2_m2m_unregister_media_controller(vxd->m2m_dev);
	video_unregister_device(&vxd->vdev);
	media_device_cleanup(&vxd->mdev);
	v4l2_m2m_release(vxd->m2m_dev);
	vxd->m2m_dev = NULL;
	v4l2_device_unregister(&vxd->v4l2_dev);
}

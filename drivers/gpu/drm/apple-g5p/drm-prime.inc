/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* PRIME for shmem GEM BOs.
 *
 * Export: Mesa WSI/client buffers and kmsro bookkeeping. The dma-buf adds
 * begin/end_cpu_access, so an importer that CPU-reads (simpledrm's blit,
 * llvmpipe) gets cache maintenance on the G5P DMA mapping. vmap/mmap reuse
 * the shmem helpers (write-combined when the device is not DMA-coherent).
 *
 * Import: foreign dma-bufs, e.g. simpledrm dumb buffers that the GPU renders
 * into for scanout. The attachment stays mapped (pinned) for the life of the
 * GEM object; VM_BIND maps its sg_table. Render jobs listing an imported BO
 * get whole-buffer cache maintenance and dma_resv fences (drm-render.inc).
 */

static int g5p_dmabuf_begin_cpu_access(struct dma_buf *buf,
				       enum dma_data_direction dir)
{
	struct drm_gem_object *obj = buf->priv;
	struct sg_table *sgt = to_drm_gem_shmem_obj(obj)->sgt;

	/* No device mapping yet means no device-owned cache lines either. */
	if (sgt)
		dma_sync_sgtable_for_cpu(obj->dev->dev, sgt, DMA_BIDIRECTIONAL);
	return 0;
}

static int g5p_dmabuf_end_cpu_access(struct dma_buf *buf,
				     enum dma_data_direction dir)
{
	struct drm_gem_object *obj = buf->priv;
	struct sg_table *sgt = to_drm_gem_shmem_obj(obj)->sgt;

	if (sgt)
		dma_sync_sgtable_for_device(obj->dev->dev, sgt,
					    DMA_BIDIRECTIONAL);
	return 0;
}

static const struct dma_buf_ops g5p_dmabuf_ops = {
	.attach = drm_gem_map_attach,
	.detach = drm_gem_map_detach,
	.map_dma_buf = drm_gem_map_dma_buf,
	.unmap_dma_buf = drm_gem_unmap_dma_buf,
	.release = drm_gem_dmabuf_release,
	.mmap = drm_gem_dmabuf_mmap,
	.vmap = drm_gem_dmabuf_vmap,
	.vunmap = drm_gem_dmabuf_vunmap,
	.begin_cpu_access = g5p_dmabuf_begin_cpu_access,
	.end_cpu_access = g5p_dmabuf_end_cpu_access,
};

static struct dma_buf *g5p_gem_prime_export(struct drm_gem_object *obj,
					    int flags)
{
	struct dma_buf_export_info info = {
		.exp_name = KBUILD_MODNAME,
		.owner = obj->dev->driver->fops->owner,
		.ops = &g5p_dmabuf_ops,
		.size = obj->size,
		.flags = flags,
		.priv = obj,
		.resv = obj->resv,
	};

	return drm_gem_dmabuf_export(obj->dev, &info);
}

static int g5p_prime_handle_to_fd(struct drm_device *dev, struct drm_file *file,
				  u32 handle, u32 flags, int *fd)
{
	return drm_gem_prime_handle_to_fd(dev, file, handle, flags, fd);
}

static int g5p_prime_fd_to_handle(struct drm_device *dev, struct drm_file *file,
				  int fd, u32 *handle)
{
	return drm_gem_prime_fd_to_handle(dev, file, fd, handle);
}

/* Our own export comes back as the same GEM object (the core's identity check
 * only knows its default ops); anything else is attached and mapped.
 */
static struct drm_gem_object *g5p_gem_prime_import(struct drm_device *dev,
						   struct dma_buf *buf)
{
	struct drm_gem_object *obj = buf->priv;

	if (buf->ops == &g5p_dmabuf_ops && obj->dev == dev) {
		drm_gem_object_get(obj);
		return obj;
	}
	return drm_gem_prime_import(dev, buf);
}

/* Foreign buffers are GPU READ|WRITE data, never executable, and must be
 * whole 16KiB GPU pages within the BO size limit.
 */
static struct drm_gem_object *
g5p_prime_import_sg_table(struct drm_device *dev,
			  struct dma_buf_attachment *attach,
			  struct sg_table *sgt)
{
	struct drm_gem_object *obj;

	if (attach->dmabuf->size & (G5P_GPU_PAGE - 1) ||
	    attach->dmabuf->size > G5P_MAX_BO_SIZE)
		return ERR_PTR(-EINVAL);
	obj = drm_gem_shmem_prime_import_sg_table(dev, attach, sgt);
	if (!IS_ERR(obj)) {
		to_g5p_bo(obj)->roles = DRM_G5P_BO_READ | DRM_G5P_BO_WRITE;
		g5p_bo_track(obj);
	}
	return obj;
}

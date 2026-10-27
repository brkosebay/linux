// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - the core's own two-level memory translation unit ("GART")
 *
 * The vxd node has no iommu-parent; instead AppleD5500 programs a page
 * table hierarchy of its own (AppleD5500::initialize_gart [E]):
 *
 *   - directory table: 1024 x 4-byte entries, one per 4 MiB of device VA,
 *     in a 4 KiB buffer allocated 64 KiB aligned
 *   - page tables:     1024 x 4-byte entries, one per 4 KiB of device VA,
 *     num_pts consecutive 4 KiB tables
 *   - a dummy page filling every unmapped directory/page slot
 *   - entries: ((PA >> 8) & 0xfffffff0) | 1 on the 40-bit physical address
 *   - the directory table address is programmed through MTX_MMU_DIR_ADR
 *     (0x1020) as (PA >> 8) in ProcessorInitialise [E]
 *
 * GART pages are 4 KiB whatever the kernel page size: every table walk
 * below steps VXD_GART_PAGE_SIZE, never PAGE_SIZE (the device kernel runs
 * 16 KiB pages, which rounds 1-3 got wrong).
 *
 * We pre-allocate variant->num_gart_pts page tables (512 = the kext's
 * "2GB mapping" for a 2 GiB device) and identity-map the low 32 bits of
 * every DMA address, so mapping is just writing the slot selected by the
 * address itself.  The tables are dma_alloc_coherent memory, so no cache
 * maintenance is needed when they change.
 */
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "apple-vxd.h"

/*
 * Coherent buffer with a bus-address alignment beyond PAGE_SIZE.  The
 * direct allocator hands out naturally aligned blocks in practice, so the
 * exact size is tried first; only a misaligned result is retried with
 * enough slack to align inside it.
 */
int apple_vxd_dma_alloc_aligned(struct apple_vxd_dev *vxd, size_t size,
				size_t align, struct apple_vxd_dma_buf *buf)
{
	struct device *dev = vxd->dev;
	size_t off;

	if (WARN_ON(!size || !is_power_of_2(align)))
		return -EINVAL;

	memset(buf, 0, sizeof(*buf));
	buf->raw_size = PAGE_ALIGN(size);
	buf->raw_vaddr = dma_alloc_coherent(dev, buf->raw_size, &buf->raw_dma,
					    GFP_KERNEL);
	if (!buf->raw_vaddr)
		return -ENOMEM;

	if (buf->raw_dma & (align - 1)) {
		dma_free_coherent(dev, buf->raw_size, buf->raw_vaddr,
				  buf->raw_dma);
		buf->raw_size = PAGE_ALIGN(size + align);
		buf->raw_vaddr = dma_alloc_coherent(dev, buf->raw_size,
						    &buf->raw_dma, GFP_KERNEL);
		if (!buf->raw_vaddr)
			return -ENOMEM;
	}

	off = ALIGN(buf->raw_dma, align) - buf->raw_dma;
	buf->vaddr = buf->raw_vaddr + off;
	buf->dma = buf->raw_dma + off;
	buf->size = size;
	return 0;
}

void apple_vxd_dma_free_aligned(struct apple_vxd_dev *vxd,
				struct apple_vxd_dma_buf *buf)
{
	if (buf->raw_vaddr)
		dma_free_coherent(vxd->dev, buf->raw_size, buf->raw_vaddr,
				  buf->raw_dma);
	memset(buf, 0, sizeof(*buf));
}

int apple_vxd_gart_init(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_gart *gart = &vxd->gart;
	struct device *dev = vxd->dev;
	u32 num_pts = vxd->variant->num_gart_pts;
	u32 i;
	int ret;

	mutex_init(&gart->lock);
	INIT_LIST_HEAD(&gart->mappings);

	ret = apple_vxd_dma_alloc_aligned(vxd,
					  VXD_GART_DIR_ENTRIES * sizeof(u32),
					  VXD_GART_DIR_ALIGN, &gart->dir_buf);
	if (ret)
		goto err;
	gart->dir = gart->dir_buf.vaddr;
	gart->dir_dma = gart->dir_buf.dma;

	gart->dummy = dma_alloc_coherent(dev, PAGE_SIZE, &gart->dummy_dma,
					 GFP_KERNEL);
	if (!gart->dummy) {
		ret = -ENOMEM;
		goto err_free_dir;
	}

	/* Page tables are one contiguous DMA block of num_pts 4 KiB tables:
	 * table i sits at pts_dma + i * 4 KiB, which is what the directory
	 * entry for i points at.
	 */
	gart->pts = dma_alloc_coherent(dev, (size_t)num_pts * VXD_GART_PT_SIZE,
				       &gart->pts_dma, GFP_KERNEL);
	if (!gart->pts) {
		ret = -ENOMEM;
		goto err_free_dummy;
	}
	gart->num_pts = num_pts;

	if (upper_32_bits(gart->pts_dma) >> (VXD_GART_PA_BITS - 32) ||
	    upper_32_bits(gart->dir_dma) >> (VXD_GART_PA_BITS - 32) ||
	    upper_32_bits(gart->dummy_dma) >> (VXD_GART_PA_BITS - 32)) {
		dev_err(dev, "GART tables above the %u-bit PA range\n",
			VXD_GART_PA_BITS);
		ret = -ERANGE;
		goto err_free_pts;
	}

	/* Unmapped VAs resolve to the dummy page, as in AppleD5500. */
	for (i = 0; i < num_pts * VXD_GART_PT_ENTRIES; i++)
		gart->pts[i] = VXD_GART_PTE(gart->dummy_dma);
	for (i = 0; i < VXD_GART_DIR_ENTRIES; i++) {
		if (i < num_pts)
			gart->dir[i] = VXD_GART_PTE(gart->pts_dma +
						   (dma_addr_t)i * VXD_GART_PT_SIZE);
		else
			gart->dir[i] = VXD_GART_PTE(gart->dummy_dma);
	}

	dev_info(dev,
		 "GART initialized: dir %pad (64 KiB aligned), %u x 4 KiB page tables at %pad, dummy %pad, dir[0] %#x\n",
		 &gart->dir_dma, num_pts, &gart->pts_dma, &gart->dummy_dma,
		 gart->dir[0]);

	return 0;

err_free_pts:
	dma_free_coherent(dev, (size_t)num_pts * VXD_GART_PT_SIZE, gart->pts,
			  gart->pts_dma);
err_free_dummy:
	dma_free_coherent(dev, PAGE_SIZE, gart->dummy, gart->dummy_dma);
err_free_dir:
	apple_vxd_dma_free_aligned(vxd, &gart->dir_buf);
err:
	memset(gart, 0, sizeof(*gart));
	return ret;
}

void apple_vxd_gart_fini(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_gart *gart = &vxd->gart;
	struct device *dev = vxd->dev;

	WARN_ON(gart->pts && !list_empty(&gart->mappings));

	if (gart->pts)
		dma_free_coherent(dev, (size_t)gart->num_pts * VXD_GART_PT_SIZE,
				  gart->pts, gart->pts_dma);
	if (gart->dummy)
		dma_free_coherent(dev, PAGE_SIZE, gart->dummy, gart->dummy_dma);
	apple_vxd_dma_free_aligned(vxd, &gart->dir_buf);
	memset(gart, 0, sizeof(*gart));
}

/* Slot of device VA @va in the page-table block. */
static u32 *vxd_gart_slot(struct apple_vxd_gart *gart, u32 va)
{
	u32 dir_idx = va >> 22;		/* 4 MiB per page table */
	u32 ent_idx = (va >> VXD_GART_PAGE_SHIFT) & (VXD_GART_PT_ENTRIES - 1);

	if (dir_idx >= gart->num_pts)
		return NULL;
	return &gart->pts[dir_idx * VXD_GART_PT_ENTRIES + ent_idx];
}

int apple_vxd_gart_map_at(struct apple_vxd_dev *vxd, dma_addr_t dma,
			  size_t size, u32 va_base,
			  struct apple_vxd_mapping *map)
{
	struct apple_vxd_gart *gart = &vxd->gart;
	u32 npages, i;

	memset(map, 0, sizeof(*map));
	INIT_LIST_HEAD(&map->list);

	if (!size || !IS_ALIGNED(size, VXD_GART_PAGE_SIZE) ||
	    !IS_ALIGNED(dma, VXD_GART_PAGE_SIZE) ||
	    !IS_ALIGNED(va_base, VXD_GART_PAGE_SIZE) ||
	    upper_32_bits(dma + size - 1) >> (VXD_GART_PA_BITS - 32))
		return -EINVAL;

	npages = size >> VXD_GART_PAGE_SHIFT;
	if ((u64)va_base + size > (u64)gart->num_pts << 22) {
		/* Device aperture beyond the mapped window. */
		dev_err(vxd->dev,
			"GART: VA %#x (DMA %pad+0x%zx) maps outside the %u page-table window\n",
			va_base, &dma, size, gart->num_pts);
		return -ENOSPC;
	}

	mutex_lock(&gart->lock);
	for (i = 0; i < npages; i++)
		*vxd_gart_slot(gart, va_base + (i << VXD_GART_PAGE_SHIFT)) =
			VXD_GART_PTE(dma + ((dma_addr_t)i << VXD_GART_PAGE_SHIFT));
	map->dma = dma;
	map->size = size;
	map->va = va_base;
	list_add_tail(&map->list, &gart->mappings);
	gart->dirty = true;
	mutex_unlock(&gart->lock);
	return 0;
}

int apple_vxd_gart_map(struct apple_vxd_dev *vxd, dma_addr_t dma, size_t size,
		       struct apple_vxd_mapping *map)
{
	return apple_vxd_gart_map_at(vxd, dma, size, (u32)dma + vxd->va_offset,
				     map);
}

void apple_vxd_gart_unmap(struct apple_vxd_dev *vxd, struct apple_vxd_mapping *map)
{
	struct apple_vxd_gart *gart = &vxd->gart;
	u32 npages, i;
	u32 *slot;

	if (!map->size)
		return;

	mutex_lock(&gart->lock);
	npages = map->size >> VXD_GART_PAGE_SHIFT;
	for (i = 0; i < npages; i++) {
		slot = vxd_gart_slot(gart, map->va + (i << VXD_GART_PAGE_SHIFT));
		if (slot)
			*slot = VXD_GART_PTE(gart->dummy_dma);
	}
	list_del(&map->list);
	gart->dirty = true;
	mutex_unlock(&gart->lock);
	memset(map, 0, sizeof(*map));
}

/*
 * A zeroed coherent buffer, 4 KiB aligned (every MSVDX base-address
 * register drops the low 12 bits), GART-mapped for its lifetime.
 */
int apple_vxd_dev_buf_alloc(struct apple_vxd_dev *vxd, size_t size,
			    struct apple_vxd_dev_buf *buf)
{
	int ret;

	ret = apple_vxd_dma_alloc_aligned(vxd, ALIGN(size, VXD_GART_PAGE_SIZE),
					  VXD_GART_PAGE_SIZE, &buf->mem);
	if (ret)
		return ret;
	memset(buf->mem.vaddr, 0, buf->mem.size);

	ret = apple_vxd_gart_map(vxd, buf->mem.dma, buf->mem.size, &buf->map);
	if (ret) {
		apple_vxd_dma_free_aligned(vxd, &buf->mem);
		return ret;
	}
	buf->va = buf->map.va;
	return 0;
}

void apple_vxd_dev_buf_free(struct apple_vxd_dev *vxd,
			    struct apple_vxd_dev_buf *buf)
{
	if (!buf->mem.raw_vaddr)
		return;
	apple_vxd_gart_unmap(vxd, &buf->map);
	apple_vxd_dma_free_aligned(vxd, &buf->mem);
	buf->va = 0;
}

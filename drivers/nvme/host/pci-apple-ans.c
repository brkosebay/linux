// SPDX-License-Identifier: GPL-2.0
/*
 * Apple ANS PCI NVMe glue for H8 and H9 SoCs.
 *
 * Copyright (C) 2020 Corellium LLC
 * Copyright (C) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/apple-ans.h>
#include <linux/bitmap.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/iopoll.h>
#include <linux/log2.h>
#include <linux/mempool.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci.h>

#include "pci-internal.h"

#define NVME_CMD_APPLE_ANS_FLATDMA	BIT(5)

#define APPLE_ANS_REG_INIT		0x1800
#define APPLE_ANS_REG_INIT_REGULAR	0
#define APPLE_ANS_REG_HMB_SIZE_REQ	0x1808
#define APPLE_ANS_REG_HMB_ALIGN_REQ	0x180c
#define APPLE_ANS_REG_HMB_BASE_LO	0x1810
#define APPLE_ANS_REG_HMB_BASE_HI	0x1814
#define APPLE_ANS_REG_HMB_SIZE		0x1818
#define APPLE_ANS_REG_CORE_MASK		0x1824
#define APPLE_ANS_REG_LOG_SIZE		0x1828
/*
 * S3E config space: the A10 template wrote 0x17c, 0x18c and 0x188, which are
 * the LTR max latencies and the L1 PM substates CTL2/CTL1 registers when the
 * LTR capability sits at 0x178 and L1SS at 0x180 (inferred). Look the
 * capabilities up instead of hardcoding the offsets. The values match the
 * ADT "pci-max-latency" and "pci-l1pm-control" (enable bits cleared).
 */
#define APPLE_ANS_LTR_MAX_LATENCY	0x10081008
#define APPLE_ANS_L1SS_CTL1_T8010	0x40550000
#define APPLE_ANS_L1SS_CTL1_S8000	0x40440000
#define APPLE_ANS_NVME_MAX_SECTORS \
	(APPLE_ANS_NVMMU_MAX_PAGES * APPLE_ANS_NVMMU_PAGE_SIZE >> SECTOR_SHIFT)

struct apple_ans_nvme_req {
	u64 pages[APPLE_ANS_NVMMU_MAX_PAGES];
};

static bool apple_ans_dart_hmb;
module_param(apple_ans_dart_hmb, bool, 0444);
MODULE_PARM_DESC(apple_ans_dart_hmb,
	"Apple ANS: if the loader provides no HMB, use a DART-mapped host buffer (the iOS fallback)");

struct apple_ans_nvme {
	dma_addr_t hmb_dma;
	u32 hmb_size;
	void *hmb_buf;		/* DART-mapped HMB, NULL for the loader's */
	struct apple_ans_nvme_req req[APPLE_ANS_NVMMU_MAX_REQS];
	DECLARE_BITMAP(used_req, APPLE_ANS_NVMMU_MAX_REQS);
	unsigned int last_req;
	/* Protects the FlatDMA request-slot bitmap. */
	spinlock_t req_lock;
};

static struct apple_ans_nvme *nvme_pci_apple_ans(struct nvme_dev *dev)
{
	return dev->dma_data;
}

static struct device_node *nvme_pci_apple_ans_host_np(struct pci_dev *pdev)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);

	if (!bridge)
		return NULL;
	if (bridge->dev.parent && bridge->dev.parent->of_node)
		return bridge->dev.parent->of_node;
	return bridge->dev.of_node;
}

static const struct of_device_id nvme_pci_apple_ans_hosts[] = {
	{ .compatible = "apple,t8010-pcie",
	  .data = (void *)APPLE_ANS_L1SS_CTL1_T8010 },
	{ .compatible = "apple,s8000-pcie",
	  .data = (void *)APPLE_ANS_L1SS_CTL1_S8000 },
	{ .compatible = "apple,s8003-pcie",
	  .data = (void *)APPLE_ANS_L1SS_CTL1_S8000 },
	{ }
};

/*
 * The S3E behind an H8/H9 PCIe root complex needs the ANS glue. A10 reports
 * 106b:2002; the A9 ID is unverified, so match on the host bridge instead.
 */
bool nvme_pci_apple_ans_match(struct pci_dev *pdev)
{
	struct device_node *np;

	if (pdev->vendor != PCI_VENDOR_ID_APPLE)
		return false;

	np = nvme_pci_apple_ans_host_np(pdev);
	return np && of_match_node(nvme_pci_apple_ans_hosts, np);
}

static int nvme_pci_apple_ans_init(struct nvme_dev *dev, int node)
{
	struct apple_ans_nvme *ans;

	ans = kzalloc_node(sizeof(*ans), GFP_KERNEL, node);
	if (!ans)
		return -ENOMEM;

	spin_lock_init(&ans->req_lock);
	dev->dma_data = ans;
	return 0;
}

static void nvme_pci_apple_ans_exit(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);

	/* The controller is disabled by now, so it can't touch the HMB */
	if (ans && ans->hmb_buf)
		dma_free_coherent(dev->dev, ans->hmb_size, ans->hmb_buf,
				  ans->hmb_dma);
	kfree(dev->dma_data);
	dev->dma_data = NULL;
}

static int nvme_pci_apple_ans_find_hmb(struct nvme_dev *dev, u32 size_req,
				       u32 align_req)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	struct pci_host_bridge *bridge;
	struct device_node *pcie_np;
	struct device_node *mem_np;
	struct resource res;
	resource_size_t size;
	u32 iova;
	int ret;

	bridge = pci_find_host_bridge(pdev->bus);
	if (!bridge)
		return -ENODEV;

	pcie_np = bridge->dev.parent ? bridge->dev.parent->of_node : NULL;
	if (!pcie_np)
		pcie_np = bridge->dev.of_node;
	if (!pcie_np)
		return -ENODEV;

	mem_np = of_parse_phandle(pcie_np, "apple,nvme-hmb", 0);
	if (!mem_np)
		return -ENODEV;

	ret = of_address_to_resource(mem_np, 0, &res);
	if (ret)
		goto out_put_mem;
	if (!res.start || !resource_size(&res)) {
		ret = -ENODEV;
		goto out_put_mem;
	}

	ret = of_property_read_u32(pcie_np, "apple,nvmmu-iova", &iova);
	if (ret)
		goto out_put_mem;

	if (!size_req || size_req == U32_MAX) {
		ret = -EINVAL;
		goto out_put_mem;
	}
	if (!align_req || align_req == U32_MAX)
		align_req = 1;

	size = resource_size(&res);
	if (size < size_req || size > U32_MAX) {
		ret = -ENOSPC;
		goto out_put_mem;
	}
	if (!IS_ALIGNED(res.start, align_req) || !IS_ALIGNED(iova, align_req)) {
		ret = -EINVAL;
		goto out_put_mem;
	}

	ans->hmb_dma = iova;
	ans->hmb_size = size;
	dev_info(dev->dev, "Apple ANS: HMB %#x@%pa as DMA %#llx\n",
		ans->hmb_size, &res.start, (u64)ans->hmb_dma);

out_put_mem:
	of_node_put(mem_np);
	return ret;
}

/*
 * iOS (IONVMeFamily) takes the HMB ("scratch buffer") from
 * nvme-scratch-virt-region, else nvme-scratch-region, else allocates a
 * buffer of the requested size and maps it through the DART. This is the
 * last case: the buffer lives in the DART window, the NVMMU SART isn't used.
 */
static int nvme_pci_apple_ans_alloc_dart_hmb(struct nvme_dev *dev,
					     u32 size_req, u32 align_req)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	dma_addr_t dma;
	void *buf;

	if (!size_req || size_req == U32_MAX)
		return -EINVAL;
	if (!align_req || align_req == U32_MAX)
		align_req = 1;
	if (!is_power_of_2(align_req))
		return -EINVAL;

	buf = dma_alloc_coherent(dev->dev, size_req, &dma, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (!IS_ALIGNED(dma, align_req)) {
		dma_free_coherent(dev->dev, size_req, buf, dma);
		return -EINVAL;
	}

	ans->hmb_buf = buf;
	ans->hmb_dma = dma;
	ans->hmb_size = size_req;
	dev_info(dev->dev, "Apple ANS: HMB %#x at DART DMA %pad\n",
		 ans->hmb_size, &ans->hmb_dma);
	return 0;
}

static int nvme_pci_apple_ans_reset_dma(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	unsigned long flags;
	unsigned int tag;
	int ret;

	for (tag = 0; tag < APPLE_ANS_NVMMU_MAX_REQS; tag++) {
		ret = apple_ans_nvmmu_map(dev->dev, tag, NULL, 0, false, NULL);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&ans->req_lock, flags);
	bitmap_zero(ans->used_req, APPLE_ANS_NVMMU_MAX_REQS);
	ans->last_req = 0;
	spin_unlock_irqrestore(&ans->req_lock, flags);
	return 0;
}

static void nvme_pci_apple_ans_config(struct nvme_dev *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	const struct of_device_id *match;
	u16 ltr, l1ss;
	u32 l1ss_ctl1;

	match = of_match_node(nvme_pci_apple_ans_hosts,
			      nvme_pci_apple_ans_host_np(pdev));
	l1ss_ctl1 = match ? (u32)(uintptr_t)match->data :
			    APPLE_ANS_L1SS_CTL1_T8010;

	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0x40);

	ltr = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_LTR);
	l1ss = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
	dev_info(dev->dev, "Apple ANS: LTR cap at %#x, L1SS cap at %#x\n",
		 ltr, l1ss);

	if (ltr)
		pci_write_config_dword(pdev, ltr + PCI_LTR_MAX_SNOOP_LAT,
				       APPLE_ANS_LTR_MAX_LATENCY);
	if (l1ss) {
		pci_write_config_dword(pdev, l1ss + PCI_L1SS_CTL2, 0);
		pci_write_config_dword(pdev, l1ss + PCI_L1SS_CTL1, l1ss_ctl1);
	}
}

static int nvme_pci_apple_ans_preinit(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	u32 csts, core_mask, log_size;
	u32 hmb_size, hmb_align;
	int ret;

	if (!ans)
		return 0;

	core_mask = readl(dev->bar + APPLE_ANS_REG_CORE_MASK);
	log_size = readl(dev->bar + APPLE_ANS_REG_LOG_SIZE);
	hmb_size = readl(dev->bar + APPLE_ANS_REG_HMB_SIZE_REQ);
	hmb_align = readl(dev->bar + APPLE_ANS_REG_HMB_ALIGN_REQ);
	dev_info(dev->dev,
		 "Apple ANS: core_mask=%#x log_size=%#x hmb_req=%#x align=%#x\n",
		 core_mask, log_size, hmb_size, hmb_align);

	/*
	 * Check everything this needs from the loader and the host bridge
	 * before writing anything to the controller.
	 */
	if (!ans->hmb_size) {
		ret = nvme_pci_apple_ans_find_hmb(dev, hmb_size, hmb_align);
		if (ret && apple_ans_dart_hmb)
			ret = nvme_pci_apple_ans_alloc_dart_hmb(dev, hmb_size,
								hmb_align);
		if (ret)
			return dev_err_probe(dev->dev, ret,
					     "Apple ANS: no usable HMB (needs a loader that fills /reserved-memory/nvme-hmb, or nvme.apple_ans_dart_hmb=1)\n");
	}

	ret = nvme_pci_apple_ans_reset_dma(dev);
	if (ret)
		return dev_err_probe(dev->dev, ret,
				     "Apple ANS: failed to reset NVMMU DMA slots\n");

	nvme_pci_apple_ans_config(dev);

	writel(0, dev->bar + NVME_REG_CC);
	ret = readl_poll_timeout(dev->bar + NVME_REG_CSTS, csts,
				 !(csts & (NVME_CSTS_RDY | NVME_CSTS_CFS)),
				 1000, 2000000);
	if (ret)
		return dev_err_probe(dev->dev, ret,
				     "Apple ANS: controller did not stop (CSTS=%#x)\n",
				     csts);

	return 0;
}

static int nvme_pci_apple_ans_prepare_enable(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);

	if (!ans)
		return 0;
	if (!ans->hmb_size)
		return -EINVAL;

	writel(APPLE_ANS_REG_INIT_REGULAR, dev->bar + APPLE_ANS_REG_INIT);
	writel(lower_32_bits(ans->hmb_dma),
	       dev->bar + APPLE_ANS_REG_HMB_BASE_LO);
	writel(upper_32_bits(ans->hmb_dma),
	       dev->bar + APPLE_ANS_REG_HMB_BASE_HI);
	writel(ans->hmb_size, dev->bar + APPLE_ANS_REG_HMB_SIZE);
	readl(dev->bar + APPLE_ANS_REG_HMB_SIZE);

	return 0;
}

static blk_status_t nvme_pci_apple_ans_alloc_req(struct nvme_dev *dev,
						 struct apple_ans_nvme_req **req,
						 unsigned int *tag)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	unsigned long flags;
	unsigned int idx;

	spin_lock_irqsave(&ans->req_lock, flags);
	idx = find_next_zero_bit(ans->used_req, APPLE_ANS_NVMMU_MAX_REQS,
				 (ans->last_req + 1) %
				 APPLE_ANS_NVMMU_MAX_REQS);
	if (idx >= APPLE_ANS_NVMMU_MAX_REQS)
		idx = find_first_zero_bit(ans->used_req,
					  APPLE_ANS_NVMMU_MAX_REQS);
	if (idx >= APPLE_ANS_NVMMU_MAX_REQS) {
		spin_unlock_irqrestore(&ans->req_lock, flags);
		dev_dbg_ratelimited(dev->dev,
				    "Apple ANS NVMe FlatDMA slots exhausted\n");
		return BLK_STS_RESOURCE;
	}

	ans->last_req = idx;
	__set_bit(idx, ans->used_req);
	spin_unlock_irqrestore(&ans->req_lock, flags);

	*req = &ans->req[idx];
	*tag = idx;
	memset((*req)->pages, 0, sizeof((*req)->pages));
	return BLK_STS_OK;
}

static void nvme_pci_apple_ans_free_req(struct nvme_dev *dev,
					struct apple_ans_nvme_req *req)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	unsigned long flags;
	unsigned int tag;
	int ret;

	if (!ans || !req)
		return;

	tag = req - ans->req;
	if (tag >= APPLE_ANS_NVMMU_MAX_REQS)
		return;

	ret = apple_ans_nvmmu_map(dev->dev, tag, NULL, 0, false, NULL);
	if (ret)
		dev_warn_ratelimited(dev->dev,
				     "failed to release ANS DMA slot %u: %d\n",
				     tag, ret);

	spin_lock_irqsave(&ans->req_lock, flags);
	__clear_bit(tag, ans->used_req);
	spin_unlock_irqrestore(&ans->req_lock, flags);
}

static blk_status_t nvme_pci_apple_ans_map_bvec(struct nvme_dev *dev,
						struct request *req,
						struct nvme_iod *iod,
						struct apple_ans_nvme_req *areq,
						const struct bio_vec *bv,
						unsigned int total,
						unsigned int *npages,
						unsigned int *consumed,
						u64 *offs)
{
	dma_addr_t dma_addr;
	u64 phys;
	unsigned int len = bv->bv_len;

	if (!len)
		return BLK_STS_OK;

	if (WARN_ON_ONCE(iod->nr_dma_vecs >= dev->ctrl.max_segments))
		return BLK_STS_IOERR;

	dma_addr = dma_map_bvec(dev->dev, bv, rq_dma_dir(req), 0);
	if (dma_mapping_error(dev->dev, dma_addr))
		return BLK_STS_RESOURCE;

	iod->dma_vecs[iod->nr_dma_vecs].addr = dma_addr;
	iod->dma_vecs[iod->nr_dma_vecs].len = len;
	iod->nr_dma_vecs++;

	phys = page_to_phys(bv->bv_page) + bv->bv_offset;
	if (!*consumed) {
		*offs = phys & (APPLE_ANS_NVMMU_PAGE_SIZE - 1);
		phys -= *offs;
		len += *offs;
	} else if (phys & (APPLE_ANS_NVMMU_PAGE_SIZE - 1)) {
		dev_err_ratelimited(dev->dev,
				    "Apple ANS FlatDMA segment is not page-aligned: phys=%#llx\n",
				    phys);
		return BLK_STS_IOERR;
	}

	if (*consumed + bv->bv_len != total &&
	    (len & (APPLE_ANS_NVMMU_PAGE_SIZE - 1))) {
		dev_err_ratelimited(dev->dev,
				    "Apple ANS FlatDMA segment length is not page-aligned: len=%#x\n",
				    len);
		return BLK_STS_IOERR;
	}

	while (len) {
		if (*npages >= APPLE_ANS_NVMMU_MAX_PAGES)
			return BLK_STS_IOERR;

		areq->pages[(*npages)++] = phys;
		phys += APPLE_ANS_NVMMU_PAGE_SIZE;
		len = len > APPLE_ANS_NVMMU_PAGE_SIZE ?
			len - APPLE_ANS_NVMMU_PAGE_SIZE : 0;
	}

	*consumed += bv->bv_len;
	return BLK_STS_OK;
}

static bool nvme_pci_apple_ans_unmap_data(struct request *req)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
	struct nvme_dev *dev = nvmeq->dev;
	unsigned int i;

	if (!iod->dma_private)
		return false;

	for (i = 0; i < iod->nr_dma_vecs; i++)
		dma_unmap_page(dev->dev, iod->dma_vecs[i].addr,
			       iod->dma_vecs[i].len, rq_dma_dir(req));
	if (iod->dma_vecs) {
		mempool_free(iod->dma_vecs, dev->dmavec_mempool);
		iod->dma_vecs = NULL;
	}
	iod->nr_dma_vecs = 0;

	nvme_pci_apple_ans_free_req(dev, iod->dma_private);
	iod->dma_private = NULL;
	iod->cmd.common.flags &= ~NVME_CMD_APPLE_ANS_FLATDMA;
	return true;
}

static blk_status_t nvme_pci_apple_ans_map_data(struct request *req)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
	struct nvme_dev *dev = nvmeq->dev;
	struct apple_ans_nvme_req *areq;
	struct req_iterator iter;
	struct bio_vec bv;
	dma_addr_t flatdma;
	u64 offs = 0;
	unsigned int tag, npages = 0, consumed = 0;
	unsigned int total = blk_rq_payload_bytes(req);
	blk_status_t status;
	int ret;

	if (!nvme_pci_apple_ans(dev))
		return BLK_STS_NOTSUPP;
	if (total > APPLE_ANS_NVMMU_MAX_PAGES * APPLE_ANS_NVMMU_PAGE_SIZE)
		return BLK_STS_IOERR;

	status = nvme_pci_apple_ans_alloc_req(dev, &areq, &tag);
	if (status)
		return status;

	iod->dma_private = areq;
	iod->dma_vecs = mempool_alloc(dev->dmavec_mempool, GFP_ATOMIC);
	if (!iod->dma_vecs) {
		status = BLK_STS_RESOURCE;
		goto out_unmap;
	}

	if (req->rq_flags & RQF_SPECIAL_PAYLOAD) {
		bv = req_bvec(req);
		status = nvme_pci_apple_ans_map_bvec(dev, req, iod, areq, &bv,
						     total, &npages, &consumed,
						     &offs);
		if (status)
			goto out_unmap;
	} else {
		rq_for_each_bvec(bv, req, iter) {
			status = nvme_pci_apple_ans_map_bvec(dev, req, iod,
							     areq, &bv, total,
							     &npages, &consumed,
							     &offs);
			if (status)
				goto out_unmap;
		}
	}

	if (consumed != total) {
		dev_err_ratelimited(dev->dev,
				    "Apple ANS FlatDMA payload length mismatch: consumed=%#x total=%#x\n",
				    consumed, total);
		status = BLK_STS_IOERR;
		goto out_unmap;
	}

	ret = apple_ans_nvmmu_map(dev->dev, tag, areq->pages, npages,
				  rq_dma_dir(req) == DMA_FROM_DEVICE, &flatdma);
	if (ret) {
		status = errno_to_blk_status(ret);
		if (status == BLK_STS_NOTSUPP)
			status = BLK_STS_IOERR;
		goto out_unmap;
	}

	iod->total_len = total;
	iod->cmd.common.flags |= NVME_CMD_APPLE_ANS_FLATDMA;
	iod->cmd.common.dptr.prp1 = cpu_to_le64(flatdma + offs);
	iod->cmd.common.dptr.prp2 = 0;
	return BLK_STS_OK;

out_unmap:
	nvme_pci_apple_ans_unmap_data(req);
	return status;
}

static u32 nvme_pci_apple_ans_queue_depth(struct nvme_dev *dev, u32 depth)
{
	return min_t(u32, depth, APPLE_ANS_NVMMU_MAX_REQS);
}

static u32 nvme_pci_apple_ans_max_hw_sectors(struct nvme_dev *dev,
					     u32 max_hw_sectors)
{
	return min_t(u32, max_hw_sectors, APPLE_ANS_NVME_MAX_SECTORS);
}

const struct nvme_pci_dma_ops nvme_pci_apple_ans_ops = {
	.init		= nvme_pci_apple_ans_init,
	.exit		= nvme_pci_apple_ans_exit,
	.preinit	= nvme_pci_apple_ans_preinit,
	.prepare_enable = nvme_pci_apple_ans_prepare_enable,
	.map_data	= nvme_pci_apple_ans_map_data,
	.unmap_data	= nvme_pci_apple_ans_unmap_data,
	.queue_depth	= nvme_pci_apple_ans_queue_depth,
	.max_hw_sectors = nvme_pci_apple_ans_max_hw_sectors,
};

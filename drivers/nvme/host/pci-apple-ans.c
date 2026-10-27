// SPDX-License-Identifier: GPL-2.0
/*
 * Apple ANS PCI NVMe glue for H8 and H9 SoCs.
 *
 * Copyright (C) 2020 Corellium LLC
 * Copyright (C) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/apple-ans.h>
#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/iommu.h>
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
 * iOS reads this first in InitializeController: 0xbfbfbfbf means the
 * controller waits for a boot-from-host firmware upload (0x1b00-0x1b14),
 * which Linux doesn't do.
 */
#define APPLE_ANS_REG_BFH		0x1b18
#define APPLE_ANS_BFH_MAGIC		0xbfbfbfbf
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
	u32 nsid;
	u16 qid;
	u16 cid;
	u8 opcode;
	bool mapped;
};

/*
 * NSID 1 is the data namespace (iOS disk0: the APFS container). The others
 * hold factory and boot data a DFU restore can't bring back. On j71t:
 * 2 LLB/iBoot, 3 syscfg, 4 iPCB, 5 NVRAM, 6 effaceable storage, 7 unused.
 */
#define APPLE_ANS_DATA_NSID	1

static bool apple_ans_unlock_system_namespaces;
module_param(apple_ans_unlock_system_namespaces, bool, 0444);
MODULE_PARM_DESC(apple_ans_unlock_system_namespaces,
	"Apple ANS: DANGEROUS. Allow commands that can change namespaces other than NSID 1 (LLB/iBoot, syscfg, NVRAM, effaceable storage) or the whole controller (format/sanitize all, firmware, namespace management, security, vendor commands). Default N; boot-time only.");

static bool apple_ans_dart_hmb;
module_param(apple_ans_dart_hmb, bool, 0444);
MODULE_PARM_DESC(apple_ans_dart_hmb,
	"Apple ANS: if the loader provides no HMB, use a DART-mapped host buffer (the iOS fallback)");

/*
 * Bring-up debugging knobs. Enabling the controller (CC.EN) has frozen
 * the whole SoC, so the enable can be stopped short, held until userspace
 * says go, slowed down so each step reaches the screen, and is followed by
 * a watch of SoC-side state only.
 */
static bool apple_ans_probe = !IS_ENABLED(CONFIG_NVME_APPLE_ANS_EARLY_USB);
module_param(apple_ans_probe, bool, 0644);
MODULE_PARM_DESC(apple_ans_probe,
	"j71t: allow PCI ANS probe (boot initramfs releases after USB logging is ready)");

bool nvme_pci_apple_ans_probe_allowed(struct pci_dev *pdev)
{
	return !of_machine_is_compatible("apple,j71t") ||
	       READ_ONCE(apple_ans_probe) || !nvme_pci_apple_ans_match(pdev);
}

static unsigned int apple_ans_enable = 1;
module_param(apple_ans_enable, uint, 0644);
MODULE_PARM_DESC(apple_ans_enable,
	"Apple ANS: 1 = enable the controller, 0 = stop right before CC.EN (dry run)");

static unsigned int apple_ans_hold;
module_param(apple_ans_hold, uint, 0644);
MODULE_PARM_DESC(apple_ans_hold,
	"Apple ANS: wait up to this many seconds before CC.EN for 'echo 1 > /sys/module/nvme/parameters/apple_ans_go' (0 = no wait); on timeout the controller stays disabled");

static unsigned int apple_ans_slow_ms;
module_param(apple_ans_slow_ms, uint, 0644);
MODULE_PARM_DESC(apple_ans_slow_ms,
	"Apple ANS: sleep this long after each bring-up message, so it reaches the (deferred) framebuffer console before the next step");

/* A j71t diagnostic boot must reach rescue before the 120 s boot deadline. */
static unsigned int apple_ans_rdy_timeout = 5;
module_param(apple_ans_rdy_timeout, uint, 0444);
MODULE_PARM_DESC(apple_ans_rdy_timeout,
		 "j71t: RDY transition deadline in seconds (clamped to 1..30)");

static unsigned int apple_ans_watch_ms = 100;
module_param(apple_ans_watch_ms, uint, 0644);
MODULE_PARM_DESC(apple_ans_watch_ms,
	"Apple ANS: after CC.EN, watch only SoC-side state (link, NVMMU, DART) this long before the first CSTS read");

static DECLARE_COMPLETION(apple_ans_go_done);

static int apple_ans_go_set(const char *val, const struct kernel_param *kp)
{
	bool go;
	int ret = kstrtobool(val, &go);

	if (ret)
		return ret;
	if (go)
		complete_all(&apple_ans_go_done);
	return 0;
}

static int apple_ans_go_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%c\n",
			  completion_done(&apple_ans_go_done) ? 'Y' : 'N');
}

static const struct kernel_param_ops apple_ans_go_ops = {
	.set = apple_ans_go_set,
	.get = apple_ans_go_get,
};
module_param_cb(apple_ans_go, &apple_ans_go_ops, NULL, 0644);
MODULE_PARM_DESC(apple_ans_go, "Apple ANS: write 1 to release apple_ans_hold");

static bool apple_ans_debug;
module_param(apple_ans_debug, bool, 0644);
MODULE_PARM_DESC(apple_ans_debug,
	"Apple ANS: log bring-up diagnostics (MSI setup, SoC-side host/NVMMU/DART dumps, DART table walks for the admin queues, SoC-side changes after CC.EN). Fault diagnostics are always logged.");

/* Bring-up detail: dynamic debug, or always with nvme.apple_ans_debug=1 */
#define ans_dbg(dev, fmt, ...)						\
	do {								\
		if (apple_ans_debug)					\
			dev_info(dev, fmt, ##__VA_ARGS__);		\
		else							\
			dev_dbg(dev, fmt, ##__VA_ARGS__);		\
	} while (0)

static void apple_ans_settle(void)
{
	if (apple_ans_slow_ms)
		msleep(apple_ans_slow_ms);
}

struct apple_ans_nvme {
	dma_addr_t hmb_dma;
	u32 hmb_size;
	void *hmb_buf;		/* DART-mapped HMB, NULL for the loader's */
	struct apple_ans_nvme_req req[APPLE_ANS_NVMMU_MAX_REQS];
	DECLARE_BITMAP(used_req, APPLE_ANS_NVMMU_MAX_REQS);
	unsigned int last_req;
	/* Protects the FlatDMA request-slot bitmap. */
	spinlock_t req_lock;
	/* SoC-side state at CC.EN, for spotting faults while polling CSTS */
	struct apple_ans_host_state cc_base;
	bool cc_base_valid;
	bool host_fault;	/* stop touching the controller */
	bool dart_fault_logged;
	ktime_t cc_written_at;
	u32 last_cc;
	bool cc_written;
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

/*
 * System-namespace guard, on top of (and independent of) the
 * CONFIG_NVME_PCI_READ_ONLY allowlist. Called for every command at
 * submission and again when it is copied into the SQ (under the SQ lock).
 */
static bool nvme_pci_apple_ans_cmd_allowed(struct nvme_dev *dev, u16 qid,
					   const struct nvme_command *cmd)
{
	u32 nsid = le32_to_cpu(cmd->common.nsid);
	u8 opcode = cmd->common.opcode;
	bool ok;

	if (apple_ans_unlock_system_namespaces)
		return true;

	if (qid) {
		/* I/O: anything on NSID 1, only reads elsewhere */
		ok = nsid == APPLE_ANS_DATA_NSID || opcode == nvme_cmd_read ||
		     opcode == nvme_cmd_flush;
	} else {
		switch (opcode) {
		case nvme_admin_delete_sq:
		case nvme_admin_create_sq:
		case nvme_admin_get_log_page:
		case nvme_admin_delete_cq:
		case nvme_admin_create_cq:
		case nvme_admin_identify:
		case nvme_admin_abort_cmd:
		case nvme_admin_get_features:
		case nvme_admin_async_event:
		case nvme_admin_keep_alive:
			ok = true;
			break;
		case nvme_admin_set_features:
			/* controller-wide or the data namespace only */
			ok = nsid == 0 || nsid == APPLE_ANS_DATA_NSID;
			break;
		case nvme_admin_format_nvm:
			ok = nsid == APPLE_ANS_DATA_NSID;
			break;
		default:
			/*
			 * Sanitize, firmware download/commit, namespace
			 * management/attachment, security, directives,
			 * self-test and vendor-specific (Apple NAND tunnel)
			 * commands can reach every namespace.
			 */
			ok = false;
			break;
		}
	}

	if (!ok)
		dev_err_ratelimited(dev->dev,
				    "Apple ANS: refused opcode %#x for nsid %u on qid %u: system namespaces are protected\n",
				    opcode, nsid, qid);
	return ok;
}

static bool nvme_pci_apple_ans_ns_readonly(struct nvme_dev *dev, u32 nsid)
{
	return !apple_ans_unlock_system_namespaces &&
	       nsid != APPLE_ANS_DATA_NSID;
}

static int nvme_pci_apple_ans_init(struct nvme_dev *dev, int node)
{
	struct apple_ans_nvme *ans;

	ans = kzalloc_node(sizeof(*ans), GFP_KERNEL, node);
	if (!ans)
		return -ENOMEM;

	spin_lock_init(&ans->req_lock);
	dev->dma_data = ans;

	if (apple_ans_unlock_system_namespaces)
		dev_warn(dev->dev,
			 "Apple ANS: system namespaces UNLOCKED (nvme.apple_ans_unlock_system_namespaces=1)\n");
	else
		dev_info(dev->dev,
			 "Apple ANS: namespaces other than NSID %u are read-only and only safe admin commands are sent\n",
			 APPLE_ANS_DATA_NSID);
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
	int ret;

	ret = apple_ans_nvmmu_reset(dev->dev);
	if (ret)
		return ret;

	/* The hardware init invalidated every tag; do not flood the FIFO. */
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
	ans_dbg(dev->dev, "Apple ANS: LTR cap at %#x, L1SS cap at %#x\n",
		 ltr, l1ss);

	if (ltr)
		pci_write_config_dword(pdev, ltr + PCI_LTR_MAX_SNOOP_LAT,
				       APPLE_ANS_LTR_MAX_LATENCY);
	if (l1ss) {
		pci_write_config_dword(pdev, l1ss + PCI_L1SS_CTL2, 0);
		pci_write_config_dword(pdev, l1ss + PCI_L1SS_CTL1, l1ss_ctl1);
	}
}

/*
 * A BAR access over a dead link can stall the CPU for good on the H8P/H9P
 * root complex, so check the link before each step that touches the
 * controller. The phase messages say where a hang happened.
 */
static int nvme_pci_apple_ans_check_link(struct nvme_dev *dev,
					 const char *step)
{
	int up = apple_ans_link_up(dev->dev);

	if (up > 0)
		return 0;
	dev_err(dev->dev, "Apple ANS: PCIe link down before %s, leaving the controller alone\n",
		step);
	return up < 0 ? up : -ENOLINK;
}

static int nvme_pci_apple_ans_preinit(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	u32 csts, core_mask, log_size;
	u32 hmb_size, hmb_align, bfh;
	int ret;

	if (!ans)
		return 0;

	ret = nvme_pci_apple_ans_check_link(dev, "preinit");
	if (ret)
		return ret;
	core_mask = readl(dev->bar + APPLE_ANS_REG_CORE_MASK);
	log_size = readl(dev->bar + APPLE_ANS_REG_LOG_SIZE);
	hmb_size = readl(dev->bar + APPLE_ANS_REG_HMB_SIZE_REQ);
	hmb_align = readl(dev->bar + APPLE_ANS_REG_HMB_ALIGN_REQ);
	ans_dbg(dev->dev,
		 "Apple ANS: core_mask=%#x log_size=%#x hmb_req=%#x align=%#x\n",
		 core_mask, log_size, hmb_size, hmb_align);
	bfh = readl(dev->bar + APPLE_ANS_REG_BFH);
	dev_info(dev->dev, "Apple ANS: CAP=%#llx VS=%#x CC=%#x CSTS=%#x BFH=%#x\n",
		 lo_hi_readq(dev->bar + NVME_REG_CAP),
		 readl(dev->bar + NVME_REG_VS), readl(dev->bar + NVME_REG_CC),
		 readl(dev->bar + NVME_REG_CSTS), bfh);
	apple_ans_settle();
	if (bfh == APPLE_ANS_BFH_MAGIC)
		return dev_err_probe(dev->dev, -ENODEV,
				     "Apple ANS: controller waits for a boot-from-host firmware upload, not supported\n");

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

	ret = nvme_pci_apple_ans_check_link(dev, "PCI config setup");
	if (ret)
		return ret;
	nvme_pci_apple_ans_config(dev);

	ret = nvme_pci_apple_ans_check_link(dev, "CC=0");
	if (ret)
		return ret;
	ans_dbg(dev->dev, "Apple ANS: stopping controller (CC=0)\n");
	writel(0, dev->bar + NVME_REG_CC);
	ret = readl_poll_timeout(dev->bar + NVME_REG_CSTS, csts,
				 !(csts & (NVME_CSTS_RDY | NVME_CSTS_CFS)),
				 1000, 2000000);
	if (ret) {
		ans->host_fault = true;
		apple_ans_host_dump(dev->dev, "controller stop failed");
		return dev_err_probe(dev->dev, ret,
				     "Apple ANS: controller did not stop (CSTS=%#x)\n",
				     csts);
	}
	dev_info(dev->dev, "Apple ANS: controller stopped (CSTS=%#x), rebuilding DMA\n", csts);
	if (apple_ans_boot_mode(dev->dev) != APPLE_ANS_ROUND7) {
		ret = apple_ans_dart_set_active(dev->dev, true);
		if (ret)
			return ret;
	}
	ret = nvme_pci_apple_ans_reset_dma(dev);
	if (ret)
		return dev_err_probe(dev->dev, ret,
				     "Apple ANS: failed to rebuild stopped NVMMU\n");
	ans->cc_base_valid = false;
	ans->cc_written = false;
	apple_ans_settle();

	return 0;
}

/* The S3E's MSI/MSI-X setup as the controller will use it */
void nvme_pci_apple_ans_log_msi(struct nvme_dev *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	u16 flags = 0, data = 0, xflags = 0;
	u32 lo = 0, hi = 0;
	int msi, msix;

	msi = pdev->msi_cap;
	if (msi) {
		pci_read_config_word(pdev, msi + PCI_MSI_FLAGS, &flags);
		pci_read_config_dword(pdev, msi + PCI_MSI_ADDRESS_LO, &lo);
		if (flags & PCI_MSI_FLAGS_64BIT) {
			pci_read_config_dword(pdev, msi + PCI_MSI_ADDRESS_HI, &hi);
			pci_read_config_word(pdev, msi + PCI_MSI_DATA_64, &data);
		} else {
			pci_read_config_word(pdev, msi + PCI_MSI_DATA_32, &data);
		}
	}
	msix = pdev->msix_cap;
	if (msix)
		pci_read_config_word(pdev, msix + PCI_MSIX_FLAGS, &xflags);
	dev_info(dev->dev,
		 "Apple ANS: MSI cap %#x flags %#x addr %#x:%08x data %#x; MSI-X cap %#x flags %#x; irq %d\n",
		 msi, flags, hi, lo, data, msix, xflags, pci_irq_vector(pdev, 0));
}

/* Linux's own mapping of a queue IOVA, then the DART table walk for it */
static void nvme_pci_apple_ans_log_queue(struct nvme_dev *dev, const char *what,
					 dma_addr_t iova)
{
	struct iommu_domain *dom = iommu_get_domain_for_dev(dev->dev);
	phys_addr_t pa = dom ? iommu_iova_to_phys(dom, iova) : 0;

	ans_dbg(dev->dev, "Apple ANS: %s IOVA %pad -> PA %pa (Linux view)\n",
		 what, &iova, &pa);
	apple_ans_dart_walk(dev->dev, what, iova);
}

/*
 * Stop short or wait for userspace before CC.EN. Runs in reset_work with
 * shutdown_lock held, so a reboot during the hold waits for it to end.
 */
static int nvme_pci_apple_ans_gate(struct nvme_dev *dev)
{
	long left;

	if (!apple_ans_enable) {
		dev_warn(dev->dev,
			 "Apple ANS: dry run (nvme.apple_ans_enable=0): everything is programmed, CC.EN is not set\n");
		return -ENODEV;
	}
	if (!apple_ans_hold || completion_done(&apple_ans_go_done))
		return 0;

	dev_warn(dev->dev,
		 "Apple ANS: holding before CC.EN for up to %us: echo 1 > /sys/module/nvme/parameters/apple_ans_go\n",
		 apple_ans_hold);
	left = wait_for_completion_interruptible_timeout(&apple_ans_go_done,
						apple_ans_hold * HZ);
	if (left <= 0) {
		dev_warn(dev->dev,
			 "Apple ANS: hold ended without go (%ld), controller left disabled\n",
			 left);
		return left ? left : -ETIMEDOUT;
	}
	dev_info(dev->dev, "Apple ANS: go\n");
	apple_ans_settle();
	return 0;
}

static int nvme_pci_apple_ans_prepare_enable(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	int ret;

	if (!ans)
		return 0;
	if (!ans->hmb_size)
		return -EINVAL;
	if (nvme_pci_apple_ans_check_link(dev, "HMB setup"))
		return -ENOLINK;

	writel(APPLE_ANS_REG_INIT_REGULAR, dev->bar + APPLE_ANS_REG_INIT);
	writel(lower_32_bits(ans->hmb_dma),
	       dev->bar + APPLE_ANS_REG_HMB_BASE_LO);
	writel(upper_32_bits(ans->hmb_dma),
	       dev->bar + APPLE_ANS_REG_HMB_BASE_HI);
	writel(ans->hmb_size, dev->bar + APPLE_ANS_REG_HMB_SIZE);
	readl(dev->bar + APPLE_ANS_REG_HMB_SIZE);

	ans_dbg(dev->dev,
		 "Apple ANS: HMB DMA %#llx size %#x programmed; AQA %#x ASQ %#llx ACQ %#llx\n",
		 (u64)ans->hmb_dma, ans->hmb_size,
		 readl(dev->bar + NVME_REG_AQA),
		 lo_hi_readq(dev->bar + NVME_REG_ASQ),
		 lo_hi_readq(dev->bar + NVME_REG_ACQ));
	if (apple_ans_debug) {
		nvme_pci_apple_ans_log_msi(dev);
		apple_ans_host_dump(dev->dev, "before CC.EN");
		nvme_pci_apple_ans_log_queue(dev, "ASQ",
					     dev->queues[0].sq_dma_addr);
		nvme_pci_apple_ans_log_queue(dev, "ACQ",
					     dev->queues[0].cq_dma_addr);
	}
	apple_ans_settle();

	ret = nvme_pci_apple_ans_gate(dev);
	if (ret)
		return ret;
	if (nvme_pci_apple_ans_check_link(dev, "CC.EN"))
		return -ENOLINK;

	ans->host_fault = false;
	ans->dart_fault_logged = false;
	dev_info(dev->dev,
		 "Apple ANS: enabling controller (RDY timeout %llus, SoC-side watch %ums)\n",
		 (NVME_CAP_TIMEOUT(dev->ctrl.cap) + 1) / 2, apple_ans_watch_ms);
	apple_ans_settle();
	return 0;
}

/*
 * An NVMMU error, or a link that stays down. The link can drop for a
 * moment while it retrains, so re-sample for ~3 ms (@st is updated) before
 * calling that a fault; a CSTS read over a dead link can stall the CPU.
 */
static bool nvme_pci_apple_ans_new_fault(struct nvme_dev *dev,
					 struct apple_ans_host_state *st)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	int i;

	if (st->nvmmu_err && st->nvmmu_err != ans->cc_base.nvmmu_err)
		return true;
	for (i = 0; i < 10 && !st->link_up; i++) {
		usleep_range(200, 400);
		apple_ans_host_state(dev->dev, st);
	}
	return !st->link_up;
}

static bool nvme_pci_apple_ans_new_dart_fault(const struct apple_ans_host_state *st)
{
	/* Without valid, ERROR/ADDR are a live transaction record. In
	 * particular 0x500 only records write direction, even if ADDR changed.
	 * An IRQ-acked fault cannot be reconstructed from the address alone.
	 */
	return st->dart_err & BIT(31);
}

/*
 * CC writes. For CC.EN, write it, then watch only SoC-side state for a
 * while: a stuck inbound path shows up there without a non-posted CSTS
 * read that could stall the CPU. A dead link or an NVMMU fault (iOS
 * panics on those) ends the bring-up before the controller is read.
 */
static int nvme_pci_apple_ans_write_cc(struct nvme_dev *dev, u32 cc)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	struct apple_ans_host_state prev, st;
	ktime_t start;
	s64 us;

	if (ans) {
		if (ans->host_fault ||
		    nvme_pci_apple_ans_check_link(dev, "CC write"))
			return -EIO;
		ans->last_cc = cc;
		ans->cc_written_at = ktime_get();
		ans->cc_written = true;
	}
	if (!ans || !(cc & NVME_CC_ENABLE) || (cc & NVME_CC_SHN_MASK)) {
		writel(cc, dev->bar + NVME_REG_CC);
		return 0;
	}

	ans->cc_base_valid = !apple_ans_host_state(dev->dev, &ans->cc_base);
	writel(cc, dev->bar + NVME_REG_CC);
	start = ktime_get();
	if (!ans->cc_base_valid)
		return 0;

	prev = ans->cc_base;
	do {
		usleep_range(200, 400);
		apple_ans_host_state(dev->dev, &st);
		us = ktime_us_delta(ktime_get(), start);
		if (memcmp(&st, &prev, sizeof(st))) {
			ans_dbg(dev->dev,
				 "Apple ANS: CC.EN+%lldus: link %u ltssm %#x port irq %#x nvmmu err %#x dart err %#x addr %#x\n",
				 us, st.link_up, st.ltssm, st.port_irq,
				 st.nvmmu_err, st.dart_err, st.dart_addr);
			prev = st;
		}
		if (nvme_pci_apple_ans_new_fault(dev, &st)) {
			ans->host_fault = true;
			dev_err(dev->dev,
				"Apple ANS: host-side fault %lldus after CC.EN (link %u nvmmu err %#x), giving up on the controller\n",
				us, st.link_up, st.nvmmu_err);
			apple_ans_host_dump(dev->dev, "after fault");
			return -EIO;
		}
	} while (us < apple_ans_watch_ms * USEC_PER_MSEC);

	dev_info(dev->dev,
		 "Apple ANS: %ums after CC.EN, no host-side fault; polling CSTS\n",
		 apple_ans_watch_ms);
	if (apple_ans_debug)
		apple_ans_host_dump(dev->dev, "after watch");
	apple_ans_settle();
	return 0;
}

/* CSTS reads: only while the SoC side still looks sane */
static int nvme_pci_apple_ans_read_csts(struct nvme_dev *dev, u32 *csts)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	struct apple_ans_host_state st;

	if (ans && (ans->host_fault ||
		    nvme_pci_apple_ans_check_link(dev, "CSTS read")))
		return -EIO;
	if (ans && ans->cc_base_valid) {
		if (ans->host_fault)
			return -EIO;
		apple_ans_host_state(dev->dev, &st);
		if (nvme_pci_apple_ans_new_fault(dev, &st)) {
			ans->host_fault = true;
			dev_err(dev->dev,
				"Apple ANS: host-side fault (link %u nvmmu err %#x) while polling CSTS, giving up on the controller\n",
				st.link_up, st.nvmmu_err);
			apple_ans_host_dump(dev->dev, "after fault");
			return -EIO;
		}
		if (!ans->dart_fault_logged &&
		    nvme_pci_apple_ans_new_dart_fault(&st)) {
			ans->dart_fault_logged = true;
			dev_warn(dev->dev,
				 "Apple ANS: DART fault since CC.EN (err %#x addr %#x), still polling\n",
				 st.dart_err, st.dart_addr);
		}
	}

	*csts = readl(dev->bar + NVME_REG_CSTS);
	if (ans && ans->cc_written && of_machine_is_compatible("apple,j71t") &&
	    !(ans->last_cc & NVME_CC_SHN_MASK) &&
	    !!(*csts & NVME_CSTS_RDY) != !!(ans->last_cc & NVME_CC_ENABLE) &&
	    ktime_ms_delta(ktime_get(), ans->cc_written_at) >=
	    clamp_val(apple_ans_rdy_timeout, 1, 30) * MSEC_PER_SEC) {
		dev_err(dev->dev, "Apple ANS: RDY deadline CC %#x CSTS %#x; stopping endpoint access\n",
			ans->last_cc, *csts);
		ans->host_fault = true;
		apple_ans_host_dump(dev->dev, "RDY deadline");
		return -ETIMEDOUT;
	}
	return 0;
}

/* A9 is a PCI S3E endpoint, not the later platform/RTKit ANS controller.
 * Finish the standard controller reset while its old DART/NVMMU mappings
 * still exist. SHN alone leaves EN set and is insufficient for kexec.
 */
static void nvme_pci_apple_ans_shutdown_round7(struct nvme_dev *dev, int shutdown_ret)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	u32 csts = ~0U;
	int ret, err = 0;

	dev_info(dev->dev, "Apple ANS: kexec shutdown: SHN result %d, clearing CC.EN\n", shutdown_ret);
	if (ans->host_fault || nvme_pci_apple_ans_check_link(dev, "kexec CC=0"))
		goto block_dma;
	dev->ctrl.ctrl_config &= ~(NVME_CC_ENABLE | NVME_CC_SHN_MASK);
	writel(dev->ctrl.ctrl_config, dev->bar + NVME_REG_CC);
	ret = read_poll_timeout(nvme_pci_apple_ans_read_csts, err,
			       err || !(csts & NVME_CSTS_RDY), 1000,
			       2000000, false, dev, &csts);
	if (!ret)
		ret = err;
	if (ret)
		dev_warn(dev->dev, "Apple ANS: CC=0 failed (%d, CSTS %#x); host PERST will isolate the endpoint\n",
			 ret, csts);
	else
		dev_info(dev->dev, "Apple ANS: kexec shutdown: RDY=0 (CSTS %#x)\n", csts);
block_dma:
	/* PCI core does this too; make the ordering explicit before host teardown. */
	pci_clear_master(pdev);
}

static int nvme_pci_apple_ans_stop(struct nvme_dev *dev, int shutdown_ret,
				   bool fatal)
{
	enum apple_ans_boot_mode mode = apple_ans_boot_mode(dev->dev);
	u32 csts = ~0U, cc;
	int ret, err = 0;

	if (fatal && mode == APPLE_ANS_ROUND7) {
		nvme_pci_apple_ans_shutdown_round7(dev, shutdown_ret);
		return 0;
	}
	ret = nvme_pci_apple_ans_read_csts(dev, &csts);
	if (shutdown_ret || ret || (csts & NVME_CSTS_CFS) ||
	    (csts & NVME_CSTS_SHST_MASK) != NVME_CSTS_SHST_CMPLT) {
		pci_clear_master(to_pci_dev(dev->dev));
		apple_ans_host_dump(dev->dev, "SHST failed; refusing kexec");
		if (fatal)
			panic("Apple ANS: SHN failed %d/%d CSTS %#x; refusing unsafe handoff",
			      shutdown_ret, ret, csts);
		dev_err(dev->dev, "Apple ANS: sleep SHN failed %d/%d CSTS %#x\n",
			shutdown_ret, ret, csts);
		return -EBUSY;
	}

	/* SHST completion is not a controller reset: EN/RDY can both stay 1.
	 * The incoming full port sequence removes the link/refclk. Finish the
	 * reset while the old link and DMA translations are still available,
	 * before either mode is allowed to touch PERST or port power.
	 */
	dev_info(dev->dev, "Apple ANS: STEP shutdown SHST complete; clearing CC.EN/SHN\n");
	dev->ctrl.ctrl_config &= ~(NVME_CC_ENABLE | NVME_CC_SHN_MASK);
	ret = nvme_pci_apple_ans_write_cc(dev, dev->ctrl.ctrl_config);
	if (!ret) {
		dev_info(dev->dev, "Apple ANS: STEP shutdown wait-RDY=0 timeout_us=2000000\n");
		ret = read_poll_timeout(nvme_pci_apple_ans_read_csts, err,
					err || !(csts & NVME_CSTS_RDY), 1000,
					2000000, false, dev, &csts);
		if (!ret)
			ret = err;
	}
	if (!ret && (csts & NVME_CSTS_CFS))
		ret = -EIO;
	if (!ret)
		ret = nvme_pci_apple_ans_check_link(dev, "kexec CC readback");
	if (!ret) {
		cc = readl(dev->bar + NVME_REG_CC);
		if (cc & (NVME_CC_ENABLE | NVME_CC_SHN_MASK))
			ret = -EIO;
		if (!fatal && (cc != dev->ctrl.ctrl_config || csts))
			ret = -EIO;
	}
	/* No endpoint MMIO after BME is blocked and translations can retire. */
	pci_clear_master(to_pci_dev(dev->dev));
	if (ret) {
		apple_ans_host_dump(dev->dev, "CC disable failed; refusing kexec");
		if (fatal)
			panic("Apple ANS: CC disable failed %d CSTS %#x; refusing unsafe port reset",
			      ret, csts);
		dev_err(dev->dev, "Apple ANS: sleep CC disable failed %d CSTS %#x\n", ret, csts);
		return -EBUSY;
	}
	dev_info(dev->dev, "Apple ANS: STEP shutdown disabled CC=%#x CSTS=%#x BME=0 mode=%u\n",
		 cc, csts, mode);
	/* Both trial modes hand off a fully down port for cold bring-up. */
	ret = apple_ans_dart_set_active(dev->dev, false);
	if (ret) {
		if (fatal)
			panic("Apple ANS: DART inactive failed %d; refusing kexec", ret);
		dev_err(dev->dev, "Apple ANS: sleep DART inactive failed %d\n", ret);
		return -EBUSY;
	}
	return 0;
}

static void nvme_pci_apple_ans_shutdown(struct nvme_dev *dev, int shutdown_ret)
{
	nvme_pci_apple_ans_stop(dev, shutdown_ret, true);
}

static int nvme_pci_apple_ans_stop_for_sleep(struct nvme_dev *dev, int shutdown_ret)
{
	int ret;

	ret = nvme_pci_apple_ans_stop(dev, shutdown_ret, false);
	if (!ret)
		ret = apple_ans_set_sleep_stopped(dev->dev, true);
	return ret;
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
	memset(*req, 0, sizeof(**req));
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

	/* Allocation/map failures never gave this slot to the NVMMU. */
	if (req->mapped) {
		ret = apple_ans_nvmmu_map(dev->dev, tag, NULL, 0, false, NULL);
		if (ret) {
			/* This API returns -ETIMEDOUT only for an invalidate FIFO
			 * timeout, and -EIO only for its latched poison. Lookup,
			 * ownership and argument errors are not hardware failures.
			 * Keep the slot reserved, but never reset for those errors.
			 */
			bool failed = ret == -ETIMEDOUT || ret == -EIO;

			dev_warn_ratelimited(dev->dev,
					     "ANS DMA slot %u quarantined: %d qid %u cid %#x opcode %#x nsid %u mapped %u; %s\n",
					     tag, ret, req->qid, req->cid, req->opcode,
					     req->nsid, req->mapped,
					     failed ? "confirmed NVMMU failure; scheduling reset" :
					     "local retirement error; no reset");
			if (failed)
				nvme_reset_ctrl(&dev->ctrl);
			return;
		}
		req->mapped = false;
	}

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

	areq->qid = nvmeq->qid;
	areq->cid = le16_to_cpu(iod->cmd.common.command_id);
	areq->opcode = iod->cmd.common.opcode;
	areq->nsid = le32_to_cpu(iod->cmd.common.nsid);
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

	areq->mapped = true;
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
	.shutdown	= nvme_pci_apple_ans_shutdown,
	.stop_for_sleep	= nvme_pci_apple_ans_stop_for_sleep,
	.write_cc	= nvme_pci_apple_ans_write_cc,
	.read_csts	= nvme_pci_apple_ans_read_csts,
	.cmd_allowed	= nvme_pci_apple_ans_cmd_allowed,
	.ns_readonly	= nvme_pci_apple_ans_ns_readonly,
	.map_data	= nvme_pci_apple_ans_map_data,
	.unmap_data	= nvme_pci_apple_ans_unmap_data,
	.queue_depth	= nvme_pci_apple_ans_queue_depth,
	.max_hw_sectors = nvme_pci_apple_ans_max_hw_sectors,
};

/* Old queues are synchronized, BME is blocked; no BAR access until rearmed. */
int nvme_pci_apple_ans_recover(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	int ret = apple_ans_restart_port(dev->dev);

	if (ret)
		return ret;
	pci_restore_state(to_pci_dev(dev->dev));
	ans->host_fault = false;
	ans->cc_base_valid = false;
	ans->cc_written = false;
	ans->dart_fault_logged = false;
	return 0;
}

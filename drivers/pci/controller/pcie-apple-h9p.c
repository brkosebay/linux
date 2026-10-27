// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host bridge driver for Apple H8P/H9P (S8003, T8010) SoCs.
 *
 * The controller exposes an ECAM-compatible root complex after the SoC-specific
 * power, clock and PHY sequence has brought a port out of reset. The hardware
 * differs enough from the Apple Silicon PCIe controller to keep the early H9P
 * bring-up sequence separate, while still using the generic PCI host bridge
 * and MSI subsystems.
 *
 * Copyright (C) 2020 Corellium LLC
 * Copyright (C) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/crash_dump.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irqchip/irq-msi-lib.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/kexec.h>
#include <linux/msi.h>
#include <linux/nvme.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>

#include <linux/apple-ans.h>

#include "../pci.h"
#include "pci-host-common.h"

#define H9P_NUM_PORTS			4
#define H9P_NUM_MSI			32
#define H9P_MSI_PER_PORT		(H9P_NUM_MSI / H9P_NUM_PORTS)

#define H9P_CFG_PORT_STRIDE		0x8000
#define H9P_CFG_PORT_MISC		0x08e0

#define H9P_RC_COMMON_CTL0		0x0004
#define H9P_RC_COMMON_CTL1		0x0014
#define H9P_RC_COMMON_CTL2		0x0024
#define H9P_RC_COMMON_CTL3		0x0034
#define H9P_RC_COMMON_CTL_ENABLE	BIT(0)
#define H9P_RC_COMMON_CTL_INIT	BIT(4)
#define H9P_RC_COMMON_CTL_LANE_CFG	GENMASK(3, 0)
#define H9P_RC_PORT_STRIDE		0x0080
#define H9P_RC_PORTSTAT(port)		(0x0100 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_PORT_CTL0(port)	(0x0100 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_PORT_CTL1(port)	(0x0124 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_PORT_CTL2(port)	(0x0134 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_COMMON_STAT		0x0028
#define H9P_RC_COMMON_STAT_INIT_DONE	BIT(4)
#define H9P_RC_COMMON_STAT_READY	BIT(0)
#define H9P_RC_PORT_LINK_RATE(port)	(0x4020 + (port) * 0x0040)
#define H9P_PHY_PORTMASK		0x000c

#define H9P_PHY_IP_EQ_COMMON0		0x0180
#define H9P_PHY_IP_EQ_COMMON1		0x0184
#define H9P_PHY_IP_EQ_TIME0		0x0090
#define H9P_PHY_IP_EQ_TIME1		0x0098
#define H9P_PHY_IP_PORT_STRIDE		0x0800
#define H9P_PHY_IP_PORT(port, reg)	((reg) + (port) * H9P_PHY_IP_PORT_STRIDE)
#define H9P_PHY_IP_PORT_EQ_CTL		0x10088
#define H9P_PHY_IP_PORT_IDLE		0x10784
#define H9P_PHY_IP_PORT_EQ_PRESET	0x10004
#define H9P_PHY_IP_PORT_RX_CTL0		0x20788
#define H9P_PHY_IP_PORT_RX_CTL1		0x207a0
#define H9P_PHY_IP_PORT_RX_CTL2		0x207a8
#define H9P_PHY_IP_PORT_RX_CTL3		0x20400
#define H9P_PHY_IP_PORT_TIMER0		0x2009c
#define H9P_PHY_IP_PORT_TIMER1		0x200dc
#define H9P_PHY_IP_PORT_TIMER2		0x200a0
#define H9P_PHY_IP_PORT_TIMER3		0x200e0
#define H9P_PHY_IP_PORT_TIMER4		0x200a4
#define H9P_PHY_IP_PORT_TIMER5		0x200e4
#define H9P_PHY_IP_PORT_CLEAR0		0x20330
#define H9P_PHY_IP_PORT_CLEAR1		0x20340
#define H9P_PHY_IP_PORT_CLEAR2		0x20350

#define H9P_PORT_LTSSMCTL		0x0080
#define H9P_PORT_LTSSM_ENABLE		BIT(0)
#define H9P_PORT_IRQSTAT		0x0100
#define H9P_PORT_IRQMASK		0x0104
#define H9P_PORT_IRQMASK_PRE_LINK	0xff002fff
#define H9P_PORT_IRQSTAT_PRE_LINK	0x00ffd000
#define H9P_PORT_IRQMASK_LINK_UP	0xff002f0f
#define H9P_PORT_PWRCTL		0x0124
#define H9P_PORT_PWRCTL_INIT		0x31
#define H9P_PORT_MSIVECBASE		0x0128
#define H9P_PORT_ENABLE		0x0140
#define H9P_PORT_ENABLE_APPLE		BIT(31)
#define H9P_PORT_LINKSTS		0x0208
#define H9P_PORT_LINKSTS_LTSSM		GENMASK(13, 8)
#define H9P_PORT_LTSSM_DETECT		0x11
#define H9P_PORT_LTSSM_L0		0x14

#define H9P_LINK_SPEED_2_5GT		1
#define H9P_LINK_SPEED_5GT		2
#define H9P_LINK_SPEED_8GT		3

/*
 * Ports with a powered endpoint (pwren-gpios, e.g. the Wi-Fi chip) come up
 * from a work item after probe, so port 0 never waits for their power GPIO
 * provider. Retry the GPIO every 500 ms for at most 30 s.
 */
#define H9P_LATE_PORT_RETRY_MS		500
#define H9P_LATE_PORT_TRIES		60
/* Endpoint power: off this long first, then settle this long (A10 J172) */
#define H9P_PWREN_OFF_MS		100
#define H9P_PWREN_SETTLE_MS		100
/* Link attempts per late port (power-cycled in between) */
#define H9P_LATE_PORT_LINK_TRIES	2

/*
 * The DT keeps ASPM L1 off on every port (aspm-no-l1): iOS pairs L1 with
 * L1SS/LTR/CLKREQ# and refclk gating, and a link that can't leave L1 takes
 * the CPU with it. This lets a boot opt in per port for power testing
 * (e.g. 0x2 = Wi-Fi). L1 then follows the PCI ASPM policy:
 * /sys/module/pcie_aspm/parameters/policy = default (L1),
 * powersupersave (+ L1 substates) or performance (off again).
 */
static unsigned int aspm_l1_ports;
module_param(aspm_l1_ports, uint, 0444);
MODULE_PARM_DESC(aspm_l1_ports,
	"Bitmask of root ports that may use ASPM L1 despite the DT's aspm-no-l1 (default 0; 0x2 = Wi-Fi)");

/*
 * System sleep (s2idle). Untested on hardware, so it is refused unless a
 * boot opts in; the levels add one risk at a time:
 *  1: every port stays powered and linked. NVMe shuts down (CC.SHN) and
 *     is re-enabled by its reset path, brcmfmac does its D3 handshake.
 *     Resume checks each link and the NVMMU before any endpoint is touched.
 *  2: as 1, but ports with an endpoint power GPIO (Wi-Fi) take the iOS
 *     port sleep: PME_Turn_Off, L2, PERST#, port off. The chip stays
 *     powered (REG_ON is on the PMU, out of reach in the noirq phase); its
 *     link is retrained on resume and brcmfmac reloads the firmware.
 *  3: as 2 for every port, NVMe included (experimental).
 * Endpoints and root ports stay in D0 at every level: D3hot would take
 * the link to L1, which these ports are not trusted with (aspm-no-l1).
 * A port that doesn't come back is cut off (config space and drivers see
 * a disconnected device), so nothing touches a dead link.
 */
#define H9P_SLEEP_OFF			0
#define H9P_SLEEP_KEEP_LINKS		1
#define H9P_SLEEP_L2_POWERED		2
#define H9P_SLEEP_L2_ALL		3
static bool crash_quiesce_supported = true;
module_param(crash_quiesce_supported, bool, 0444);
MODULE_PARM_DESC(crash_quiesce_supported, "Capture-kernel host-only ANS quiesce is available");

static unsigned int h9p_suspend;
module_param_named(suspend, h9p_suspend, uint, 0644);
MODULE_PARM_DESC(suspend,
	"System sleep: 0 refuse (default), 1 keep links up, 2 also put Wi-Fi's port in L2, 3 every port in L2 (experimental)");

/* Orchestrator-only fault injection; never enabled by a normal boot. */
static bool panic_resume_once;
module_param(panic_resume_once, bool, 0600);
MODULE_PARM_DESC(panic_resume_once,
		 "j71t: deliberately panic before one PCIe resume, only with kdump armed");

static void apple_h9p_crash_resume_test(struct device *dev)
{
	if (!of_machine_is_compatible("apple,j71t") ||
	    !READ_ONCE(panic_resume_once))
		return;
	WRITE_ONCE(panic_resume_once, false);
	if (!kexec_crash_loaded()) {
		dev_warn(dev, "resume panic test refused: no capture kernel armed\n");
		return;
	}
	panic("j71t: deliberate panic before PCIe resume");
}

/* Retrain attempts per port on resume (PERST# asserted in between) */
#define H9P_WAKE_LINK_TRIES		2

/*
 * S8003 (A9, TSMC). From the iOS 16 AppleS800xPCIe/AppleS8003PCIe kexts; the
 * T8010 sequence above does not match it (status register, port registers,
 * link status, phy-ip lane mapping). See docs/pcie/kernelcache-notes.md.
 */
#define S8003_RC_LANE_CFG		0x0004	/* 0x10 | lane-cfg */
#define S8003_RC_NVME_MODE0		0x0014
#define S8003_RC_NVME_MODE1		0x0024
#define S8003_RC_STAT			0x002c
#define S8003_RC_STAT_COMMON_READY	BIT(0)
#define S8003_RC_STAT_PORT_READY	BIT(4)
#define S8003_RC_COMMON_DONE		0x0038
#define S8003_RC_PORT_CTL(port)	(0x0100 + (port) * 0x80)
#define S8003_RC_PORT_CTL_ENABLE	BIT(0)
#define S8003_RC_PORT_CTL_BIT8		BIT(8)
#define S8003_RC_PORT_PWR(port)	(0x0114 + (port) * 0x80)
#define S8003_RC_PORT_PWR_ON		BIT(0)
#define S8003_RC_PORT_REFCLK_NOGATE	BIT(8)
#define S8003_RC_PORT_REFCLK(port)	(0x0124 + (port) * 0x80)
#define S8003_RC_PORT_REFCLK_ON	BIT(0)
#define S8003_RC_PORT_READY(port)	(0x012c + (port) * 0x80)
#define S8003_RC_PORT_LSCNT(port)	(0x4020 + (port) * 0x40)

#define S8003_PORT_LINKSTS		0x0088
#define S8003_PORT_LINKSTS_UP		BIT(0)
#define S8003_PORT_MSICTL		0x0124	/* 1 | log2(vectors) << 4 */
#define S8003_PORT_LINKPMGRSTS		0x0210
#define S8003_PORT_LINKCDMSTS		0x0804
#define S8003_PORT_LINKCDMSTS_LTSSM	GENMASK(14, 9)
#define S8003_LTSSM_L0			0x11

/*
 * Port sleep, as iOS AppleS800xPCIePort::disableGated: write 0x31 to send
 * PME_Turn_Off; bit 0 clears on PME_TO_Ack, bit 5 flags a hardware timeout.
 * iOS polls the ack every 1 ms for 10 ms and continues either way, then
 * waits for L2 (link status bit 6).
 */
#define S8003_PORT_LINKSTS_L2		BIT(6)
#define S8003_PORT_PMETO		0x008c
#define S8003_PORT_PMETO_SEND		0x31
#define S8003_PORT_PMETO_PENDING	BIT(0)
#define S8003_PORT_PMETO_HW_TIMEOUT	BIT(5)
#define S8003_PME_TO_ACK_TIMEOUT_US	10000
#define S8003_L2_TIMEOUT_US		10000
/* iOS clears this port event once the link is down */
#define S8003_PORT_IRQSTAT_LINK_DOWN	BIT(12)
/* ADT perst-to-refclk-delay-us */
#define S8003_PERST_TO_REFCLK_US	60

/* NVMMU invalidate FIFO status in the port 0 block (invalidate-fifo-workaround 1) */
#define S8003_PORT_NVMMU_INV_FIFO	0x022c
#define S8003_PORT_NVMMU_INV_FIFO_EMPTY	BIT(0)
#define S8003_NVMMU_INV_FIFO_TIMEOUT_US	20000
/* invalidate-fifo-workaround 2: iOS nvme-invalidate-delay default */
#define S8003_NVMMU_INV_DELAY_US	8

/* Block sizes, to bounds-check tunable offsets */
#define H9P_RC_SIZE			0x8000
#define H9P_PHY_SIZE			0x4000
#define H9P_PORT_SIZE			0x4000
#define H9P_CFG_SIZE			0x1000

/* ADT pci-bridge t-refclk-to-perst (both bridges on j71t) */
#define S8003_T_REFCLK_TO_PERST_US	100
/* Hold endpoint reset across retained-state cleanup and clock restart. */
#define S8003_PORT_RESET_MS		100
#define S8003_PORT_LINK_TRIES		2

/* ANS-only deferral is too late to diagnose a stall in the host probe. */
static bool h9p_probe = !IS_ENABLED(CONFIG_NVME_APPLE_ANS_EARLY_USB);
module_param_named(probe, h9p_probe, bool, 0644);
MODULE_PARM_DESC(probe, "Allow j71t PCIe host probe after initramfs USB rescue is ready");

#define H9P_NVMMU_TCB_CTRL		0x0004
#define H9P_NVMMU_TCB_CTRL_INIT	0x10000
#define H9P_NVMMU_TCB_BASE_LO		0x0008
#define H9P_NVMMU_TCB_BASE_HI		0x000c
/* iOS (A9) puts a zeroed 4K scratch page here, not a TCB table */
#define H9P_NVMMU_TCB_TABLE_LO		0x0010
#define H9P_NVMMU_TCB_TABLE_HI		0x0014
#define H9P_NVMMU_SART_CTRL		0x0020
#define H9P_NVMMU_SART_VA_BASE		0x0024
#define H9P_NVMMU_SART_VA_END		0x0028
#define H9P_NVMMU_SART_PA_BASE		0x002c
/* Error registers, read by the iOS handler (AppleNVMeMMU, 0xfffffff005eea2fc) */
#define H9P_NVMMU_ERR_INFO		0x0030	/* [29:23] tag */
#define H9P_NVMMU_ERR_STATUS		0x0034	/* one bit per error type */
#define H9P_NVMMU_ERR_PRP		0x0038
#define H9P_NVMMU_ERR_ADDR_LO		0x003c
#define H9P_NVMMU_ERR_CRYPTO		0x0040
#define H9P_NVMMU_ERR_ADDR_HI		0x0044
#define H9P_NVMMU_ERR_FETCH		0x0048

/* S5L8960X DART registers, for the read-only host dump */
#define H9P_DART_TCR			0x000c
#define H9P_DART_ERROR			0x0010
#define H9P_DART_ERROR_FLAG		BIT(31)
#define H9P_DART_ERROR_ADDR		0x001c
#define H9P_DART_DIAG			0x0020
#define H9P_DART_REFLECTOR		0x0024
#define H9P_DART_FETCH			0x0030
#define H9P_DART_TTBR(i)		(0x0040 + (i) * 4)

#define H9P_NVMMU_TCB_BYTES		0x80
#define H9P_NVMMU_TCB_COUNT		128
#define H9P_NVMMU_TCB_DWORDS		(H9P_NVMMU_TCB_BYTES / sizeof(u32))
#define H9P_NVMMU_SGL_WORDS		APPLE_ANS_NVMMU_MAX_PAGES
#define H9P_NVMMU_FLATDMA_BASE		0x40000000ULL
#define H9P_NVMMU_FLATDMA_STRIDE	SZ_8M
#define H9P_NVMMU_SART_ALIGNMENT	SZ_1M
/* TCB flags, device view: 0x100 device reads memory, 0x200 device writes it */
#define H9P_NVMMU_TCB_READ		0x100
#define H9P_NVMMU_TCB_WRITE		0x200
#define H9P_NVMMU_TCB_NPAGES		1

#define H9P_DEFAULT_MSI_DOORBELL	0xbffff000ULL

struct apple_h9p_pcie;

struct apple_h9p_hw {
	/* TSMC PCIe PHY with the "phy-ip" tuning block (T8010, S8003) */
	bool has_phy_ip;
	/* Port 0 target speed when its node has no max-link-speed */
	u32 port0_default_speed;
	/* NVMMU TCB_CTRL bit that stays set while the TCB init runs */
	u32 nvmmu_init_busy;
	/* Accepted apple,lane-cfg values (bitmask) */
	u32 lane_cfgs;
	/* Port-block link status register and its link-up bit (0: unknown) */
	u32 linksts;
	u32 linksts_up;
	int (*init_common)(struct apple_h9p_pcie *pcie);
	int (*setup_port)(struct apple_h9p_pcie *pcie, unsigned int port);
	/* Link to L2 and port off for system sleep; NULL: links stay up */
	void (*port_sleep)(struct apple_h9p_pcie *pcie, unsigned int port);
	/* Refclk and port power off, PERST# already asserted (optional) */
	void (*port_off)(struct apple_h9p_pcie *pcie, unsigned int port);
};

/* Round 8 is a supervised trial; round7 retains the previous sequence. */
static enum apple_ans_boot_mode ans_boot_mode = APPLE_ANS_HANDOFF;
static const char * const ans_boot_modes[] = { "round7", "handoff", "full-cycle" };

static int ans_boot_mode_set(const char *val, const struct kernel_param *kp)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ans_boot_modes); i++) {
		if (!sysfs_streq(val, ans_boot_modes[i]))
			continue;
		*(enum apple_ans_boot_mode *)kp->arg = i;
		return 0;
	}
	return -EINVAL;
}

static int ans_boot_mode_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%s\n",
		ans_boot_modes[*(enum apple_ans_boot_mode *)kp->arg]);
}

static const struct kernel_param_ops ans_boot_mode_ops = {
	.set = ans_boot_mode_set,
	.get = ans_boot_mode_get,
};
module_param_cb(ans_boot_mode, &ans_boot_mode_ops, &ans_boot_mode, 0444);
MODULE_PARM_DESC(ans_boot_mode, "j71t ANS warm boot: handoff (trial default), full-cycle, round7");

/* Supervised alternative to clock removal; full-cycle remains the control. */
static bool ans_keep_power;
module_param(ans_keep_power, bool, 0444);
MODULE_PARM_DESC(ans_keep_power,
		 "j71t handoff: retain port 0 power/refclk under PERST across kexec (default false)");

struct apple_h9p_nvmmu {
	struct apple_h9p_pcie *pcie;
	void __iomem *base;
	bool ready;
	bool active; /* iOS setActive(false) only records state; no MMIO */
	bool sart;
	bool poisoned; /* failed invalidate: never reuse a cached translation */
	/* ADT invalidate-fifo-workaround: 0 none, 1 poll FIFO, 2 delay */
	u32 inv_fifo;
	/* Serializes invalidates (FIFO poll + write) across CPUs */
	spinlock_t inv_lock;
	/* Tags mapped since their last invalidate */
	DECLARE_BITMAP(used, APPLE_ANS_NVMMU_MAX_REQS);
	u64 pa_base;
	u32 va_base;
	u32 size;
	void *tcb;
	void *tcb_table;
	void *tcb_sgl;
	size_t tcb_size;
	size_t tcb_table_size;
	size_t tcb_sgl_size;
	dma_addr_t tcb_dma;
	dma_addr_t tcb_table_dma;
	dma_addr_t tcb_sgl_dma;
	/*
	 * Port 0's DART, mapped here only to read its status for the ANS
	 * bring-up diagnostics. apple-dart owns it.
	 */
	void __iomem *dart;
	u64 dart_base;		/* its apple,dma-range start */
};

struct apple_h9p_pcie {
	struct device *dev;
	struct platform_device *pdev;
	struct pci_host_bridge *bridge;
	struct pci_config_window *cfgwin;
	const struct apple_h9p_hw *hw;

	void __iomem *base_config;
	void __iomem *base_rc;
	void __iomem *base_phy;
	void __iomem *base_phy_ip;
	void __iomem *base_port[H9P_NUM_PORTS];

	struct clk_bulk_data clks[3];
	struct device_node *port_nodes[H9P_NUM_PORTS];
	struct gpio_desc *perst[H9P_NUM_PORTS];
	struct gpio_desc *clkreq[H9P_NUM_PORTS];
	u32 max_speed[H9P_NUM_PORTS];
	bool nvme_mode[H9P_NUM_PORTS];
	struct gpio_descs *devpwr;
	struct pinctrl *pinctrl;
	u32 enabled_ports;
	/* Not up yet: brought up by late_work once their power GPIO exists */
	u32 late_ports;
	struct gpio_desc *pwren[H9P_NUM_PORTS];
	struct gpio_desc *devwake[H9P_NUM_PORTS];
	struct delayed_work late_work;
	unsigned int late_tries;
	u32 lane_cfg;

	struct apple_h9p_nvmmu nvmmu;
	enum apple_ans_boot_mode ans_mode;
	bool ans_boot_retry_used;
	bool ans_deep_recovery;
	bool ans_gen1;
	bool ans_pre_enumeration;
	u16 link_cap[H9P_NUM_PORTS];
	bool ans_rescue_allowed;
	unsigned int ans_rescue_attempts;
	int ans_rescue_result;

	struct device **pd_dev;
	struct device_link **pd_link;
	int pd_count;

	/* System sleep, see h9p_suspend */
	bool ans_sleep_stopped;	/* endpoint certified EN/SHN/RDY/BME=0 */
	u32 sleep_ports;	/* chosen at prepare: go to L2 */
	u32 resume_ports;	/* linked at suspend_noirq: bring back */
	u32 down_ports;		/* link down on purpose or dead: no config access below */
	struct pci_dev *pinned[8];	/* held in D0 over the sleep */
	unsigned int npinned;
	bool pin_overflow;

	DECLARE_BITMAP(used_msi[H9P_NUM_PORTS], H9P_MSI_PER_PORT);
	u64 msi_doorbell;
	/* Protects the per-port MSI allocation bitmaps. */
	spinlock_t used_msi_lock;
	struct irq_domain *irq_dom;
	struct irq_fwspec msi_fwspec;
	u32 nvecs;
};

static inline void h9p_rmw(void __iomem *addr, u32 clear, u32 set)
{
	writel((readl(addr) & ~clear) | set, addr);
}

static inline void h9p_rmww(void __iomem *addr, u16 clear, u16 set)
{
	writew((readw(addr) & ~clear) | set, addr);
}

static inline void h9p_writel_flush(u32 value, void __iomem *addr)
{
	writel(value, addr);
	readl(addr);
}

static void apple_h9p_pcie_detach_genpd(struct apple_h9p_pcie *pcie)
{
	int i;

	for (i = pcie->pd_count - 1; i >= 0; i--) {
		if (pcie->pd_link[i])
			device_link_del(pcie->pd_link[i]);
		if (!IS_ERR_OR_NULL(pcie->pd_dev[i]))
			dev_pm_domain_detach(pcie->pd_dev[i], true);
	}
}

static int apple_h9p_pcie_attach_genpd(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	int i;

	pcie->pd_count = of_count_phandle_with_args(dev->of_node,
						    "power-domains",
						    "#power-domain-cells");
	if (pcie->pd_count <= 1)
		return 0;

	pcie->pd_dev = devm_kcalloc(dev, pcie->pd_count,
				    sizeof(*pcie->pd_dev), GFP_KERNEL);
	if (!pcie->pd_dev)
		return -ENOMEM;

	pcie->pd_link = devm_kcalloc(dev, pcie->pd_count,
				     sizeof(*pcie->pd_link), GFP_KERNEL);
	if (!pcie->pd_link)
		return -ENOMEM;

	for (i = 0; i < pcie->pd_count; i++) {
		pcie->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(pcie->pd_dev[i])) {
			apple_h9p_pcie_detach_genpd(pcie);
			return PTR_ERR(pcie->pd_dev[i]);
		}

		pcie->pd_link[i] = device_link_add(dev, pcie->pd_dev[i],
						   DL_FLAG_STATELESS |
						   DL_FLAG_PM_RUNTIME |
						   DL_FLAG_RPM_ACTIVE);
		if (!pcie->pd_link[i]) {
			apple_h9p_pcie_detach_genpd(pcie);
			return -EINVAL;
		}

		/*
		 * Keep the domains out of system sleep: genpd would power them
		 * off at suspend_noirq, under linked ports and the DARTs. This
		 * driver takes the links down itself (h9p_suspend).
		 */
		dev_pm_syscore_device(pcie->pd_dev[i], true);
	}

	return 0;
}

static void apple_h9p_pcie_genpd_cleanup(void *data)
{
	apple_h9p_pcie_detach_genpd(data);
}

static void apple_h9p_pcie_clk_cleanup(void *data)
{
	struct apple_h9p_pcie *pcie = data;

	clk_bulk_disable_unprepare(ARRAY_SIZE(pcie->clks), pcie->clks);
}

static struct apple_h9p_pcie *apple_h9p_pcie_lookup(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);

	return bridge ? pci_host_bridge_priv(bridge) : NULL;
}

/* Keep the generic failed-link quirk from undoing a successful Gen1
 * recovery. supported_speeds is initialized before HEADER fixups; scope
 * this software ceiling to our recovered port 0, without falsifying ECAM.
 */
static void apple_h9p_ans_gen1_fixup(struct pci_dev *dev)
{
	struct pci_host_bridge *bridge;
	struct apple_h9p_pcie *pcie;
	struct device *parent;

	if (!pci_is_root_bus(dev->bus) || dev->devfn != PCI_DEVFN(0, 0))
		return;
	bridge = pci_find_host_bridge(dev->bus);
	parent = bridge->dev.parent;
	if (!parent || !of_device_is_compatible(parent->of_node, "apple,s8003-pcie"))
		return;
	pcie = apple_h9p_pcie_lookup(parent);
	if (pcie && pcie->ans_gen1) {
		dev->supported_speeds = PCI_EXP_LNKCAP2_SLS_2_5GB;
		pci_info(dev, "Apple ANS: STEP port 0 Gen1 recovery ceiling retained\n");
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_APPLE, PCI_ANY_ID, apple_h9p_ans_gen1_fixup);

/* The root port that a bus below the root bus hangs off */
static unsigned int apple_h9p_pcie_bus_port(struct pci_bus *bus)
{
	while (bus->parent && !pci_is_root_bus(bus->parent))
		bus = bus->parent;

	return bus->self ? PCI_SLOT(bus->self->devfn) : H9P_NUM_PORTS;
}

/*
 * A root port exists only while its port is up. Below a port whose link is
 * down (asleep, or dead after a resume) no request may go out on the link.
 */
static bool apple_h9p_pcie_cfg_ok(struct pci_bus *bus, unsigned int devfn)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(cfg->parent);
	u32 down;

	if (bus->number == cfg->busr.start)
		return pcie && PCI_SLOT(devfn) < H9P_NUM_PORTS &&
		       (pcie->enabled_ports & BIT(PCI_SLOT(devfn)));

	down = pcie ? READ_ONCE(pcie->down_ports) : 0;
	return !down || !(down & BIT(apple_h9p_pcie_bus_port(bus)));
}

static int apple_h9p_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				      int where, int size, u32 *val)
{
	if (!apple_h9p_pcie_cfg_ok(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return pci_generic_config_read(bus, devfn, where, size, val);
}

static int apple_h9p_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 val)
{
	if (!apple_h9p_pcie_cfg_ok(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (where <= PCI_INTERRUPT_LINE && where + size > PCI_INTERRUPT_LINE)
		val |= 0xffu << ((PCI_INTERRUPT_LINE - where) << 3);

	return pci_generic_config_write(bus, devfn, where, size, val);
}

static int apple_h9p_pcie_device_port(struct apple_h9p_pcie *pcie,
				      struct device *dev)
{
	struct pci_dev *pdev;

	if (!dev_is_pci(dev))
		return -ENODEV;

	pdev = to_pci_dev(dev);
	if (!pdev->bus)
		return -ENODEV;

	/* Completion runs concurrently with late-port rescans. Bridge bus
	 * registers are routing state, not stable ownership: use the same
	 * topology walk as the config-access guard, with no ECAM reads.
	 */
	return apple_h9p_pcie_bus_port(pdev->bus);
}

static void apple_h9p_msi_compose_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct apple_h9p_pcie *pcie = irq_data_get_irq_chip_data(d);

	msg->address_lo = lower_32_bits(pcie->msi_doorbell);
	msg->address_hi = upper_32_bits(pcie->msi_doorbell);
	msg->data = d->hwirq;
}

static struct irq_chip apple_h9p_msi_bottom_chip = {
	.name = "Apple H9P PCIe MSI",
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_eoi = irq_chip_eoi_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_compose_msi_msg = apple_h9p_msi_compose_msg,
};

static int apple_h9p_msi_alloc(struct irq_domain *domain, unsigned int virq,
			       unsigned int nr_irqs, void *args)
{
	struct apple_h9p_pcie *pcie = domain->host_data;
	struct irq_fwspec fwspec = pcie->msi_fwspec;
	msi_alloc_info_t *info = args;
	struct msi_desc *desc = info ? info->desc : NULL;
	struct device *msi_dev = desc ? msi_desc_to_dev(desc) : NULL;
	struct pci_dev *pdev;
	unsigned long flags;
	unsigned int port;
	int ret, slot;

	if (nr_irqs != 1 || !msi_dev || !dev_is_pci(msi_dev))
		return -ENOSPC;

	pdev = to_pci_dev(msi_dev);
	if (!pdev->bus)
		return -ENOSPC;

	port = apple_h9p_pcie_device_port(pcie, msi_dev);
	if (port >= H9P_NUM_PORTS)
		return -ENOSPC;
	if (!(pcie->enabled_ports & BIT(port)))
		return -ENOSPC;

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	slot = find_first_zero_bit(pcie->used_msi[port], H9P_MSI_PER_PORT);
	if (slot >= H9P_MSI_PER_PORT) {
		spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
		return -ENOSPC;
	}
	__set_bit(slot, pcie->used_msi[port]);
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);

	fwspec.param[fwspec.param_count - 2] +=
		port * H9P_MSI_PER_PORT + slot;
	ret = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (ret) {
		spin_lock_irqsave(&pcie->used_msi_lock, flags);
		__clear_bit(slot, pcie->used_msi[port]);
		spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
		return ret;
	}

	irq_domain_set_hwirq_and_chip(domain, virq,
				      port * H9P_MSI_PER_PORT + slot,
				      &apple_h9p_msi_bottom_chip, pcie);
	return 0;
}

static void apple_h9p_msi_free(struct irq_domain *domain, unsigned int virq,
			       unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct apple_h9p_pcie *pcie = d ? irq_data_get_irq_chip_data(d) : NULL;
	irq_hw_number_t hwirq;
	unsigned long flags;
	unsigned int i;

	if (!pcie || !d)
		return;

	hwirq = d->hwirq;
	irq_domain_free_irqs_parent(domain, virq, nr_irqs);

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	for (i = 0; i < nr_irqs; i++) {
		irq_hw_number_t irq = hwirq + i;
		unsigned int port = irq / H9P_MSI_PER_PORT;
		unsigned int slot = irq % H9P_MSI_PER_PORT;

		if (port < H9P_NUM_PORTS)
			__clear_bit(slot, pcie->used_msi[port]);
	}
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
}

static const struct irq_domain_ops apple_h9p_msi_domain_ops = {
	.alloc = apple_h9p_msi_alloc,
	.free = apple_h9p_msi_free,
};

static const struct msi_parent_ops apple_h9p_msi_parent_ops = {
	.supported_flags = MSI_GENERIC_FLAGS_MASK | MSI_FLAG_PCI_MSIX |
			   MSI_FLAG_MULTI_PCI_MSI,
	.required_flags = MSI_FLAG_USE_DEF_DOM_OPS |
			  MSI_FLAG_USE_DEF_CHIP_OPS |
			  MSI_FLAG_PCI_MSI_MASK_PARENT,
	.chip_flags = MSI_CHIP_FLAG_SET_EOI,
	.bus_select_token = DOMAIN_BUS_PCI_MSI,
	.init_dev_msi_info = msi_lib_init_dev_msi_info,
};

static int apple_h9p_pcie_setup_msi(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct irq_domain_info info = {
		.fwnode = fwnode,
		.ops = &apple_h9p_msi_domain_ops,
		.size = H9P_NUM_MSI,
		.host_data = pcie,
	};
	struct of_phandle_args args = {};
	int ret;

	ret = of_parse_phandle_with_args(to_of_node(fwnode), "msi-ranges",
					 "#interrupt-cells", 0, &args);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(to_of_node(fwnode), "msi-ranges",
					 args.args_count + 1, &pcie->nvecs);
	if (ret)
		goto out_put_node;
	if (pcie->nvecs != H9P_NUM_MSI) {
		ret = -EINVAL;
		goto out_put_node;
	}

	of_phandle_args_to_fwspec(args.np, args.args, args.args_count,
				  &pcie->msi_fwspec);
	info.parent = irq_find_matching_fwspec(&pcie->msi_fwspec,
					       DOMAIN_BUS_WIRED);
	if (!info.parent) {
		ret = -ENXIO;
		goto out_put_node;
	}

	pcie->irq_dom = msi_create_parent_irq_domain(&info,
						     &apple_h9p_msi_parent_ops);
	if (!pcie->irq_dom)
		ret = -ENOMEM;

out_put_node:
	of_node_put(args.np);
	return ret;
}

static void apple_h9p_pcie_msi_cleanup(void *data)
{
	struct apple_h9p_pcie *pcie = data;

	irq_domain_remove(pcie->irq_dom);
}

static u64 apple_h9p_read_pci_cap(struct apple_h9p_pcie *pcie,
				  unsigned int busdevfn, u32 type)
{
	void __iomem *cfg = pcie->base_config + (busdevfn << 12);
	u32 ptr = readl(cfg + PCI_CAPABILITY_LIST) & 0xff;
	unsigned int ttl = 48;

	while (ptr && ttl--) {
		u32 next = readl(cfg + ptr);

		if (ptr < PCI_STD_HEADER_SIZEOF || ptr > 0xfc || (ptr & 3))
			break;
		if ((next & 0xff) == type)
			return ptr;
		ptr = (next >> 8) & 0xff;
	}

	return 0;
}

static int apple_h9p_wait(void __iomem *addr, u32 mask, u32 min, u32 max,
			  unsigned long timeout_us)
{
	u32 val;

	return readl_poll_timeout(addr, val, (val & mask) >= min &&
				  (val & mask) <= max, 1000, timeout_us);
}

static int apple_h9p_wait_gpio(struct gpio_desc *desc, int value,
			       unsigned long timeout_us)
{
	ktime_t timeout = ktime_add_us(ktime_get(), timeout_us);
	int ret;

	do {
		ret = gpiod_get_value_cansleep(desc);
		if (ret < 0)
			return ret;
		if (ret == value)
			return 0;
		usleep_range(1000, 2000);
	} while (ktime_before(ktime_get(), timeout));

	return -ETIMEDOUT;
}

/*
 * iOS panics on every NVMMU error ("nvme mmu fetch error ...") and never
 * acks one, so there is no known way to clear it. Log everything once and
 * mask the interrupt so it can't storm.
 */
static irqreturn_t apple_h9p_nvmmu_irq(int irq, void *data)
{
	struct apple_h9p_nvmmu *nvmmu = data;
	struct apple_h9p_pcie *pcie = nvmmu->pcie;
	void __iomem *b = nvmmu->base;

	dev_err(pcie->dev,
		"NVMMU fault: status %#x info %#x (tag %lu) prp %#x addr %#x:%08x crypto %#x fetch %#x; masking the NVMMU interrupt\n",
		readl(b + H9P_NVMMU_ERR_STATUS), readl(b + H9P_NVMMU_ERR_INFO),
		FIELD_GET(GENMASK(29, 23), readl(b + H9P_NVMMU_ERR_INFO)),
		readl(b + H9P_NVMMU_ERR_PRP), readl(b + H9P_NVMMU_ERR_ADDR_HI),
		readl(b + H9P_NVMMU_ERR_ADDR_LO), readl(b + H9P_NVMMU_ERR_CRYPTO),
		readl(b + H9P_NVMMU_ERR_FETCH));
	disable_irq_nosync(irq);
	return IRQ_HANDLED;
}

static void apple_h9p_iounmap(void *addr)
{
	iounmap(addr);
}

/* Map port 0's DART (first iommu-map entry) for read-only diagnostics */
static void apple_h9p_map_dart(struct apple_h9p_pcie *pcie)
{
	struct device_node *np;

	np = of_parse_phandle(pcie->dev->of_node, "iommu-map", 1);
	if (!np)
		return;
	if (of_device_is_compatible(np, "apple,s5l8960x-dart") &&
	    of_device_is_available(np)) {
		u64 range[2];

		if (!of_property_read_u64_array(np, "apple,dma-range", range, 2))
			pcie->nvmmu.dart_base = range[0];
		/* of_iomap: apple-dart already requested the region */
		pcie->nvmmu.dart = of_iomap(np, 0);
		if (pcie->nvmmu.dart &&
		    devm_add_action_or_reset(pcie->dev, apple_h9p_iounmap,
					     (void __force *)pcie->nvmmu.dart))
			pcie->nvmmu.dart = NULL;
	}
	of_node_put(np);
}

/*
 * The loader copies the runtime ADT nvme-mmu0 "sart-region" (the HMB iBoot
 * set up) into the reserved-memory node behind apple,nvme-hmb. Map it through
 * the NVMMU SART window. Without it FlatDMA still works and the ANS glue can
 * fall back to a DART-mapped HMB.
 */
static int apple_h9p_nvmmu_get_sart(struct apple_h9p_pcie *pcie)
{
	struct apple_h9p_nvmmu *nvmmu = &pcie->nvmmu;
	struct device *dev = pcie->dev;
	struct device_node *mem_np;
	struct resource res;
	resource_size_t size;
	u32 iova;
	int ret;

	mem_np = of_parse_phandle(dev->of_node, "apple,nvme-hmb", 0);
	if (!mem_np)
		return -ENODEV;

	ret = of_address_to_resource(mem_np, 0, &res);
	if (ret)
		goto out_put_node;

	ret = of_property_read_u32(dev->of_node, "apple,nvmmu-iova", &iova);
	if (ret)
		goto out_put_node;

	size = resource_size(&res);
	if (!res.start || !size) {
		ret = -ENODEV;
		goto out_put_node;
	}
	/* iOS: MB-aligned, and the window must end at or below 0xbff00000 */
	if (size < H9P_NVMMU_SART_ALIGNMENT || size > U32_MAX ||
	    !IS_ALIGNED(res.start, H9P_NVMMU_SART_ALIGNMENT) ||
	    !IS_ALIGNED(size, H9P_NVMMU_SART_ALIGNMENT) ||
	    !IS_ALIGNED(iova, H9P_NVMMU_SART_ALIGNMENT) ||
	    iova < 0x80000000U ||
	    (u64)iova + size - H9P_NVMMU_SART_ALIGNMENT > 0xbff00000U) {
		ret = -EINVAL;
		goto out_put_node;
	}

	nvmmu->pa_base = res.start;
	nvmmu->va_base = iova;
	nvmmu->size = size;

out_put_node:
	of_node_put(mem_np);
	return ret;
}

/* NVMMU registers; also re-run on resume if they didn't survive the sleep */
static int apple_h9p_nvmmu_program(struct apple_h9p_pcie *pcie)
{
	struct apple_h9p_nvmmu *nvmmu = &pcie->nvmmu;
	u32 val;
	int ret;

	if (nvmmu->inv_fifo == 1) {
		ret = readl_poll_timeout(pcie->base_port[0] +
					S8003_PORT_NVMMU_INV_FIFO, val,
					val & S8003_PORT_NVMMU_INV_FIFO_EMPTY,
					10, S8003_NVMMU_INV_FIFO_TIMEOUT_US);
		if (ret) {
			dev_err(pcie->dev, "NVMMU: FIFO %#x blocks reinitialization\n", val);
			return ret;
		}
	}
	/* No old aperture may remain enabled while its address is replaced. */
	h9p_writel_flush(0, nvmmu->base + H9P_NVMMU_SART_CTRL);
	/* Publish WC descriptor/scratch initialization before the hardware init. */
	dma_wmb();
	/* Same order as iOS: scratch page, TCB base, init, SART */
	h9p_writel_flush(lower_32_bits(nvmmu->tcb_table_dma),
			 nvmmu->base + H9P_NVMMU_TCB_TABLE_LO);
	h9p_writel_flush(upper_32_bits(nvmmu->tcb_table_dma),
			 nvmmu->base + H9P_NVMMU_TCB_TABLE_HI);
	h9p_writel_flush(lower_32_bits(nvmmu->tcb_dma),
			 nvmmu->base + H9P_NVMMU_TCB_BASE_LO);
	h9p_writel_flush(upper_32_bits(nvmmu->tcb_dma),
			 nvmmu->base + H9P_NVMMU_TCB_BASE_HI);
	h9p_writel_flush(H9P_NVMMU_TCB_CTRL_INIT,
			 nvmmu->base + H9P_NVMMU_TCB_CTRL);

	ret = apple_h9p_wait(nvmmu->base + H9P_NVMMU_TCB_CTRL,
			     pcie->hw->nvmmu_init_busy, 0, 0, 250000);
	if (ret)
		return ret;

	if (nvmmu->sart) {
		h9p_writel_flush(nvmmu->va_base - 0x80000000U,
				 nvmmu->base + H9P_NVMMU_SART_VA_BASE);
		h9p_writel_flush(nvmmu->va_base + nvmmu->size - 0x80100000U,
				 nvmmu->base + H9P_NVMMU_SART_VA_END);
		h9p_writel_flush(nvmmu->pa_base >> 20,
				 nvmmu->base + H9P_NVMMU_SART_PA_BASE);
		h9p_writel_flush(1, nvmmu->base + H9P_NVMMU_SART_CTRL);
	}
	return 0;
}

static int apple_h9p_setup_nvmmu(struct apple_h9p_pcie *pcie)
{
	struct apple_h9p_nvmmu *nvmmu = &pcie->nvmmu;
	struct device *dev = pcie->dev;
	int irq;
	int ret;

	if (!(pcie->enabled_ports & BIT(0)) || !nvmmu->base)
		return 0;

	nvmmu->pcie = pcie;
	spin_lock_init(&nvmmu->inv_lock);
	of_property_read_u32(dev->of_node, "apple,nvmmu-invalidate-fifo",
			     &nvmmu->inv_fifo);
	if (nvmmu->inv_fifo > 2)
		return dev_err_probe(dev, -EINVAL,
				     "bad apple,nvmmu-invalidate-fifo %u\n",
				     nvmmu->inv_fifo);
	ret = apple_h9p_nvmmu_get_sart(pcie);
	if (ret == -ENODEV)
		dev_info(dev, "NVMMU: no loader-provided HMB (apple,nvme-hmb empty), SART off\n");
	else if (ret)
		return dev_err_probe(dev, ret, "NVMMU: bad apple,nvme-hmb\n");
	nvmmu->sart = !ret;

	/* All 128 TCBs the hardware can index, so no tag reads past the end */
	nvmmu->tcb_size = round_up(H9P_NVMMU_TCB_COUNT * H9P_NVMMU_TCB_BYTES,
				   PAGE_SIZE);
	nvmmu->tcb_table_size = PAGE_SIZE * 16;
	nvmmu->tcb_sgl_size = round_up(APPLE_ANS_NVMMU_MAX_REQS *
				       H9P_NVMMU_SGL_WORDS * sizeof(u32),
				       PAGE_SIZE);

	nvmmu->tcb = dmam_alloc_attrs(dev, nvmmu->tcb_size, &nvmmu->tcb_dma,
				      GFP_KERNEL | __GFP_ZERO,
				      DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb)
		return -ENOMEM;

	nvmmu->tcb_table = dmam_alloc_attrs(dev, nvmmu->tcb_table_size,
					    &nvmmu->tcb_table_dma,
					    GFP_KERNEL | __GFP_ZERO,
					    DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb_table)
		return -ENOMEM;

	nvmmu->tcb_sgl = dmam_alloc_attrs(dev, nvmmu->tcb_sgl_size,
					  &nvmmu->tcb_sgl_dma,
					  GFP_KERNEL | __GFP_ZERO,
					  DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb_sgl)
		return -ENOMEM;

	/* Port bring-up has isolated the old endpoint before replacing tables. */
	ret = apple_h9p_nvmmu_program(pcie);
	if (ret)
		return ret;

	irq = platform_get_irq_byname_optional(pcie->pdev, "nvmmu");
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, apple_h9p_nvmmu_irq, 0,
				       dev_name(dev), nvmmu);
		if (ret)
			return ret;
	} else if (irq != -ENXIO) {
		return irq;
	}

	apple_h9p_map_dart(pcie);

	nvmmu->ready = true;
	nvmmu->active = true;
	if (nvmmu->sart)
		dev_info(dev, "NVMMU: ready, SART window %#x -> %pa size %#x, invalidate-fifo %u\n",
			 nvmmu->va_base, &nvmmu->pa_base, nvmmu->size,
			 nvmmu->inv_fifo);
	else
		dev_info(dev, "NVMMU: ready (FlatDMA only), invalidate-fifo %u\n",
			 nvmmu->inv_fifo);

	return 0;
}

/*
 * Invalidate one TCB in the NVMMU cache. iOS (AppleNVMeMMU) with
 * invalidate-fifo-workaround 1 waits for the invalidate FIFO in the port 0
 * block to drain before each write, and never reads the register back;
 * overrunning the FIFO is suspected to stall the port's inbound path.
 */
static int apple_h9p_nvmmu_invalidate(struct apple_h9p_pcie *pcie,
				      unsigned int tag)
{
	struct apple_h9p_nvmmu *nvmmu = &pcie->nvmmu;
	unsigned long flags;
	u32 val;
	int ret = 0;

	spin_lock_irqsave(&nvmmu->inv_lock, flags);
	if (nvmmu->inv_fifo == 1)
		ret = readl_poll_timeout_atomic(pcie->base_port[0] +
						S8003_PORT_NVMMU_INV_FIFO, val,
						val & S8003_PORT_NVMMU_INV_FIFO_EMPTY,
						1, S8003_NVMMU_INV_FIFO_TIMEOUT_US);
	else if (nvmmu->inv_fifo == 2)
		udelay(S8003_NVMMU_INV_DELAY_US);
	if (!ret)
		writel(tag, nvmmu->base + H9P_NVMMU_TCB_CTRL);
	spin_unlock_irqrestore(&nvmmu->inv_lock, flags);

	if (ret)
		dev_err_ratelimited(pcie->dev,
				    "NVMMU: invalidate FIFO did not drain (%#x), tag %u not invalidated\n",
				    val, tag);
	return ret;
}

static struct apple_h9p_pcie *apple_ans_host(struct device *dev)
{
	struct device *host_dev = dev;

	while (host_dev && host_dev->bus == dev->bus)
		host_dev = host_dev->parent;
	if (!host_dev || !host_dev->parent)
		return NULL;

	return apple_h9p_pcie_lookup(host_dev->parent);
}

enum apple_ans_boot_mode apple_ans_boot_mode(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);

	return pcie ? pcie->ans_mode : APPLE_ANS_ROUND7;
}
EXPORT_SYMBOL_GPL(apple_ans_boot_mode);

/* Set only after the endpoint has stopped and relinquished DMA. */
int apple_ans_set_sleep_stopped(struct device *dev, bool stopped)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);

	if (!pcie || !of_machine_is_compatible("apple,j71t"))
		return -EOPNOTSUPP;
	WRITE_ONCE(pcie->ans_sleep_stopped, stopped);
	return 0;
}
EXPORT_SYMBOL_GPL(apple_ans_set_sleep_stopped);

/* Caller has frozen/drained I/O; no new data mappings can be submitted. */
int apple_ans_nvmmu_drain(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);
	struct apple_h9p_nvmmu *nvmmu;
	u32 val;
	int ret = 0;

	if (!pcie || !pcie->nvmmu.ready)
		return -ENODEV;
	nvmmu = &pcie->nvmmu;
	/* Retire completions, not just queued invalidates: used tags at rest
	 * mean a request never released its mapping. Never silently clear them.
	 */
	if (READ_ONCE(nvmmu->poisoned) ||
	    !bitmap_empty(nvmmu->used, APPLE_ANS_NVMMU_MAX_REQS))
		return -EBUSY;
	if (nvmmu->inv_fifo == 1)
		ret = readl_poll_timeout(pcie->base_port[0] +
				S8003_PORT_NVMMU_INV_FIFO, val,
				val & S8003_PORT_NVMMU_INV_FIFO_EMPTY,
				10, S8003_NVMMU_INV_FIFO_TIMEOUT_US);
	dev_info(dev, "Apple ANS: outgoing NVMMU FIFO empty check (20ms) result %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_ans_nvmmu_drain);

/*
 * Is the endpoint's link still up? A BAR access over a dead link can stall
 * the CPU forever on these root complexes, so the ANS glue checks this before
 * the riskier steps of its bring-up.
 */
int apple_ans_link_up(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);
	int port;

	if (!pcie)
		return -ENODEV;
	port = apple_h9p_pcie_device_port(pcie, dev);
	if (port < 0 || port >= H9P_NUM_PORTS ||
	    !(pcie->enabled_ports & BIT(port)))
		return -ENODEV;
	if (!pcie->hw->linksts)
		return 1;	/* not known for T8010 */

	return !!(readl(pcie->base_port[port] + pcie->hw->linksts) &
		  pcie->hw->linksts_up);
}
EXPORT_SYMBOL_GPL(apple_ans_link_up);

/*
 * SoC-side state of the endpoint's port, NVMMU and DART. None of these
 * reads goes over the PCIe link, so they can't stall on a wedged
 * endpoint; the ANS glue watches them around CC.EN instead of hammering
 * the controller with non-posted CSTS reads.
 */
int apple_ans_host_state(struct device *dev, struct apple_ans_host_state *st)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);
	void __iomem *pb;

	memset(st, 0, sizeof(*st));
	if (!pcie || apple_h9p_pcie_device_port(pcie, dev) != 0 ||
	    !(pcie->enabled_ports & BIT(0)))
		return -ENODEV;

	pb = pcie->base_port[0];
	st->link_up = pcie->hw->linksts ?
		!!(readl(pb + pcie->hw->linksts) & pcie->hw->linksts_up) : 1;
	if (pcie->hw->linksts == S8003_PORT_LINKSTS)
		st->ltssm = FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM,
				      readl(pb + S8003_PORT_LINKCDMSTS));
	st->port_irq = readl(pb + H9P_PORT_IRQSTAT);
	if (pcie->nvmmu.ready)
		st->nvmmu_err = readl(pcie->nvmmu.base + H9P_NVMMU_ERR_STATUS);
	if (pcie->nvmmu.dart) {
		st->dart_err = readl(pcie->nvmmu.dart + H9P_DART_ERROR);
		st->dart_addr = readl(pcie->nvmmu.dart + H9P_DART_ERROR_ADDR);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(apple_ans_host_state);

void apple_ans_host_dump(struct device *dev, const char *when)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);
	void __iomem *rc, *pb, *nv, *da;

	if (!pcie || apple_h9p_pcie_device_port(pcie, dev) != 0 ||
	    !(pcie->enabled_ports & BIT(0)))
		return;
	/* The rc/port offsets below are the S8003 ones */
	if (pcie->hw->linksts != S8003_PORT_LINKSTS)
		return;

	rc = pcie->base_rc;
	pb = pcie->base_port[0];
	dev_info(dev,
		 "host %s: rc ctl %#x pwr %#x refclk %#x rdy %#x stat %#x; port linksts %#x cdm %#x irq %#x/%#x msi %#x/%#x en %#x pmgr %#x\n",
		 when, readl(rc + S8003_RC_PORT_CTL(0)),
		 readl(rc + S8003_RC_PORT_PWR(0)),
		 readl(rc + S8003_RC_PORT_REFCLK(0)),
		 readl(rc + S8003_RC_PORT_READY(0)), readl(rc + S8003_RC_STAT),
		 readl(pb + S8003_PORT_LINKSTS),
		 readl(pb + S8003_PORT_LINKCDMSTS),
		 readl(pb + H9P_PORT_IRQSTAT), readl(pb + H9P_PORT_IRQMASK),
		 readl(pb + S8003_PORT_MSICTL),
		 readl(pb + H9P_PORT_MSIVECBASE), readl(pb + H9P_PORT_ENABLE),
		 readl(pb + S8003_PORT_LINKPMGRSTS));

	dev_info(dev, "host %s: invalidate FIFO %#x mode %u poisoned %u used %*pb\n",
		 when, readl(pb + S8003_PORT_NVMMU_INV_FIFO),
		 pcie->nvmmu.inv_fifo, READ_ONCE(pcie->nvmmu.poisoned),
		 APPLE_ANS_NVMMU_MAX_REQS, pcie->nvmmu.used);
	nv = pcie->nvmmu.base;
	if (pcie->nvmmu.ready)
		dev_info(dev,
			 "host %s: nvmmu tcb %#x:%08x scratch %#x:%08x sart %#x %#x..%#x pa %#x; err %#x %#x %#x %#x:%08x %#x %#x\n",
			 /* Never read the invalidate command register in a live dump. */
			 when,
			 readl(nv + H9P_NVMMU_TCB_BASE_HI),
			 readl(nv + H9P_NVMMU_TCB_BASE_LO),
			 readl(nv + H9P_NVMMU_TCB_TABLE_HI),
			 readl(nv + H9P_NVMMU_TCB_TABLE_LO),
			 readl(nv + H9P_NVMMU_SART_CTRL),
			 readl(nv + H9P_NVMMU_SART_VA_BASE),
			 readl(nv + H9P_NVMMU_SART_VA_END),
			 readl(nv + H9P_NVMMU_SART_PA_BASE),
			 readl(nv + H9P_NVMMU_ERR_STATUS),
			 readl(nv + H9P_NVMMU_ERR_INFO),
			 readl(nv + H9P_NVMMU_ERR_PRP),
			 readl(nv + H9P_NVMMU_ERR_ADDR_HI),
			 readl(nv + H9P_NVMMU_ERR_ADDR_LO),
			 readl(nv + H9P_NVMMU_ERR_CRYPTO),
			 readl(nv + H9P_NVMMU_ERR_FETCH));

	da = pcie->nvmmu.dart;
	if (da)
		dev_info(dev,
			 "host %s: dart tcr %#x err %#x addr %#x diag %#x reflector %#x fetch %#x ttbr %#x %#x %#x %#x\n",
			 when, readl(da + H9P_DART_TCR), readl(da + H9P_DART_ERROR),
			 readl(da + H9P_DART_ERROR_ADDR), readl(da + H9P_DART_DIAG),
			 readl(da + H9P_DART_REFLECTOR), readl(da + H9P_DART_FETCH),
			 readl(da + H9P_DART_TTBR(0)), readl(da + H9P_DART_TTBR(1)),
			 readl(da + H9P_DART_TTBR(2)), readl(da + H9P_DART_TTBR(3)));
}
EXPORT_SYMBOL_GPL(apple_ans_host_dump);

/* Port 0 DART, stream 0: what the hardware finds for DART-side @addr */
static void apple_h9p_dart_walk(struct apple_h9p_pcie *pcie, struct device *dev,
				const char *what, const char *view, u32 addr)
{
	u32 ttbr = readl(pcie->nvmmu.dart + H9P_DART_TTBR(addr >> 30));
	phys_addr_t pa = (phys_addr_t)(ttbr & GENMASK(23, 0)) << 12;
	u32 ste_idx = (addr >> 21) & 0x1ff, pte_idx = (addr >> 12) & 0x1ff;
	u64 ste = 0, pte = 0;

	if ((ttbr & BIT(31)) && pfn_valid(PHYS_PFN(pa))) {
		ste = READ_ONCE(((u64 *)phys_to_virt(pa))[ste_idx]);
		pa = ste & GENMASK_ULL(35, 12);
		if ((ste & BIT(0)) && pfn_valid(PHYS_PFN(pa)))
			pte = READ_ONCE(((u64 *)phys_to_virt(pa))[pte_idx]);
	}
	dev_info(dev, "dart walk %s, %s %#x: TTBR%u %#x -> STE[%#x] %#llx -> PTE[%#x] %#llx\n",
		 what, view, addr, addr >> 30, ttbr, ste_idx, ste, pte_idx, pte);
}

/*
 * Walk the port 0 DART's stream 0 tables for device IOVA @iova, both as the
 * hardware sees it (window base removed) and raw, to show where Linux put
 * the entry and where the DART will look.
 */
void apple_ans_dart_walk(struct device *dev, const char *what, u64 iova)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);

	if (!pcie || !pcie->nvmmu.dart ||
	    apple_h9p_pcie_device_port(pcie, dev) != 0)
		return;

	apple_h9p_dart_walk(pcie, dev, what, "dart-side",
			    (u32)(iova - pcie->nvmmu.dart_base));
	if (pcie->nvmmu.dart_base)
		apple_h9p_dart_walk(pcie, dev, what, "raw", (u32)iova);
}
EXPORT_SYMBOL_GPL(apple_ans_dart_walk);

int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
			const u64 *pages, unsigned int npages,
			bool to_memory, dma_addr_t *iova)
{
	struct apple_h9p_nvmmu *nvmmu;
	struct apple_h9p_pcie *pcie;
	unsigned int i;
	u64 sgl_dma;
	u32 *tcb;
	u32 *sgl;
	int ret;

	if (tag >= APPLE_ANS_NVMMU_MAX_REQS ||
	    npages > APPLE_ANS_NVMMU_MAX_PAGES)
		return -EINVAL;
	if (npages && !pages)
		return -EINVAL;

	pcie = apple_ans_host(dev);
	if (!pcie) {
		dev_err_ratelimited(dev,
				    "NVMMU: tag %u host lookup failed; no hardware access\n", tag);
		return -ENODEV;
	}
	ret = apple_h9p_pcie_device_port(pcie, dev);
	if (ret != 0 || !(pcie->enabled_ports & BIT(0))) {
		dev_err_ratelimited(dev,
				    "NVMMU: tag %u owner port %d enabled %#x; no hardware access\n",
				    tag, ret, pcie->enabled_ports);
		return ret < 0 ? ret : -ENODEV;
	}

	nvmmu = &pcie->nvmmu;
	if (!nvmmu->ready)
		return -EOPNOTSUPP;
	if (READ_ONCE(nvmmu->poisoned))
		return -EIO;

	tcb = (u32 *)nvmmu->tcb + tag * H9P_NVMMU_TCB_DWORDS;
	sgl = (u32 *)nvmmu->tcb_sgl + tag * H9P_NVMMU_SGL_WORDS;

	if (!npages) {
		/* Retire the slot like iOS: count first, then the rest, then invalidate */
		WRITE_ONCE(tcb[H9P_NVMMU_TCB_NPAGES], 0);
		dma_wmb();
		memset(tcb, 0, H9P_NVMMU_TCB_BYTES);
		dma_wmb();
		if (iova)
			*iova = 0;
		/* Never-mapped slots aren't cached; don't feed the FIFO */
		if (!test_bit(tag, nvmmu->used))
			return 0;
		ret = apple_h9p_nvmmu_invalidate(pcie, tag);
		if (ret) {
			WRITE_ONCE(nvmmu->poisoned, true);
			return ret;
		}
		clear_bit(tag, nvmmu->used);
		return 0;
	}

	memset(tcb, 0, H9P_NVMMU_TCB_BYTES);
	memset(sgl, 0, H9P_NVMMU_SGL_WORDS * sizeof(*sgl));
	for (i = 0; i < npages; i++)
		sgl[i] = pages[i] >> ilog2(APPLE_ANS_NVMMU_PAGE_SIZE);

	/* iOS uses exactly one direction per TCB (InOut is rejected) */
	tcb[0] = to_memory ? H9P_NVMMU_TCB_WRITE : H9P_NVMMU_TCB_READ;
	tcb[2] = pages[0] >> ilog2(APPLE_ANS_NVMMU_PAGE_SIZE);
	sgl_dma = nvmmu->tcb_sgl_dma + tag * H9P_NVMMU_SGL_WORDS * sizeof(*sgl);
	memcpy(&tcb[4], &sgl_dma, sizeof(sgl_dma));
	/* The page count makes the TCB valid, so it goes last */
	dma_wmb();
	WRITE_ONCE(tcb[H9P_NVMMU_TCB_NPAGES], npages);
	dma_wmb();
	set_bit(tag, nvmmu->used);

	if (iova)
		*iova = H9P_NVMMU_FLATDMA_BASE + tag * H9P_NVMMU_FLATDMA_STRIDE;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_ans_nvmmu_map);

/* Caller holds the NVMe shutdown lock, RDY=0 and old queues are synchronized.
 * CC reset does not reset the host NVMMU. Retiring only software-used tags
 * cannot recover an invalidate failure or a retained hardware cache entry.
 */
int apple_ans_nvmmu_reset(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);
	struct apple_h9p_nvmmu *nvmmu;
	int ret;

	if (!pcie || apple_h9p_pcie_device_port(pcie, dev) != 0)
		return -ENODEV;
	nvmmu = &pcie->nvmmu;
	if (!nvmmu->ready || !(pcie->enabled_ports & BIT(0)))
		return -ENODEV;

	memset(nvmmu->tcb, 0, nvmmu->tcb_size);
	memset(nvmmu->tcb_table, 0, nvmmu->tcb_table_size);
	memset(nvmmu->tcb_sgl, 0, nvmmu->tcb_sgl_size);
	ret = apple_h9p_nvmmu_program(pcie);
	if (ret) {
		WRITE_ONCE(nvmmu->poisoned, true);
		return ret;
	}
	bitmap_zero(nvmmu->used, APPLE_ANS_NVMMU_MAX_REQS);
	WRITE_ONCE(nvmmu->poisoned, false);
	nvmmu->active = true;
	dev_info(dev, "NVMMU: stopped-controller cache/SART rebuilt\n");
	return 0;
}
EXPORT_SYMBOL_GPL(apple_ans_nvmmu_reset);

static bool apple_h9p_link_up(struct apple_h9p_pcie *pcie, unsigned int port)
{
	u32 linksts = readl(pcie->base_port[port] + H9P_PORT_LINKSTS);

	linksts = FIELD_GET(H9P_PORT_LINKSTS_LTSSM, linksts);
	return linksts >= H9P_PORT_LTSSM_DETECT && linksts <= H9P_PORT_LTSSM_L0;
}

static void apple_h9p_phy_ip_tune(struct apple_h9p_pcie *pcie,
				  unsigned int port)
{
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_COMMON0, 0, 0x4000);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_COMMON1, 0, 0x4000);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_TIME0, 0xfff, 100);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_TIME1, 0xfff, 25);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_EQ_CTL),
		0, 0x4000);
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_IDLE));
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_EQ_PRESET), 0xfff, 0x600);
	writel(0x3105, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL0));
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL1), 0xff, 0x9f);
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL2), 0xff, 0x01);
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL3), 0x1f, 0x0a);
	writel(175, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER0));
	writel(175, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER1));
	writel(333, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER2));
	writel(333, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER3));
	writel(530, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER4));
	writel(530, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER5));
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_CLEAR0));
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_CLEAR1));
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_CLEAR2));
}

static int apple_t8010_setup_port(struct apple_h9p_pcie *pcie,
				  unsigned int port)
{
	struct device *dev = pcie->dev;
	u32 speed = pcie->max_speed[port];
	u64 cap;
	int ret;

	/*
	 * PERST# was asserted when the GPIO was requested, so the endpoint is
	 * in reset even if the RC still reports a trained link: always run
	 * the full sequence rather than trusting the loader's link state.
	 */
	dev_info(dev, "port %u: bringing up link (Gen%u)\n", port, speed);
	if (apple_h9p_link_up(pcie, port))
		dev_info(dev, "port %u: link was up at probe, retraining\n",
			 port);

	ret = gpiod_direction_output(pcie->perst[port], 1);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to assert port %u PERST#\n", port);

	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL2(port), 1, 0);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(port), 0, 1);

	ret = apple_h9p_wait(pcie->base_rc + H9P_RC_COMMON_STAT,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE, 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u init timeout\n", port);

	usleep_range(250, 1000);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL0(port), 0, 1);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL0(port), 0x100, 0);
	usleep_range(500, 1000);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL2(port), 0, 1);

	/* T8010 wrote 3 for its Gen3 port and 0 for Gen1 ports (inferred) */
	writel(speed > H9P_LINK_SPEED_2_5GT ? speed : 0,
	       pcie->base_rc + H9P_RC_PORT_LINK_RATE(port));
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(port), 0x100, 0);

	cap = apple_h9p_read_pci_cap(pcie, port << 3, PCI_CAP_ID_EXP);
	if (cap)
		h9p_rmww(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
			 cap + PCI_EXP_LNKCTL2, PCI_EXP_LNKCTL2_TLS, speed);

	h9p_rmw(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
		H9P_CFG_PORT_MISC, 0, 1);

	writel(H9P_PORT_IRQMASK_PRE_LINK,
	       pcie->base_port[port] + H9P_PORT_IRQMASK);
	writel(H9P_PORT_IRQSTAT_PRE_LINK,
	       pcie->base_port[port] + H9P_PORT_IRQSTAT);

	h9p_rmw(pcie->base_port[port] + H9P_PORT_ENABLE, 0,
		H9P_PORT_ENABLE_APPLE);
	writel(H9P_PORT_PWRCTL_INIT, pcie->base_port[port] + H9P_PORT_PWRCTL);
	writel(port * 0x10001 * H9P_MSI_PER_PORT,
	       pcie->base_port[port] + H9P_PORT_MSIVECBASE);

	usleep_range(250, 1000);
	if (!pcie->clkreq[port])
		return dev_err_probe(dev, -EINVAL, "port %u has no CLKREQ#\n",
				     port);
	ret = apple_h9p_wait_gpio(pcie->clkreq[port], 1, 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u CLKREQ# timeout\n",
				     port);

	ret = gpiod_direction_output(pcie->perst[port], 0);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert port %u PERST#\n", port);
	usleep_range(250, 1000);

	ret = apple_h9p_wait(pcie->base_phy + H9P_PHY_PORTMASK,
			     BIT(port), BIT(port), BIT(port), 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u PHY up timeout\n",
				     port);

	if (pcie->base_phy_ip)
		apple_h9p_phy_ip_tune(pcie, port);

	writel(H9P_PORT_IRQMASK_LINK_UP,
	       pcie->base_port[port] + H9P_PORT_IRQMASK);
	usleep_range(5000, 10000);

	h9p_rmw(pcie->base_port[port] + H9P_PORT_LTSSMCTL, 0,
		H9P_PORT_LTSSM_ENABLE);
	ret = apple_h9p_wait(pcie->base_port[port] + H9P_PORT_LINKSTS,
			     H9P_PORT_LINKSTS_LTSSM,
			     FIELD_PREP(H9P_PORT_LINKSTS_LTSSM,
					H9P_PORT_LTSSM_DETECT),
			     FIELD_PREP(H9P_PORT_LINKSTS_LTSSM,
					H9P_PORT_LTSSM_L0),
			     500000);
	if (ret)
		return dev_err_probe(dev, ret,
				     "port %u link did not reach L0 (LINKSTS %#x)\n",
				     port, readl(pcie->base_port[port] +
						 H9P_PORT_LINKSTS));

	dev_info(dev, "port %u: link up (LINKSTS %#x)\n", port,
		 readl(pcie->base_port[port] + H9P_PORT_LINKSTS));
	return 0;
}

/* lane-cfg 1: x2 x1 x1 x1; lane-cfg 3 ("X2_X1_X2"): x2 x1 x2 - */
static const u8 s8003_lane_start[H9P_NUM_PORTS] = { 0, 2, 3, 4 };

static u32 apple_s8003_lane_mask(struct apple_h9p_pcie *pcie,
				 unsigned int port)
{
	unsigned int width = 1;

	if (port == 0 || (port == 2 && pcie->lane_cfg == 3))
		width = 2;

	return GENMASK(width - 1, 0) << s8003_lane_start[port];
}

/*
 * Apply an ADT-style tunable list. iOS reads apcie-phy-tunables etc. from the
 * live ADT (records {u32 offset, u32 size, u64 mask, u64 value}); here they
 * come from the DT as <offset mask value> u32 triplets copied from the j71t
 * live ADT. Like iOS, only registers whose value changes are written.
 */
static int apple_h9p_apply_tunables(struct apple_h9p_pcie *pcie,
				    struct device_node *np, const char *prop,
				    void __iomem *base, u32 size,
				    const char *what)
{
	struct device *dev = pcie->dev;
	int n = of_property_count_u32_elems(np, prop);
	unsigned int i, changed = 0;
	u32 off, mask, val, old, new;

	if (n <= 0)
		return 0;
	if (n % 3)
		return dev_err_probe(dev, -EINVAL, "%pOF: bad %s\n", np, prop);

	for (i = 0; i < n; i += 3) {
		of_property_read_u32_index(np, prop, i, &off);
		of_property_read_u32_index(np, prop, i + 1, &mask);
		of_property_read_u32_index(np, prop, i + 2, &val);
		if ((off & 3) || off > size - 4)
			return dev_err_probe(dev, -EINVAL,
					     "%pOF: %s offset %#x out of range\n",
					     np, prop, off);
		old = readl(base + off);
		new = (old & ~mask) | (val & mask);
		if (new != old) {
			writel(new, base + off);
			changed++;
		}
	}

	dev_info(dev, "%s tunables: %u of %d registers changed\n", what,
		 changed, n / 3);
	return 0;
}

static int apple_s8003_init_common(struct apple_h9p_pcie *pcie)
{
	int ret;

	writel(H9P_RC_COMMON_CTL_INIT |
	       FIELD_PREP(H9P_RC_COMMON_CTL_LANE_CFG, pcie->lane_cfg),
	       pcie->base_rc + S8003_RC_LANE_CFG);
	if (pcie->ans_mode == APPLE_ANS_ROUND7)
		h9p_rmw(pcie->base_rc + S8003_RC_PORT_PWR(0), 0,
			S8003_RC_PORT_PWR_ON);

	ret = apple_h9p_wait(pcie->base_rc + S8003_RC_STAT,
			     S8003_RC_STAT_COMMON_READY,
			     S8003_RC_STAT_COMMON_READY,
			     S8003_RC_STAT_COMMON_READY, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret, "common init timeout\n");

	writel(1, pcie->base_rc + S8003_RC_COMMON_DONE);

	/* iOS: apcie-phy-tunables, then apcie-common-tunables */
	ret = apple_h9p_apply_tunables(pcie, pcie->dev->of_node,
				       "apple,tunable-phy", pcie->base_phy,
				       H9P_PHY_SIZE, "PHY");
	if (ret)
		return ret;
	return apple_h9p_apply_tunables(pcie, pcie->dev->of_node,
					"apple,tunable-common", pcie->base_rc,
					H9P_RC_SIZE, "common");
}

static void apple_s8003_phy_ip_init(struct apple_h9p_pcie *pcie,
				    unsigned int port, u32 lane_mask, u32 speed)
{
	void __iomem *pip = pcie->base_phy_ip;
	unsigned long lanes = lane_mask;
	unsigned int lane;

	h9p_rmw(pip + H9P_PHY_IP_EQ_COMMON0, 0, 0x4000);
	h9p_rmw(pip + H9P_PHY_IP_EQ_COMMON1, 0, 0x4000);
	h9p_rmw(pip + H9P_PHY_IP_EQ_TIME0, 0xfff, 0x64);
	h9p_rmw(pip + H9P_PHY_IP_EQ_TIME1, 0xfff, 0x19);

	/* The 0x800 stride is per PHY lane, not per port */
	for_each_set_bit(lane, &lanes, BITS_PER_LONG) {
		void __iomem *b = pip + lane * H9P_PHY_IP_PORT_STRIDE;

		h9p_rmw(b + H9P_PHY_IP_PORT_EQ_CTL, 0, 0x4000);
		writel(0, b + H9P_PHY_IP_PORT_IDLE);
		h9p_rmw(b + H9P_PHY_IP_PORT_EQ_PRESET, 0xfff, 0x600);
		writel(0x3105, b + H9P_PHY_IP_PORT_RX_CTL0);
		h9p_rmw(b + H9P_PHY_IP_PORT_RX_CTL1, 0xff, 0x9f);
		h9p_rmw(b + H9P_PHY_IP_PORT_RX_CTL2, 0xff, 0x01);
		if (speed == H9P_LINK_SPEED_5GT)
			writel(0x100, b + 0x10188);
		if (speed <= H9P_LINK_SPEED_5GT)
			h9p_rmw(b + H9P_PHY_IP_PORT_RX_CTL3, 0x1f, 0x0a);
		writel(0xa9, b + H9P_PHY_IP_PORT_TIMER0);
		writel(0xa9, b + H9P_PHY_IP_PORT_TIMER1);
		writel(0x14d, b + H9P_PHY_IP_PORT_TIMER2);
		writel(0x14d, b + H9P_PHY_IP_PORT_TIMER3);
		writel(0, b + H9P_PHY_IP_PORT_CLEAR0);
		writel(0, b + H9P_PHY_IP_PORT_CLEAR1);
		writel(0, b + H9P_PHY_IP_PORT_CLEAR2);
	}
}

/* iOS port power-down: refclk off, then the port's power and enable */
static void apple_s8003_port_off(struct apple_h9p_pcie *pcie, unsigned int port)
{
	void __iomem *rc = pcie->base_rc;

	h9p_rmw(rc + S8003_RC_PORT_REFCLK(port), S8003_RC_PORT_REFCLK_ON, 0);
	udelay(S8003_PERST_TO_REFCLK_US);
	h9p_rmw(rc + S8003_RC_PORT_PWR(port), S8003_RC_PORT_PWR_ON, 0);
	h9p_rmw(rc + S8003_RC_PORT_CTL(port), S8003_RC_PORT_CTL_ENABLE, 0);
}

/* iOS port power/ready sequence; no port-block access until this succeeds. */
static int apple_s8003_port_power_on(struct apple_h9p_pcie *pcie,
				     unsigned int port)
{
	void __iomem *rc = pcie->base_rc;
	u32 val;
	int ret;

	dev_info(pcie->dev, "Apple ANS: STEP port %u power-on-refclk-off\n", port);
	h9p_rmw(rc + S8003_RC_PORT_REFCLK(port), S8003_RC_PORT_REFCLK_ON, 0);
	/* An old kernel may have allowed CLKREQ# gating: force the clock on. */
	dev_info(pcie->dev, "Apple ANS: STEP port %u power-on-request\n", port);
	h9p_rmw(rc + S8003_RC_PORT_PWR(port), 0,
		S8003_RC_PORT_PWR_ON | S8003_RC_PORT_REFCLK_NOGATE);
	dev_info(pcie->dev, "Apple ANS: STEP port %u power-ready-wait-250ms\n", port);
	ret = apple_h9p_wait(rc + S8003_RC_STAT, S8003_RC_STAT_PORT_READY,
			     S8003_RC_STAT_PORT_READY,
			     S8003_RC_STAT_PORT_READY, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret, "STEP port %u power-ready FAIL\n", port);
	dev_info(pcie->dev, "Apple ANS: STEP port %u power-settle-100us\n", port);
	udelay(100);
	dev_info(pcie->dev, "Apple ANS: STEP port %u port-enable\n", port);
	h9p_rmw(rc + S8003_RC_PORT_CTL(port), 0, S8003_RC_PORT_CTL_ENABLE);
	dev_info(pcie->dev, "Apple ANS: STEP port %u port-ready-wait-250ms\n", port);
	ret = readl_poll_timeout(rc + S8003_RC_PORT_READY(port), val, val & 1,
				 10, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret, "STEP port %u port-ready FAIL last=%#x\n",
				     port, val);
	dev_info(pcie->dev, "Apple ANS: STEP port %u port-reset-release\n", port);
	h9p_rmw(rc + S8003_RC_PORT_CTL(port), S8003_RC_PORT_CTL_BIT8, 0);
	dev_info(pcie->dev, "Apple ANS: STEP port %u refclk-on\n", port);
	h9p_rmw(rc + S8003_RC_PORT_REFCLK(port), 0, S8003_RC_PORT_REFCLK_ON);
	dev_info(pcie->dev, "Apple ANS: STEP port %u refclk-flush\n", port);
	readl(rc + S8003_RC_PORT_REFCLK(port));
	dev_info(pcie->dev, "Apple ANS: STEP port %u refclk-settle-100us\n", port);
	udelay(S8003_T_REFCLK_TO_PERST_US);
	return 0;
}

/* iOS name table at kernelcache 0xfffffff007a60538, not LINKSTS values. */
static const char *apple_s8003_ltssm_name(u32 state)
{
	static const char * const names[] = {
		"DETECT_QUIET", "DETECT_ACT", "POLL_ACTIVE", "POLL_COMPLIANCE",
		"POLL_CONFIG", "PRE_DETECT_QUIET", "DETECT_WAIT", "CFG_LINKWD_START",
		"CFG_LINKWD_ACEPT", "CFG_LANENUM_WAIT", "CFG_LANENUM_ACEPT",
		"CFG_COMPLETE", "CFG_IDLE", "RCVRY_LOCK", "RCVRY_SPEED",
		"RCVRY_RCVRCFG", "RCVRY_IDLE", "L0", "L0S", "L123_SEND_EIDLE",
		"L1_IDLE", "L2_IDLE", "L2_WAKE", "DISABLED_ENTRY", "DISABLED_IDLE",
		"DISABLED", "LPBK_ENTRY", "LPBK_ACTIVE", "LPBK_EXIT", "LPBK_EXIT_TIMEOUT",
		"HOT_RESET_ENTRY", "HOT_RESET", "RCVRY_EQ0", "RCVRY_EQ1",
		"RCVRY_EQ2", "RCVRY_EQ3",
	};

	return state < ARRAY_SIZE(names) ? names[state] : "UNKNOWN";
}

/* Only while the port registers are powered; never read an endpoint BAR. */
static void apple_s8003_port_dump(struct apple_h9p_pcie *pcie,
				  unsigned int port, const char *when)
{
	void __iomem *rc = pcie->base_rc;
	void __iomem *pb = pcie->base_port[port];
	u32 link = readl(pb + S8003_PORT_LINKSTS);
	u32 cdm = readl(pb + S8003_PORT_LINKCDMSTS);

	dev_info(pcie->dev,
		 "port %u %s: PERST %d ctl %#x pwr %#x refclk %#x ready %#x; LTSSMCTL %#x PME %#x link %#x LTSSM %#lx lanes %#x\n",
		 port, when, gpiod_get_value_cansleep(pcie->perst[port]),
		 readl(rc + S8003_RC_PORT_CTL(port)),
		 readl(rc + S8003_RC_PORT_PWR(port)),
		 readl(rc + S8003_RC_PORT_REFCLK(port)),
		 readl(rc + S8003_RC_PORT_READY(port)),
		 readl(pb + H9P_PORT_LTSSMCTL), readl(pb + S8003_PORT_PMETO),
		 link,
		 FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM,
			   cdm),
		 readl(pcie->base_phy + H9P_PHY_PORTMASK));
	dev_info(pcie->dev,
		 "Apple ANS: STEP port %u capture %s LINKSTS=%#x CDM=%#x LTSSM=%#lx/%s IRQ=%#x/%#x PMGR=%#x PHY=%#x RC2c=%#x\n",
		 port, when, link,
		 cdm,
		 FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM, cdm),
		 apple_s8003_ltssm_name(FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM, cdm)),
		 readl(pb + H9P_PORT_IRQSTAT), readl(pb + H9P_PORT_IRQMASK),
		 readl(pb + S8003_PORT_LINKPMGRSTS),
		 readl(pcie->base_phy + H9P_PHY_PORTMASK),
		 readl(pcie->base_rc + S8003_RC_STAT));
	if (pcie->link_cap[port]) {
		void __iomem *cfg = pcie->base_config + port * H9P_CFG_PORT_STRIDE;
		u16 cap = pcie->link_cap[port];

		dev_info(pcie->dev,
			 "Apple ANS: STEP port %u root-capture LNKCTL=%#x LNKCTL2=%#x LNKCAP2=%#x cfgA0=%#x cfg670=%#x cfg76c=%#x\n",
			 port, readw(cfg + cap + PCI_EXP_LNKCTL),
			 readw(cfg + cap + PCI_EXP_LNKCTL2), readl(cfg + cap + PCI_EXP_LNKCAP2),
			 readw(cfg + 0xa0), readl(cfg + 0x670), readl(cfg + 0x76c));
	}
}

/* Stop the retained LTSSM before changing clocks or releasing PERST#. */
static void apple_s8003_port_stop(struct apple_h9p_pcie *pcie, unsigned int port)
{
	void __iomem *pb = pcie->base_port[port];

	writel(H9P_PORT_IRQMASK_PRE_LINK, pb + H9P_PORT_IRQMASK);
	h9p_rmw(pb + H9P_PORT_LTSSMCTL, H9P_PORT_LTSSM_ENABLE, 0);
	readl(pb + H9P_PORT_LTSSMCTL);
	/* Cancel the previous kernel's PME_Turn_Off/L2 request. */
	h9p_writel_flush(0, pb + S8003_PORT_PMETO);
	writel(H9P_PORT_IRQSTAT_PRE_LINK, pb + H9P_PORT_IRQSTAT);
}

/* AppleEmbeddedPCIEPort::enableGated + AppleS800xPCIePort::enablePortHardware */
static int apple_s8003_setup_port_round7(struct apple_h9p_pcie *pcie,
					 unsigned int port)
{
	void __iomem *rc = pcie->base_rc;
	void __iomem *pb = pcie->base_port[port];
	struct device *dev = pcie->dev;
	u32 lanes = apple_s8003_lane_mask(pcie, port);
	u32 speed = pcie->max_speed[port];
	u16 lnkcap2_speeds;
	u64 cap;
	u32 val;
	int ret;

	if (pcie->lane_cfg == 3 && port == 3)
		return dev_err_probe(dev, -EINVAL,
				     "port 3 is unusable with lane-cfg 3\n");

	/* No port-block access before the port is powered (iOS doesn't) */
	dev_info(dev, "port %u: bringing up link (Gen%u, lanes %#x)\n", port,
		 speed, lanes);

	/* The endpoint stays in reset (PERST# was asserted at GPIO request) */
	ret = gpiod_direction_output(pcie->perst[port], 1);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to assert port %u PERST#\n", port);

	/*
	 * A kexec can leave the port off, in L2, or training in compliance.
	 * First power its register block, stop the old state machine, then
	 * make a real off/on edge. Reapplying tunables alone does not reset it.
	 * Do not reset the shared PHY/PMGR: other ports and DARTs share it.
	 */
	if (pcie->nvme_mode[port]) {
		writel(0x1, rc + S8003_RC_NVME_MODE0);
		udelay(100);
		h9p_writel_flush(1, rc + S8003_RC_NVME_MODE1);
	}
	ret = apple_s8003_port_power_on(pcie, port);
	if (ret)
		return dev_err_probe(dev, ret, "port %u initial power timeout\n", port);
	apple_s8003_port_dump(pcie, port, "inherited");
	apple_s8003_port_stop(pcie, port);
	if (!port && pcie->nvmmu.base &&
	    of_machine_is_compatible("apple,j71t")) {
		dev_info(dev, "ANS inherited NVMMU: SART %#x TCB %#x:%08x; retiring under PERST\n",
			 readl(pcie->nvmmu.base + H9P_NVMMU_SART_CTRL),
			 readl(pcie->nvmmu.base + H9P_NVMMU_TCB_BASE_HI),
			 readl(pcie->nvmmu.base + H9P_NVMMU_TCB_BASE_LO));
		h9p_writel_flush(0, pcie->nvmmu.base + H9P_NVMMU_SART_CTRL);
	}
	apple_s8003_port_off(pcie, port);
	if (pcie->nvme_mode[port])
		h9p_writel_flush(0, rc + S8003_RC_NVME_MODE1);
	usleep_range(1000, 2000);
	if (pcie->nvme_mode[port]) {
		writel(0x1, rc + S8003_RC_NVME_MODE0);
		udelay(100);
		h9p_writel_flush(1, rc + S8003_RC_NVME_MODE1);
	}
	ret = apple_s8003_port_power_on(pcie, port);
	if (ret)
		return dev_err_probe(dev, ret, "port %u restart power timeout\n", port);
	apple_s8003_port_stop(pcie, port);
	writel(3, rc + S8003_RC_PORT_LSCNT(port));
	val = readl(pb + S8003_PORT_LINKPMGRSTS);
	if (val)
		writel(val, pb + S8003_PORT_LINKPMGRSTS);

	cap = apple_h9p_read_pci_cap(pcie, port << 3, PCI_CAP_ID_EXP);
	if (!cap)
		return dev_err_probe(dev, -ENODEV,
				     "port %u root port has no PCIe capability\n",
				     port);
	/* Clamp to LNKCAP2 like iOS setMaximumLinkSpeed */
	lnkcap2_speeds = (readl(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
				cap + PCI_EXP_LNKCAP2) >> 1) & 0x7f;
	while (speed > H9P_LINK_SPEED_2_5GT &&
	       !(lnkcap2_speeds & BIT(speed - 1)))
		speed--;
	dev_info(dev, "port %u: restart LNKCTL %#x LNKCTL2 %#x\n", port,
		 readw(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		       PCI_EXP_LNKCTL),
		 readw(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		       PCI_EXP_LNKCTL2));
	h9p_rmww(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		 PCI_EXP_LNKCTL, PCI_EXP_LNKCTL_ASPMC | PCI_EXP_LNKCTL_LD |
		 PCI_EXP_LNKCTL_RL, 0);
	h9p_rmww(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		 PCI_EXP_LNKCTL2, PCI_EXP_LNKCTL2_TLS | PCI_EXP_LNKCTL2_ENTER_COMP,
		 speed);

	/* iOS: pcie-rc-tunables (root port config), apcie-config-tunables */
	ret = apple_h9p_apply_tunables(pcie, pcie->port_nodes[port],
				       "apple,tunable-rc",
				       pcie->base_config +
				       port * H9P_CFG_PORT_STRIDE,
				       H9P_CFG_SIZE, "root port");
	if (ret)
		return ret;
	ret = apple_h9p_apply_tunables(pcie, pcie->port_nodes[port],
				       "apple,tunable-config", pb,
				       H9P_PORT_SIZE, "port");
	if (ret)
		return ret;

	writel(H9P_PORT_IRQMASK_PRE_LINK, pb + H9P_PORT_IRQMASK);
	writel(H9P_PORT_IRQSTAT_PRE_LINK, pb + H9P_PORT_IRQSTAT);
	h9p_rmw(pb + H9P_PORT_ENABLE, 0, H9P_PORT_ENABLE_APPLE);
	writel(1 | ilog2(H9P_MSI_PER_PORT) << 4, pb + S8003_PORT_MSICTL);
	writel(port * 0x10001 * H9P_MSI_PER_PORT, pb + H9P_PORT_MSIVECBASE);

	/* Clock is stable and reset is held even when GPIO ownership survived. */
	msleep(S8003_PORT_RESET_MS);
	ret = gpiod_direction_output(pcie->perst[port], 0);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert port %u PERST#\n", port);

	ret = apple_h9p_wait(pcie->base_phy + H9P_PHY_PORTMASK, lanes, lanes,
			     lanes, 250000);
	if (ret)
		return dev_err_probe(dev, ret,
				     "port %u PHY lanes %#x not ready (%#x)\n",
				     port, lanes,
				     readl(pcie->base_phy + H9P_PHY_PORTMASK));

	apple_s8003_phy_ip_init(pcie, port, lanes, speed);

	writel(H9P_PORT_IRQMASK_LINK_UP, pb + H9P_PORT_IRQMASK);
	h9p_rmw(pb + H9P_PORT_LTSSMCTL, 0, H9P_PORT_LTSSM_ENABLE);

	ret = readl_poll_timeout(pb + S8003_PORT_LINKSTS, val,
				 val & S8003_PORT_LINKSTS_UP, 1000, 500000);
	val = FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM,
			readl(pb + S8003_PORT_LINKCDMSTS));
	if (ret) {
		apple_s8003_port_dump(pcie, port, "training timeout");
		return dev_err_probe(dev, ret,
				     "port %u link did not come up (LTSSM %#x)\n",
				     port, val);
	}

	/* Link-up is reported during the Gen2 recovery; let it settle in L0 */
	ret = readl_poll_timeout(pb + S8003_PORT_LINKCDMSTS, val,
				 FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM, val) ==
				 S8003_LTSSM_L0, 100, 100000);
	if (ret || !(readl(pb + S8003_PORT_LINKSTS) & S8003_PORT_LINKSTS_UP)) {
		apple_s8003_port_dump(pcie, port, "L0 timeout");
		return -ETIMEDOUT;
	}
	dev_info(dev, "port %u: link up (LTSSM %#lx)\n", port,
		 FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM, val));
	return 0;
}

/* A timeout cannot cancel a stalled bus transaction. Keep these accesses
 * host-side, with the register block powered, and log before every phase.
 * A returning failure drops the port; the early-USB initramfs reaches rescue.
 */
/* Keep the 100 us link poll and its original deadline; sample every 20 ms.
 * Called only by LINKSTS waits after power/ready and capability setup.
 */
static bool apple_s8003_link_sample(struct apple_h9p_pcie *pcie,
				    unsigned int port, u64 *next)
{
	u64 now = ktime_get_ns();

	if (now >= *next) {
		apple_s8003_port_dump(pcie, port, "training-sample");
		*next = now + 20000000;
	}
	return true;
}

static int apple_s8003_port_wait(struct apple_h9p_pcie *pcie, unsigned int port,
				 void __iomem *addr, u32 mask, u32 expected,
				 unsigned int timeout_us, const char *step)
{
	u32 val;
	u64 next = 0;
	bool link = addr == pcie->base_port[port] + S8003_PORT_LINKSTS;
	int ret;

	dev_info(pcie->dev, "Apple ANS: STEP port %u %s wait timeout_us=%u\n",
		 port, step, timeout_us);
	ret = readl_poll_timeout(addr, val, (!link || apple_s8003_link_sample(pcie, port, &next)) &&
				 (val & mask) == expected,
				 100, timeout_us);
	if (ret)
		dev_err(pcie->dev, "Apple ANS: STEP port %u %s FAIL %d last=%#x mask=%#x expected=%#x\n",
			port, step, ret, val, mask, expected);
	else
		dev_info(pcie->dev, "Apple ANS: STEP port %u %s done last=%#x\n",
			 port, step, val);
	return ret;
}

static int apple_s8003_link_finish(struct apple_h9p_pcie *pcie,
				   unsigned int port, unsigned int timeout_us)
{
	void __iomem *pb = pcie->base_port[port];
	struct device *dev = pcie->dev;
	u32 val;
	int ret, sample;

	ret = apple_s8003_port_wait(pcie, port, pb + S8003_PORT_LINKSTS,
				    S8003_PORT_LINKSTS_UP, S8003_PORT_LINKSTS_UP,
				    timeout_us, "link-up");
	if (ret) {
		apple_s8003_port_dump(pcie, port, "link-up-failed");
		return ret;
	}
	ret = apple_s8003_port_wait(pcie, port, pb + S8003_PORT_LINKCDMSTS,
				    S8003_PORT_LINKCDMSTS_LTSSM,
				    S8003_LTSSM_L0 << 9, 100000, "link-L0");
	if (ret) {
		apple_s8003_port_dump(pcie, port, "link-L0-failed");
		return ret;
	}
	dev_info(dev, "Apple ANS: STEP port %u link-up-confirm\n", port);
	if (!(readl(pb + S8003_PORT_LINKSTS) & S8003_PORT_LINKSTS_UP)) {
		apple_s8003_port_dump(pcie, port, "link-confirm-failed");
		return dev_err_probe(dev, -ENOLINK, "STEP port %u link lost after L0\n", port);
	}
	if (!port && pcie->ans_deep_recovery) {
		/* One transient UP/L0 sample cannot authorize config/BAR traffic. */
		dev_info(dev, "Apple ANS: STEP port 0 deep-stable wait-100ms\n");
		for (sample = 0; sample < 100; sample++) {
			usleep_range(1000, 1100);
			val = readl(pb + S8003_PORT_LINKCDMSTS);
			if (!(readl(pb + S8003_PORT_LINKSTS) & S8003_PORT_LINKSTS_UP) ||
			    FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM, val) != S8003_LTSSM_L0) {
				apple_s8003_port_dump(pcie, port, "deep-stable-failed");
				return dev_err_probe(dev, -ENOLINK,
						     "STEP port 0 deep-stable FAIL sample=%d\n",
						     sample);
			}
		}
		dev_info(dev, "Apple ANS: STEP port 0 deep-stable done\n");
	}
	val = readl(pb + S8003_PORT_LINKCDMSTS);
	dev_info(dev, "Apple ANS: STEP port %u full-bringup-done\n", port);
	dev_info(dev, "port %u: link up (LTSSM %#lx)\n", port,
		 FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM, val));
	return 0;
}

static int apple_s8003_setup_port_reset(struct apple_h9p_pcie *pcie,
					unsigned int port, bool keep_power)
{
	void __iomem *rc = pcie->base_rc;
	void __iomem *pb = pcie->base_port[port];
	struct device *dev = pcie->dev;
	u32 lanes = apple_s8003_lane_mask(pcie, port);
	u32 speed = !port && pcie->ans_deep_recovery ?
		H9P_LINK_SPEED_2_5GT : pcie->max_speed[port];
	u16 lnkcap2_speeds;
	u64 cap;
	u32 val;
	int ret;

	if (pcie->lane_cfg == 3 && port == 3)
		return dev_err_probe(dev, -EINVAL,
				     "port 3 is unusable with lane-cfg 3\n");

	/* No port-block access before the port is powered (iOS doesn't) */
	dev_info(dev, "port %u: bringing up link (Gen%u, lanes %#x)\n", port,
		 speed, lanes);

	dev_info(dev, "Apple ANS: STEP port %u perst-assert\n", port);
	/* The endpoint stays in reset (PERST# was asserted at GPIO request) */
	ret = gpiod_direction_output(pcie->perst[port], 1);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to assert port %u PERST#\n", port);

	/* disablePortHardware: force_active(false) records inactive. No live
	 * tables are zeroed; reactivation is mandatory in endpoint preinit.
	 * rc+124 reset, 60us, rc+114 refclk, rc+100 power (not the legacy
	 * helper's misleading CTL/PWR/REFCLK names).
	 */
	dev_info(dev, "Apple ANS: STEP port %u inherited-power-check\n", port);
	if ((readl(rc + S8003_RC_PORT_CTL(port)) & S8003_RC_PORT_CTL_ENABLE) &&
	    (readl(rc + S8003_RC_PORT_PWR(port)) & S8003_RC_PORT_PWR_ON)) {
		dev_info(dev, "Apple ANS: STEP port %u inherited-stop\n", port);
		apple_s8003_port_stop(pcie, port);
		ret = apple_s8003_port_wait(pcie, port, pb + H9P_PORT_LTSSMCTL,
					    H9P_PORT_LTSSM_ENABLE, 0, 250000,
					    "inherited-LTSSM-stop");
		if (ret)
			return ret;
	}
	if (keep_power) {
		/* PERST and the stopped LTSSM isolate the endpoint. Keep the
		 * already stable clock instead of making another off/on edge.
		 * Controller queues and DMA translations are rebuilt as usual.
		 */
		dev_info(dev, "Apple ANS: STEP port 0 reset with power/refclk retained\n");
		h9p_rmw(rc + S8003_RC_PORT_PWR(port), 0,
			S8003_RC_PORT_REFCLK_NOGATE);
		readl(rc + S8003_RC_PORT_PWR(port));
	} else {
		dev_info(dev, "Apple ANS: STEP port %u refclk-off\n", port);
		h9p_rmw(rc + S8003_RC_PORT_REFCLK(port), S8003_RC_PORT_REFCLK_ON, 0);
		dev_info(dev, "Apple ANS: STEP port %u perst-to-refclk-delay-60us\n", port);
		udelay(S8003_PERST_TO_REFCLK_US);
		dev_info(dev, "Apple ANS: STEP port %u power-off\n", port);
		h9p_rmw(rc + S8003_RC_PORT_PWR(port), S8003_RC_PORT_PWR_ON, 0);
		dev_info(dev, "Apple ANS: STEP port %u port-disable\n", port);
		h9p_rmw(rc + S8003_RC_PORT_CTL(port), S8003_RC_PORT_CTL_ENABLE, 0);
		dev_info(dev, "Apple ANS: STEP port %u power-off-settle-10ms\n", port);
		usleep_range(10000, 11000);
		ret = apple_s8003_port_wait(pcie, port, rc + S8003_RC_PORT_CTL(port),
					    S8003_RC_PORT_CTL_ENABLE, 0, 250000, "port-off");
		if (ret)
			return ret;
		if (pcie->nvme_mode[port]) {
			dev_info(dev, "Apple ANS: STEP port %u nvme-mode0\n", port);
			writel(1, rc + S8003_RC_NVME_MODE0);
			usleep_range(100, 200);
			dev_info(dev, "Apple ANS: STEP port %u nvme-mode1\n", port);
			h9p_writel_flush(1, rc + S8003_RC_NVME_MODE1);
		}
		ret = apple_s8003_port_power_on(pcie, port);
		if (ret)
			return dev_err_probe(dev, ret, "port %u full-cycle power timeout\n", port);
	}
	dev_info(dev, "Apple ANS: STEP port %u powered-stop\n", port);
	apple_s8003_port_stop(pcie, port);
	ret = apple_s8003_port_wait(pcie, port, pb + H9P_PORT_LTSSMCTL,
				    H9P_PORT_LTSSM_ENABLE, 0, 250000, "powered-LTSSM-stop");
	if (ret)
		return ret;
	dev_info(dev, "Apple ANS: STEP port %u link-counters\n", port);
	writel(3, rc + S8003_RC_PORT_LSCNT(port));
	dev_info(dev, "Apple ANS: STEP port %u link-pmgr-ack\n", port);
	val = readl(pb + S8003_PORT_LINKPMGRSTS);
	if (val)
		writel(val, pb + S8003_PORT_LINKPMGRSTS);

	dev_info(dev, "Apple ANS: STEP port %u root-capability-scan-max48\n", port);
	cap = apple_h9p_read_pci_cap(pcie, port << 3, PCI_CAP_ID_EXP);
	if (!cap)
		return dev_err_probe(dev, -ENODEV,
				     "port %u root port has no PCIe capability\n",
				     port);
	pcie->link_cap[port] = cap;
	/* Clamp to LNKCAP2 like iOS setMaximumLinkSpeed */
	dev_info(dev, "Apple ANS: STEP port %u link-speed-config\n", port);
	lnkcap2_speeds = (readl(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
				cap + PCI_EXP_LNKCAP2) >> 1) & 0x7f;
	while (speed > H9P_LINK_SPEED_2_5GT &&
	       !(lnkcap2_speeds & BIT(speed - 1)))
		speed--;
	dev_info(dev, "port %u: restart LNKCTL %#x LNKCTL2 %#x\n", port,
		 readw(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		       PCI_EXP_LNKCTL),
		 readw(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		       PCI_EXP_LNKCTL2));
	h9p_rmww(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		 PCI_EXP_LNKCTL, PCI_EXP_LNKCTL_ASPMC | PCI_EXP_LNKCTL_LD |
		 PCI_EXP_LNKCTL_RL, 0);
	h9p_rmww(pcie->base_config + port * H9P_CFG_PORT_STRIDE + cap +
		 PCI_EXP_LNKCTL2, PCI_EXP_LNKCTL2_TLS | PCI_EXP_LNKCTL2_ENTER_COMP,
		 speed);

	/* iOS: pcie-rc-tunables (root port config), apcie-config-tunables */
	dev_info(dev, "Apple ANS: STEP port %u root-tunables\n", port);
	ret = apple_h9p_apply_tunables(pcie, pcie->port_nodes[port],
				       "apple,tunable-rc",
				       pcie->base_config +
				       port * H9P_CFG_PORT_STRIDE,
				       H9P_CFG_SIZE, "root port");
	if (ret)
		return ret;
	dev_info(dev, "Apple ANS: STEP port %u port-tunables\n", port);
	ret = apple_h9p_apply_tunables(pcie, pcie->port_nodes[port],
				       "apple,tunable-config", pb,
				       H9P_PORT_SIZE, "port");
	if (ret)
		return ret;

	dev_info(dev, "Apple ANS: STEP port %u irq-mask\n", port);
	writel(H9P_PORT_IRQMASK_PRE_LINK, pb + H9P_PORT_IRQMASK);
	dev_info(dev, "Apple ANS: STEP port %u irq-ack\n", port);
	writel(H9P_PORT_IRQSTAT_PRE_LINK, pb + H9P_PORT_IRQSTAT);
	dev_info(dev, "Apple ANS: STEP port %u apple-port-enable\n", port);
	h9p_rmw(pb + H9P_PORT_ENABLE, 0, H9P_PORT_ENABLE_APPLE);
	dev_info(dev, "Apple ANS: STEP port %u msi-control\n", port);
	writel(1 | ilog2(H9P_MSI_PER_PORT) << 4, pb + S8003_PORT_MSICTL);
	dev_info(dev, "Apple ANS: STEP port %u msi-vector-base\n", port);
	writel(port * 0x10001 * H9P_MSI_PER_PORT, pb + H9P_PORT_MSIVECBASE);

	/* Tunables may include link controls. Enforce the recovery speed LAST. */
	if (!port && pcie->ans_deep_recovery) {
		h9p_rmww(pcie->base_config + cap + PCI_EXP_LNKCTL2,
			 PCI_EXP_LNKCTL2_TLS | PCI_EXP_LNKCTL2_ENTER_COMP,
			 H9P_LINK_SPEED_2_5GT);
		val = readw(pcie->base_config + cap + PCI_EXP_LNKCTL2);
		dev_info(dev, "Apple ANS: STEP port 0 deep-Gen1 LNKCTL2=%#x lanes=%#x\n",
			 val, lanes);
		if ((val & PCI_EXP_LNKCTL2_TLS) != H9P_LINK_SPEED_2_5GT)
			return -EIO;
	}

	/* Clock is stable and reset is held even when GPIO ownership survived. */
	dev_info(dev, "Apple ANS: STEP port %u perst-hold-100ms\n", port);
	msleep(S8003_PORT_RESET_MS);
	dev_info(dev, "Apple ANS: STEP port %u perst-release\n", port);
	ret = gpiod_direction_output(pcie->perst[port], 0);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert port %u PERST#\n", port);

	ret = apple_s8003_port_wait(pcie, port, pcie->base_phy + H9P_PHY_PORTMASK,
				    lanes, lanes, 250000, "phy-lanes-ready");
	if (ret)
		return ret;

	dev_info(dev, "Apple ANS: STEP port %u phy-ip-init\n", port);
	apple_s8003_phy_ip_init(pcie, port, lanes, speed);

	dev_info(dev, "Apple ANS: STEP port %u link-irq-mask\n", port);
	writel(H9P_PORT_IRQMASK_LINK_UP, pb + H9P_PORT_IRQMASK);
	dev_info(dev, "Apple ANS: STEP port %u ltssm-start\n", port);
	h9p_rmw(pb + H9P_PORT_LTSSMCTL, 0, H9P_PORT_LTSSM_ENABLE);

	return apple_s8003_link_finish(pcie, port, 500000);
}

/* Recovery and cold boots always retain the complete off/on sequence. */
static int apple_s8003_setup_port_full(struct apple_h9p_pcie *pcie,
				       unsigned int port)
{
	return apple_s8003_setup_port_reset(pcie, port, false);
}

static int apple_s8003_setup_port_once(struct apple_h9p_pcie *pcie,
				       unsigned int port)
{
	u32 ctl, pwr, reset;

	if (port || pcie->ans_mode == APPLE_ANS_ROUND7)
		return apple_s8003_setup_port_round7(pcie, port);
	/* L0 proves link training, not ownership of inherited DMA state.
	 * Both paths assert PERST before installing new translations.
	 * The outgoing drain/SHST/CC-disable/BME contract is mandatory.
	 */
	dev_info(pcie->dev, "Apple ANS: %s full port bring-up\n",
		 ans_boot_modes[pcie->ans_mode]);
	ctl = readl(pcie->base_rc + S8003_RC_PORT_CTL(0));
	pwr = readl(pcie->base_rc + S8003_RC_PORT_PWR(0));
	reset = readl(pcie->base_rc + S8003_RC_PORT_REFCLK(0));
	dev_info(pcie->dev, "Apple ANS: incoming port 0 RC124=%#x RC114=%#x RC100=%#x; policy=%s\n",
		 reset, pwr, ctl, ans_keep_power && pcie->ans_mode == APPLE_ANS_HANDOFF ?
		 "powered-reset-trial/full-fallback" : "full-down/full-bringup");
	/* Accept only the opt-in, powered, reset-held warm handoff. A cold
	 * boot, old outgoing kernel, or incomplete certificate uses full reset.
	 * No live-link/old-queue shortcut: setup still holds PERST for 100 ms,
	 * replays port/PHY/MSI setup and requires fresh UP/L0 before enumeration.
	 */
	if (ans_keep_power && pcie->ans_mode == APPLE_ANS_HANDOFF &&
	    of_property_read_bool(of_chosen, "linux,booted-from-kexec") &&
	    (ctl & S8003_RC_PORT_CTL_ENABLE) &&
	    !(ctl & S8003_RC_PORT_CTL_BIT8) &&
	    (pwr & S8003_RC_PORT_PWR_ON) &&
	    (pwr & S8003_RC_PORT_REFCLK_NOGATE) &&
	    (reset & S8003_RC_PORT_REFCLK_ON) &&
	    (readl(pcie->base_rc + S8003_RC_PORT_READY(0)) & BIT(0)) &&
	    gpiod_get_value_cansleep(pcie->perst[0]) == 1) {
		int ret;

		ret = apple_s8003_setup_port_reset(pcie, port, true);
		if (!ret || (ret != -ETIMEDOUT && ret != -ENOLINK))
			return ret;
		dev_warn(pcie->dev,
			 "Apple ANS: powered reset failed %d; falling back to full port cycle\n",
			 ret);
	}
	return apple_s8003_setup_port_full(pcie, port);
}

/* Queues synchronized and bus mastering off. Exactly one caller owns the
 * recovery quota; never reset a shared PMGR domain or the Wi-Fi port.
 */
int apple_ans_restart_port(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_ans_host(dev);
	int ret;

	if (!pcie || pcie->ans_mode == APPLE_ANS_ROUND7)
		return -EOPNOTSUPP;
	pcie->ans_mode = pcie->ans_mode == APPLE_ANS_HANDOFF ?
		APPLE_ANS_FULL_CYCLE : APPLE_ANS_HANDOFF;
	dev_warn(dev, "Apple ANS: one recovery using %s bring-up\n",
		 ans_boot_modes[pcie->ans_mode]);
	/* Both trial modes cycle the port, so both must rearm translations. */
	ret = apple_ans_dart_set_active(dev, false);
	if (ret)
		return ret;
	pcie->nvmmu.active = false;
	/* Both modes normalize the port before the next controller enable. */
	ret = apple_s8003_setup_port_once(pcie, 0);
	if (ret)
		WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(0));
	return ret;
}
EXPORT_SYMBOL_GPL(apple_ans_restart_port);

/* Both initial NVMe and late Wi-Fi ports recover from retained state. */
static int apple_s8003_setup_port(struct apple_h9p_pcie *pcie, unsigned int port)
{
	unsigned int attempt;
	int ret;

	for (attempt = 0; attempt < S8003_PORT_LINK_TRIES; attempt++) {
		ret = apple_s8003_setup_port_once(pcie, port);
		if (!ret)
			return 0;
		gpiod_direction_output(pcie->perst[port], 1);
		if (ret != -ETIMEDOUT)
			break;
		if (attempt + 1 < S8003_PORT_LINK_TRIES)
			dev_warn(pcie->dev, "port %u: retrying with port reset\n", port);
	}
	return ret;
}

/* A9 has an external PCIe S3E, not an AP-visible ANS2/RTKit CPU.
 * No ps_ans or endpoint PWREN exists in this DT. This resets only the
 * port/endpoint PCIe interface: do not claim it removes the S3E NAND rail.
 * PERST stays asserted throughout a 1 s refclk/port-off hold. Only port 0
 * changes; shared RC/PHY/PMGR and Wi-Fi state must survive.
 */
static int apple_s8003_deep_cycle(struct apple_h9p_pcie *pcie)
{
	void __iomem *rc = pcie->base_rc;
	u32 ctl, pwr, reset;
	int ret;

	WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(0));
	dev_warn(pcie->dev, "Apple ANS: STEP port 0 deep-recovery begin; S3E rail/firmware reset unavailable\n");
	ret = gpiod_direction_output(pcie->perst[0], 1);
	if (ret)
		return ret;
	if ((readl(rc + S8003_RC_PORT_CTL(0)) & S8003_RC_PORT_CTL_ENABLE) &&
	    (readl(rc + S8003_RC_PORT_PWR(0)) & S8003_RC_PORT_PWR_ON)) {
		apple_s8003_port_dump(pcie, 0, "deep-before-stop");
		apple_s8003_port_stop(pcie, 0);
		ret = apple_s8003_port_wait(pcie, 0,
					    pcie->base_port[0] + H9P_PORT_LTSSMCTL,
					    H9P_PORT_LTSSM_ENABLE, 0, 250000,
					    "deep-stop");
		if (ret)
			return ret;
	}
	apple_s8003_port_off(pcie, 0);
	ctl = readl(rc + S8003_RC_PORT_CTL(0));
	pwr = readl(rc + S8003_RC_PORT_PWR(0));
	reset = readl(rc + S8003_RC_PORT_REFCLK(0));
	dev_info(pcie->dev, "Apple ANS: STEP port 0 deep-off RC124=%#x RC114=%#x RC100=%#x hold-1000ms\n",
		 reset, pwr, ctl);
	if ((ctl | pwr | reset) & BIT(0))
		return -EIO;
	msleep(1000);
	pcie->ans_deep_recovery = true;
	ret = apple_s8003_setup_port_full(pcie, 0);
	pcie->ans_deep_recovery = false;
	if (ret) {
		gpiod_direction_output(pcie->perst[0], 1);
		if ((readl(rc + S8003_RC_PORT_CTL(0)) & S8003_RC_PORT_CTL_ENABLE) &&
		    (readl(rc + S8003_RC_PORT_PWR(0)) & S8003_RC_PORT_PWR_ON) &&
		    (readl(rc + S8003_RC_PORT_READY(0)) & BIT(0))) {
			apple_s8003_port_stop(pcie, 0);
			if (apple_s8003_port_wait(pcie, 0,
						  pcie->base_port[0] + H9P_PORT_LTSSMCTL,
						  H9P_PORT_LTSSM_ENABLE, 0, 250000,
						  "deep-failed-stop"))
				return ret;
		}
		/* No powered-off port or endpoint readback. */
		apple_s8003_port_off(pcie, 0);
		dev_err(pcie->dev, "Apple ANS: STEP port 0 deep-recovery FAIL %d; PERST asserted, port off\n",
			ret);
	} else {
		pcie->max_speed[0] = H9P_LINK_SPEED_2_5GT;
		pcie->ans_gen1 = true;
		dev_info(pcie->dev, "Apple ANS: STEP port 0 deep-recovery done; Gen1 retained\n");
	}
	return ret;
}

/* No power toggle: restart LTSSM, optionally re-run lane init at Gen1,
 * then issue the standard RW1S LNKCTL.RL trigger. Never poll RL for one.
 */
static int apple_s8003_retrain(struct apple_h9p_pcie *pcie, bool gen1)
{
	void __iomem *pb = pcie->base_port[0];
	void __iomem *rc = pcie->base_rc;
	u16 cap = pcie->link_cap[0];
	u32 state;
	int ret;

	if (!cap || gpiod_get_value_cansleep(pcie->perst[0]) ||
	    !(readl(rc + S8003_RC_PORT_CTL(0)) & BIT(0)) ||
	    !(readl(rc + S8003_RC_PORT_PWR(0)) & BIT(0)) ||
	    !(readl(rc + S8003_RC_PORT_READY(0)) & BIT(0)))
		return -ENOLINK;
	state = FIELD_GET(S8003_PORT_LINKCDMSTS_LTSSM,
			  readl(pb + S8003_PORT_LINKCDMSTS));
	if (state >= 0x17 && state <= 0x19) {
		dev_info(pcie->dev, "Apple ANS: STEP port 0 retrain SKIP disabled LTSSM=%#x\n",
			 state);
		return -ENOLINK;
	}
	dev_info(pcie->dev, "Apple ANS: STEP port 0 retrain-%s begin\n", gen1 ? "Gen1" : "current");
	apple_s8003_port_dump(pcie, 0, "before-retrain");
	apple_s8003_port_stop(pcie, 0);
	ret = apple_s8003_port_wait(pcie, 0, pb + H9P_PORT_LTSSMCTL,
				    H9P_PORT_LTSSM_ENABLE, 0, 20000, "retrain-stop");
	if (ret)
		return ret;
	if (gen1) {
		h9p_rmww(pcie->base_config + cap + PCI_EXP_LNKCTL2,
			 PCI_EXP_LNKCTL2_TLS | PCI_EXP_LNKCTL2_ENTER_COMP,
			 H9P_LINK_SPEED_2_5GT);
		apple_s8003_phy_ip_init(pcie, 0, apple_s8003_lane_mask(pcie, 0),
					H9P_LINK_SPEED_2_5GT);
	}
	h9p_rmw(pb + H9P_PORT_LTSSMCTL, 0, H9P_PORT_LTSSM_ENABLE);
	h9p_rmww(pcie->base_config + cap + PCI_EXP_LNKCTL,
		 PCI_EXP_LNKCTL_LD, PCI_EXP_LNKCTL_RL);
	pcie->ans_deep_recovery = true;
	ret = apple_s8003_link_finish(pcie, 0, 100000);
	pcie->ans_deep_recovery = false;
	if (!ret && gen1) {
		pcie->ans_gen1 = true;
		pcie->max_speed[0] = H9P_LINK_SPEED_2_5GT;
	}
	return ret;
}

static int apple_s8003_replay_phy(struct apple_h9p_pcie *pcie)
{
	void __iomem *rc = pcie->base_rc;
	unsigned int port;
	int ret;

	if (!pcie->ans_pre_enumeration ||
	    ((readl(rc + S8003_RC_PORT_CTL(0)) |
	      readl(rc + S8003_RC_PORT_PWR(0)) |
	      readl(rc + S8003_RC_PORT_REFCLK(0))) & BIT(0)))
		return -EBUSY;
	for (port = 0; port < H9P_NUM_PORTS; port++) {
		if (!pcie->perst[port])
			continue;
		if (gpiod_get_value_cansleep(pcie->perst[port]) != 1)
			return -EBUSY;
		if (!(readl(rc + S8003_RC_PORT_CTL(port)) & BIT(0)) ||
		    !(readl(rc + S8003_RC_PORT_PWR(port)) & BIT(0)))
			continue;
		apple_s8003_port_stop(pcie, port);
		ret = apple_s8003_port_wait(pcie, port,
					    pcie->base_port[port] + H9P_PORT_LTSSMCTL,
					    H9P_PORT_LTSSM_ENABLE, 0, 250000,
					    "shared-PHY-stop");
		if (ret)
			return ret;
	}
	dev_info(pcie->dev, "Apple ANS: STEP port 0 shared-PHY configure replay (pre-scan, all PERST asserted)\n");
	return apple_s8003_init_common(pcie);
}

static int apple_s8003_deep_recover(struct apple_h9p_pcie *pcie)
{
	unsigned int attempt;
	int ret;

	dev_warn(pcie->dev, "Apple ANS: STEP port 0 deep-ladder begin\n");
	ret = apple_s8003_retrain(pcie, false);
	if (!ret)
		return 0;
	ret = apple_s8003_retrain(pcie, true);
	if (!ret)
		return 0;
	for (attempt = 0; attempt < 3; attempt++) {
		dev_info(pcie->dev, "Apple ANS: STEP port 0 deep-cycle %u/3\n", attempt + 1);
		ret = apple_s8003_deep_cycle(pcie);
		if (!ret)
			return 0;
		if (ret != -ETIMEDOUT && ret != -ENOLINK)
			break;
	}
	/* A common PHY replay changes ALL lanes. Only the initial host probe
	 * precedes Wi-Fi/PCI enumeration; an explicit rescue retry cannot do it.
	 */
	if (pcie->ans_pre_enumeration && (ret == -ETIMEDOUT || ret == -ENOLINK)) {
		ret = apple_s8003_replay_phy(pcie);
		if (!ret)
			ret = apple_s8003_deep_cycle(pcie);
		if (!ret)
			return 0;
	} else {
		dev_info(pcie->dev, "Apple ANS: STEP port 0 shared-PHY replay SKIP after probe or non-link failure\n");
	}
	/* DART platform providers have already probed before this host. The
	 * warm keep-on policy also makes genpd power_off a no-op. Detaching
	 * only this consumer cannot certify actual-off and risks live DART
	 * state. Do not call that an apcie or S3E power cycle.
	 */
	dev_err(pcie->dev, "Apple ANS: STEP port 0 apcie-domain-cycle SKIP: DART consumers/keep-on; actual-off unproven\n");
	dev_err(pcie->dev, "Apple ANS: deep recovery exhausted %d; DFU required if explicit rescue retry fails\n",
		ret);
	return ret;
}

/* Before enumeration there is no controller worker to recover a missing root.
 * After the usual electrical attempts, allow one cold port-0 retry. Endpoint
 * preinit then rebuilds DART/NVMMU and the controller/admin queues as usual.
 * Never touch an endpoint BAR, a shared PMGR domain, or the Wi-Fi port here.
 */
static int apple_s8003_boot_retry(struct apple_h9p_pcie *pcie, int reason)
{
	void __iomem *rc = pcie->base_rc;
	u32 ctl, pwr, reset;
	int ret;

	if (pcie->ans_boot_retry_used ||
	    pcie->ans_mode == APPLE_ANS_ROUND7 ||
	    (reason != -ETIMEDOUT && reason != -ENOLINK))
		return reason;
	pcie->ans_boot_retry_used = true;
	dev_warn(pcie->dev, "Apple ANS: no root endpoint: port 0 bring-up failed %d; one full controller+port boot retry\n",
		 reason);
	ret = gpiod_direction_output(pcie->perst[0], 1);
	if (ret)
		return ret;
	if ((readl(rc + S8003_RC_PORT_CTL(0)) & S8003_RC_PORT_CTL_ENABLE) &&
	    (readl(rc + S8003_RC_PORT_PWR(0)) & S8003_RC_PORT_PWR_ON)) {
		apple_s8003_port_stop(pcie, 0);
		ret = apple_s8003_port_wait(pcie, 0,
					    pcie->base_port[0] + H9P_PORT_LTSSMCTL,
					    H9P_PORT_LTSSM_ENABLE, 0, 250000,
					    "boot-retry-stop");
		if (ret)
			return ret;
	}
	apple_s8003_port_off(pcie, 0);
	ctl = readl(rc + S8003_RC_PORT_CTL(0));
	pwr = readl(rc + S8003_RC_PORT_PWR(0));
	reset = readl(rc + S8003_RC_PORT_REFCLK(0));
	if ((ctl | pwr | reset) & BIT(0))
		return dev_err_probe(pcie->dev, -EIO,
				     "STEP port 0 boot-retry-off FAIL ctl=%#x pwr=%#x reset=%#x\n",
				     ctl, pwr, reset);
	dev_info(pcie->dev, "Apple ANS: STEP port 0 boot-retry-off hold-100ms\n");
	msleep(100);
	ret = apple_s8003_setup_port_full(pcie, 0);
	if (ret == -ETIMEDOUT || ret == -ENOLINK) {
		dev_warn(pcie->dev, "Apple ANS: boot retry failed %d; one deep link recovery\n",
			 ret);
		ret = apple_s8003_deep_recover(pcie);
	}
	if (ret) {
		gpiod_direction_output(pcie->perst[0], 1);
		dev_err(pcie->dev, "Apple ANS: boot retry exhausted %d; no root endpoint, leaving rescue armed\n",
			ret);
	} else {
		dev_info(pcie->dev, "Apple ANS: boot retry link restored; proceeding to controller/DMA bring-up\n");
	}
	return ret;
}

/*
 * iOS AppleS800xPCIePort::disableGated for a linked port: clear the root
 * port's command register and bus numbers, so nothing more is routed to
 * the link; PME_Turn_Off and the ack; L2; PERST#; port off. A missing ack
 * or L2 only gets logged, as in iOS: PERST# and the power-down end the
 * link either way. (iOS has the endpoint in D3hot first; here it stays in
 * D0, see h9p_suspend, so L2 may not be reached.)
 */
static void apple_s8003_port_sleep(struct apple_h9p_pcie *pcie,
				   unsigned int port)
{
	void __iomem *cfg = pcie->base_config + port * H9P_CFG_PORT_STRIDE;
	void __iomem *pb = pcie->base_port[port];
	struct device *dev = pcie->dev;
	u32 ack, l2;
	int ret;

	writel(0, cfg + PCI_COMMAND);
	h9p_rmw(cfg + PCI_PRIMARY_BUS, GENMASK(23, 8), 0);

	writel(S8003_PORT_PMETO_SEND, pb + S8003_PORT_PMETO);
	ret = readl_poll_timeout(pb + S8003_PORT_PMETO, ack,
				 !(ack & S8003_PORT_PMETO_PENDING) ||
				 (ack & S8003_PORT_PMETO_HW_TIMEOUT),
				 1000, S8003_PME_TO_ACK_TIMEOUT_US);
	if (ret || (ack & S8003_PORT_PMETO_PENDING))
		dev_warn(dev, "port %u: no PME_TO_Ack (%#x), continuing\n",
			 port, ack);

	ret = readl_poll_timeout(pb + S8003_PORT_LINKSTS, l2,
				 l2 & S8003_PORT_LINKSTS_L2, 100,
				 S8003_L2_TIMEOUT_US);
	if (ret)
		dev_warn(dev, "port %u: link did not enter L2 (%#x), continuing\n",
			 port, l2);

	gpiod_direction_output(pcie->perst[port], 1);
	writel(S8003_PORT_IRQSTAT_LINK_DOWN, pb + H9P_PORT_IRQSTAT);
	udelay(S8003_PERST_TO_REFCLK_US);
	apple_s8003_port_off(pcie, port);
	dev_info(dev, "port %u: asleep (PME_TO %#x, link %#x)\n", port, ack, l2);
}

static int apple_t8010_init_common(struct apple_h9p_pcie *pcie)
{
	int ret;

	writel(H9P_RC_COMMON_CTL_INIT |
	       FIELD_PREP(H9P_RC_COMMON_CTL_LANE_CFG, pcie->lane_cfg),
	       pcie->base_rc + H9P_RC_COMMON_CTL0);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(0), 0,
		H9P_RC_COMMON_CTL_ENABLE);

	ret = apple_h9p_wait(pcie->base_rc + H9P_RC_COMMON_STAT,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "global PHY init timeout\n");

	ret = apple_h9p_wait(pcie->base_rc + H9P_RC_COMMON_STAT,
			     H9P_RC_COMMON_STAT_READY,
			     H9P_RC_COMMON_STAT_READY,
			     H9P_RC_COMMON_STAT_READY, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "global PHY ready timeout\n");

	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL3);
	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL1);
	usleep_range(5000, 10000);
	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL2);
	usleep_range(500, 1000);

	return 0;
}

/* The DT iommu-map assigns bus 1 to ANS/DART0 and bus 2 to Wi-Fi/DART1.
 * If port 0 is missing, generic scanning otherwise gives bus 1 to Wi-Fi;
 * a later rescue rescan would give ANS bus 2 and the wrong DART. Preserve
 * those RIDs before ANY scan of a boot eligible for port-0 rescue.
 */
static void apple_s8003_rescue_bus(struct apple_h9p_pcie *pcie, unsigned int port)
{
	unsigned int secondary = port + 1;

	h9p_rmw(pcie->base_config + port * H9P_CFG_PORT_STRIDE + PCI_PRIMARY_BUS,
		GENMASK(23, 0), secondary << 8 | secondary << 16);
	dev_info(pcie->dev, "Apple ANS: STEP port %u rescue-bus secondary=%u (fixed DART RID)\n",
		 port, secondary);
}

static int apple_h9p_setup_ports(struct apple_h9p_pcie *pcie)
{
	unsigned int port;
	int ret;

	ret = pcie->hw->init_common(pcie);
	if (ret)
		return ret;

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		if (!(pcie->enabled_ports & BIT(port)))
			continue;

		/*
		 * A port that fails to come up is dropped, so config accesses
		 * never reach a dead link; the other ports carry on.
		 */
		pcie->ans_pre_enumeration = true;
		ret = pcie->hw->setup_port(pcie, port);
		if (ret && !port && pcie->ans_mode != APPLE_ANS_ROUND7)
			ret = apple_s8003_boot_retry(pcie, ret);
		pcie->ans_pre_enumeration = false;
		if (!ret) {
			if (pcie->ans_rescue_allowed)
				apple_s8003_rescue_bus(pcie, port);
			WRITE_ONCE(pcie->down_ports, pcie->down_ports & ~BIT(port));
		}
		if (ret) {
			gpiod_direction_output(pcie->perst[port], 1);
			pcie->enabled_ports &= ~BIT(port);
			WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(port));
			if (!port && pcie->ans_mode != APPLE_ANS_ROUND7 &&
			    (ret == -ETIMEDOUT || ret == -ENOLINK))
				pcie->ans_rescue_allowed = true;
			dev_err(pcie->dev, "port %u disabled (%d)\n", port, ret);
		}
	}

	if (!pcie->enabled_ports && !pcie->late_ports)
		return dev_err_probe(pcie->dev, -ENODEV, "no PCIe link came up\n");

	/* PCIe r6.0 6.6.1: 100 ms after link-up before the first config request */
	msleep(100);
	return 0;
}

/*
 * Power a late port's endpoint and train its link. PERST# has been asserted
 * since probe. Returns -EPROBE_DEFER while the power GPIO's provider (e.g.
 * the PMIC GPIO driver) isn't there yet.
 */
static int apple_h9p_late_port(struct apple_h9p_pcie *pcie, unsigned int port)
{
	struct device *dev = pcie->dev;
	struct gpio_desc *pwren;
	unsigned int try;
	int ret = 0;

	if (!pcie->pwren[port]) {
		pwren = devm_fwnode_gpiod_get(dev,
					      of_fwnode_handle(pcie->port_nodes[port]),
					      "pwren", GPIOD_ASIS, "PWREN");
		if (PTR_ERR(pwren) == -EPROBE_DEFER) {
			if (!(pcie->late_tries % 10))
				dev_info(dev, "port %u: waiting for its power GPIO (%u/%u)\n",
					 port, pcie->late_tries + 1, H9P_LATE_PORT_TRIES);
			return -EPROBE_DEFER;
		}
		if (IS_ERR(pwren))
			return dev_err_probe(dev, PTR_ERR(pwren),
					     "port %u: no power GPIO\n", port);
		pcie->pwren[port] = pwren;
	}

	for (try = 0; try < H9P_LATE_PORT_LINK_TRIES; try++) {
		/* A clean power cycle, PERST# asserted throughout */
		gpiod_direction_output(pcie->perst[port], 1);
		ret = gpiod_direction_output(pcie->pwren[port], 0);
		if (ret)
			return dev_err_probe(dev, ret,
					     "port %u: can't drive power GPIO\n",
					     port);
		msleep(H9P_PWREN_OFF_MS);
		gpiod_set_value_cansleep(pcie->pwren[port], 1);
		msleep(H9P_PWREN_SETTLE_MS);
		dev_info(dev, "port %u: endpoint powered (attempt %u)\n", port,
			 try + 1);

		ret = pcie->hw->setup_port(pcie, port);
		if (!ret)
			break;
	}
	if (ret) {
		gpiod_direction_output(pcie->perst[port], 1);
		gpiod_set_value_cansleep(pcie->pwren[port], 0);
		dev_err(dev, "port %u: no link after %u power cycles (%d), endpoint powered off\n",
			port, try, ret);
		return ret;
	}

	if (pcie->ans_rescue_allowed)
		apple_s8003_rescue_bus(pcie, port);
	/* PCIe r6.0 6.6.1: 100 ms after link-up before the first config request */
	msleep(100);
	pcie->enabled_ports |= BIT(port);
	return 0;
}

static void apple_h9p_late_ports_work(struct work_struct *work)
{
	struct apple_h9p_pcie *pcie = container_of(to_delayed_work(work),
						   struct apple_h9p_pcie,
						   late_work);
	unsigned long pending = pcie->late_ports;
	bool rescan = false, retry = false;
	unsigned int port;
	int ret;

	/* The DT maps bus 1 to ANS/DART0, bus 2 to Wi-Fi/DART1. The
	 * generic host scan reassigns buses, overriding rescue_bus(). With
	 * port 0 absent it gives Wi-Fi bus 1 and attaches it to the stopped
	 * ANS DART. Leave late ports unpublished until ANS owns bus 1.
	 */
	if (of_machine_is_compatible("apple,j71t") &&
	    pcie->ans_mode != APPLE_ANS_ROUND7 &&
	    !(READ_ONCE(pcie->enabled_ports) & BIT(0))) {
		dev_warn(pcie->dev,
			 "ANS port 0 missing; deferring late ports %#lx to preserve DART routing\n",
			 pending);
		return;
	}

	for_each_set_bit(port, &pending, H9P_NUM_PORTS) {
		ret = apple_h9p_late_port(pcie, port);
		if (ret == -EPROBE_DEFER) {
			retry = true;
			continue;
		}
		pcie->late_ports &= ~BIT(port);
		rescan |= !ret;
	}

	if (rescan) {
		pci_lock_rescan_remove();
		pci_rescan_bus(pcie->bridge->bus);
		pci_unlock_rescan_remove();
	}

	if (!retry)
		return;
	if (++pcie->late_tries < H9P_LATE_PORT_TRIES) {
		schedule_delayed_work(&pcie->late_work,
				      msecs_to_jiffies(H9P_LATE_PORT_RETRY_MS));
		return;
	}
	dev_err(pcie->dev, "ports %#x: power GPIO never appeared, left off\n",
		pcie->late_ports);
	pcie->late_ports = 0;
}

static void apple_h9p_late_ports_cancel(void *data)
{
	struct apple_h9p_pcie *pcie = data;

	cancel_delayed_work_sync(&pcie->late_work);
}

static int apple_h9p_pcie_init(struct pci_config_window *cfg)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(cfg->parent);
	int ret;

	if (!pcie)
		return -ENODEV;

	pcie->cfgwin = cfg;
	pcie->base_config = cfg->win;

	ret = apple_h9p_setup_ports(pcie);
	if (ret)
		return ret;

	/* Without the NVMMU only NVMe is lost; keep the other ports. */
	ret = apple_h9p_setup_nvmmu(pcie);
	if (ret)
		dev_err(pcie->dev, "NVMMU setup failed (%d), NVMe disabled\n",
			ret);

	return 0;
}

static const struct pci_ecam_ops apple_h9p_pcie_ecam_ops = {
	.bus_shift = 20,
	.init = apple_h9p_pcie_init,
	.pci_ops = {
		.map_bus = pci_ecam_map_bus,
		.read = apple_h9p_pcie_config_read,
		.write = apple_h9p_pcie_config_write,
	},
};

static int apple_h9p_pcie_map_resources(struct platform_device *pdev,
					struct apple_h9p_pcie *pcie)
{
	unsigned int i;

	pcie->base_rc = devm_platform_ioremap_resource_byname(pdev, "rc");
	if (IS_ERR(pcie->base_rc))
		return PTR_ERR(pcie->base_rc);

	pcie->base_phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(pcie->base_phy))
		return PTR_ERR(pcie->base_phy);

	if (pcie->hw->has_phy_ip) {
		pcie->base_phy_ip =
			devm_platform_ioremap_resource_byname(pdev, "phy-ip");
		if (IS_ERR(pcie->base_phy_ip))
			return PTR_ERR(pcie->base_phy_ip);
	}

	for (i = 0; i < H9P_NUM_PORTS; i++) {
		char name[8];

		if (!((pcie->enabled_ports | pcie->late_ports) & BIT(i)))
			continue;

		snprintf(name, sizeof(name), "port%u", i);
		pcie->base_port[i] = devm_platform_ioremap_resource_byname(pdev,
									   name);
		if (IS_ERR(pcie->base_port[i]))
			return PTR_ERR(pcie->base_port[i]);
	}

	pcie->nvmmu.base =
		devm_platform_ioremap_resource_byname(pdev, "nvmmu");
	if (IS_ERR(pcie->nvmmu.base))
		return PTR_ERR(pcie->nvmmu.base);

	return 0;
}

static void apple_h9p_pcie_put_port_nodes(void *data)
{
	struct apple_h9p_pcie *pcie = data;
	unsigned int i;

	for (i = 0; i < H9P_NUM_PORTS; i++)
		of_node_put(pcie->port_nodes[i]);
}

static int apple_h9p_pcie_parse_ports(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *np;
	u32 reg, port;
	int ret;

	for_each_available_child_of_node(dev->of_node, np) {
		if (!of_node_name_eq(np, "pci"))
			continue;

		ret = of_property_read_u32_index(np, "reg", 0, &reg);
		if (ret)
			goto out_put_child;

		port = reg >> 11;
		if (port >= H9P_NUM_PORTS || pcie->port_nodes[port]) {
			ret = -EINVAL;
			goto out_put_child;
		}

		pcie->port_nodes[port] = of_node_get(np);
		if (aspm_l1_ports & BIT(port)) {
			struct property *prop = of_find_property(np, "aspm-no-l1",
								 NULL);

			if (prop && !of_remove_property(np, prop))
				dev_warn(dev, "port %u: ASPM L1 allowed (aspm_l1_ports)\n",
					 port);
		}
		if (of_property_present(np, "pwren-gpios"))
			pcie->late_ports |= BIT(port);
		else
			pcie->enabled_ports |= BIT(port);

		if (of_property_read_u32(np, "max-link-speed",
					 &pcie->max_speed[port]))
			pcie->max_speed[port] = port ? H9P_LINK_SPEED_2_5GT :
					pcie->hw->port0_default_speed;
		pcie->max_speed[port] = clamp_t(u32, pcie->max_speed[port],
					      H9P_LINK_SPEED_2_5GT,
					      H9P_LINK_SPEED_8GT);
		pcie->nvme_mode[port] = of_property_read_bool(np, "apple,nvme-mode");
	}

	if (!(pcie->enabled_ports | pcie->late_ports))
		return -ENODEV;

	return devm_add_action_or_reset(dev, apple_h9p_pcie_put_port_nodes,
					pcie);

out_put_child:
	of_node_put(np);
	apple_h9p_pcie_put_port_nodes(pcie);
	return ret;
}

static int apple_h9p_pcie_get_gpios(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	unsigned int i;

	for (i = 0; i < H9P_NUM_PORTS; i++) {
		struct fwnode_handle *fwnode;
		struct gpio_desc *gpio;

		if (!((pcie->enabled_ports | pcie->late_ports) & BIT(i)))
			continue;

		fwnode = of_fwnode_handle(pcie->port_nodes[i]);
		gpio = devm_fwnode_gpiod_get(dev, fwnode, "reset",
					     !i && pcie->ans_mode == APPLE_ANS_HANDOFF ?
					     GPIOD_ASIS : GPIOD_OUT_HIGH, "PERST#");
		pcie->perst[i] = gpio;
		if (IS_ERR(pcie->perst[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->perst[i]),
					     "failed to get PERST#%u\n", i);

		/*
		 * Optional: S8003 leaves CLKREQ# in its peripheral function
		 * (requesting it as a GPIO would take it away from the RC).
		 */
		gpio = devm_fwnode_gpiod_get(dev, fwnode, "clkreq", GPIOD_IN, "CLKREQ#");
		if (PTR_ERR(gpio) == -ENOENT)
			gpio = NULL;
		pcie->clkreq[i] = gpio;
		if (IS_ERR(pcie->clkreq[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->clkreq[i]),
					     "failed to get CLKREQ#%u\n", i);

		/* Host-to-endpoint wake (SoC GPIO); left deasserted */
		gpio = devm_fwnode_gpiod_get(dev, fwnode, "device-wake",
					     GPIOD_OUT_LOW, "DEVICE_WAKE");
		if (PTR_ERR(gpio) == -ENOENT)
			gpio = NULL;
		pcie->devwake[i] = gpio;
		if (IS_ERR(pcie->devwake[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->devwake[i]),
					     "failed to get DEVICE_WAKE%u\n", i);
	}

	pcie->devpwr = devm_gpiod_get_array_optional(dev, "devpwr", GPIOD_ASIS);
	if (IS_ERR(pcie->devpwr))
		return dev_err_probe(dev, PTR_ERR(pcie->devpwr),
				     "failed to get device power GPIOs\n");

	return 0;
}

static const struct attribute_group apple_h9p_crash_group;

/* Only a port which NEVER enumerated in this kernel can be retried here.
 * Serialize with PCI rescans, close the gate before publishing any function,
 * and never remove a device or disturb a live/failed controller object.
 */
static ssize_t ans_recover_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(dev);
	struct pci_dev *rp;
	int ret;

	if (!pcie || !sysfs_streq(buf, "1") || is_kdump_kernel() ||
	    !of_machine_is_compatible("apple,j71t") ||
	    pcie->ans_mode == APPLE_ANS_ROUND7 ||
	    !of_device_is_compatible(dev->of_node, "apple,s8003-pcie"))
		return -EPERM;
	pci_lock_rescan_remove();
	ret = -EBUSY;
	if (!pcie->ans_rescue_allowed || (pcie->enabled_ports & BIT(0)) ||
	    pcie->nvmmu.tcb)
		goto out;
	rp = pci_get_slot(pcie->bridge->bus, PCI_DEVFN(0, 0));
	if (rp) {
		pci_dev_put(rp);
		goto out;
	}
	pcie->ans_rescue_attempts++;
	dev_warn(dev, "Apple ANS: STEP port 0 rescue-retry %u begin\n",
		 pcie->ans_rescue_attempts);
	ret = apple_s8003_deep_recover(pcie);
	if (ret)
		goto out;
	apple_s8003_rescue_bus(pcie, 0);
	/* Even an NVMMU allocation/init failure cannot reopen this gate. */
	pcie->ans_rescue_allowed = false;
	pcie->enabled_ports |= BIT(0);
	ret = apple_h9p_setup_nvmmu(pcie);
	if (ret) {
		pcie->enabled_ports &= ~BIT(0);
		gpiod_direction_output(pcie->perst[0], 1);
		h9p_writel_flush(0, pcie->nvmmu.base + H9P_NVMMU_SART_CTRL);
		pcie->nvmmu.ready = false;
		pcie->nvmmu.active = false;
		apple_s8003_port_stop(pcie, 0);
		if (!apple_s8003_port_wait(pcie, 0,
					   pcie->base_port[0] + H9P_PORT_LTSSMCTL,
					   H9P_PORT_LTSSM_ENABLE, 0, 250000,
					   "rescue-failed-stop"))
			apple_s8003_port_off(pcie, 0);
		goto out;
	}
	msleep(100);
	WRITE_ONCE(pcie->down_ports, pcie->down_ports & ~BIT(0));
	pci_rescan_bus(pcie->bridge->bus);
	dev_info(dev, "Apple ANS: STEP port 0 rescue-rescan done; controller/DMA health gate required\n");
out:
	pcie->ans_rescue_result = ret;
	pci_unlock_rescan_remove();
	/* Successful ANS publication reserves bus 1 before Wi-Fi can scan. */
	if (!ret && pcie->late_ports)
		schedule_delayed_work(&pcie->late_work, 0);
	return ret ? ret : count;
}

static ssize_t ans_recover_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(dev);

	return sysfs_emit(buf, "eligible=%u attempts=%u result=%d\n",
			 READ_ONCE(pcie->ans_rescue_allowed),
			 READ_ONCE(pcie->ans_rescue_attempts),
			 READ_ONCE(pcie->ans_rescue_result));
}
static DEVICE_ATTR_RW(ans_recover);

static struct attribute *apple_h9p_ans_attrs[] = {
	&dev_attr_ans_recover.attr,
	NULL,
};

static const struct attribute_group apple_h9p_ans_group = {
	.attrs = apple_h9p_ans_attrs,
};

static int apple_h9p_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	struct apple_h9p_pcie *pcie;
	int ret;

	if (!READ_ONCE(h9p_probe) && of_machine_is_compatible("apple,j71t"))
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for initramfs USB rescue (probe gate)\n");

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);
	pcie->dev = dev;
	pcie->pdev = pdev;
	pcie->bridge = bridge;
	pcie->ans_mode = of_machine_is_compatible("apple,j71t") ?
		ans_boot_mode : APPLE_ANS_ROUND7;
	pcie->hw = of_device_get_match_data(dev);
	if (!pcie->hw)
		return -ENODEV;
	spin_lock_init(&pcie->used_msi_lock);

	/* ADT "lane-cfg" of the apcie node */
	of_property_read_u32(dev->of_node, "apple,lane-cfg", &pcie->lane_cfg);
	if (pcie->lane_cfg > FIELD_MAX(H9P_RC_COMMON_CTL_LANE_CFG) ||
	    !(pcie->hw->lane_cfgs & BIT(pcie->lane_cfg)))
		return dev_err_probe(dev, -EINVAL, "bad apple,lane-cfg %u\n",
				     pcie->lane_cfg);

	ret = apple_h9p_pcie_parse_ports(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse root ports\n");
	dev_info(dev, "ports %#x, lane-cfg %u\n", pcie->enabled_ports,
		 pcie->lane_cfg);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	ret = apple_h9p_pcie_attach_genpd(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach power domains\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_genpd_cleanup, pcie);
	if (ret)
		return ret;

	pcie->clks[0].id = "core";
	pcie->clks[1].id = "aux";
	pcie->clks[2].id = "ref";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(pcie->clks), pcie->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(pcie->clks), pcie->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_clk_cleanup, pcie);
	if (ret)
		return ret;

	pcie->pinctrl = devm_pinctrl_get_select_default(dev);
	if (PTR_ERR(pcie->pinctrl) == -ENODEV)
		pcie->pinctrl = NULL;
	else if (IS_ERR(pcie->pinctrl))
		return dev_err_probe(dev, PTR_ERR(pcie->pinctrl),
				     "failed to select pinctrl state\n");

	ret = apple_h9p_pcie_map_resources(pdev, pcie);
	if (ret)
		return ret;

	ret = apple_h9p_pcie_get_gpios(pcie);
	if (ret)
		return ret;

	ret = of_property_read_u64(dev->of_node, "apple,msi-doorbell",
				   &pcie->msi_doorbell);
	if (ret)
		pcie->msi_doorbell = H9P_DEFAULT_MSI_DOORBELL;

	ret = apple_h9p_pcie_setup_msi(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up MSI\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_msi_cleanup, pcie);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&pcie->late_work, apple_h9p_late_ports_work);
	ret = devm_add_action_or_reset(dev, apple_h9p_late_ports_cancel, pcie);
	if (ret)
		return ret;

	ret = pci_host_common_init(pdev, bridge, &apple_h9p_pcie_ecam_ops);
	if (ret)
		return ret;

	if (pcie->ans_mode != APPLE_ANS_ROUND7 &&
	    of_device_is_compatible(dev->of_node, "apple,s8003-pcie")) {
		ret = devm_device_add_group(dev, &apple_h9p_ans_group);
		if (ret)
			return ret;
	}

	if (is_kdump_kernel() && of_machine_is_compatible("apple,j71t")) {
		ret = devm_device_add_group(dev, &apple_h9p_crash_group);
		if (ret)
			return ret;
	}

	if (pcie->late_ports) {
		dev_info(dev, "ports %#x: bringing up after their endpoint power\n",
			 pcie->late_ports);
		schedule_delayed_work(&pcie->late_work, 0);
	}
	return 0;
}

/* Shared round-8f shutdown certificate: endpoint must already be stopped. */
static int apple_s8003_ans_port_down(struct apple_h9p_pcie *pcie, bool fatal)
{
	void __iomem *cfg = pcie->base_config;
	void __iomem *rc = pcie->base_rc;
	u32 ctl, pwr, reset;
	int ret;

	writew(0, cfg + PCI_COMMAND);
	h9p_rmw(cfg + PCI_PRIMARY_BUS, GENMASK(23, 8), 0);
	WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(0));
	ret = gpiod_direction_output(pcie->perst[0], 1);
	if (ret) {
		if (fatal)
			panic("Apple ANS: outgoing PERST failed %d; refusing kexec", ret);
		return ret;
	}
	apple_s8003_port_stop(pcie, 0);
	ret = apple_s8003_port_wait(pcie, 0,
				    pcie->base_port[0] + H9P_PORT_LTSSMCTL,
				    H9P_PORT_LTSSM_ENABLE, 0, 250000,
				    "handoff-stop");
	if (ret) {
		if (fatal)
			panic("Apple ANS: outgoing LTSSM stop failed %d; refusing kexec", ret);
		return ret;
	}
	if (pcie->nvmmu.ready) {
		h9p_writel_flush(0, pcie->nvmmu.base + H9P_NVMMU_SART_CTRL);
		if (fatal)
			pcie->nvmmu.ready = false;
	}
	pcie->nvmmu.active = false;
	/* Powered retention is kexec-only; sleep keeps its full-off certificate. */
	if (fatal && ans_keep_power && pcie->ans_mode == APPLE_ANS_HANDOFF) {
		h9p_rmw(rc + S8003_RC_PORT_PWR(0), 0,
			S8003_RC_PORT_REFCLK_NOGATE);
		ctl = readl(rc + S8003_RC_PORT_CTL(0));
		pwr = readl(rc + S8003_RC_PORT_PWR(0));
		reset = readl(rc + S8003_RC_PORT_REFCLK(0));
		if (!(ctl & S8003_RC_PORT_CTL_ENABLE) ||
		    (ctl & S8003_RC_PORT_CTL_BIT8) ||
		    !(pwr & S8003_RC_PORT_PWR_ON) ||
		    !(pwr & S8003_RC_PORT_REFCLK_NOGATE) ||
		    !(reset & S8003_RC_PORT_REFCLK_ON) ||
		    !(readl(rc + S8003_RC_PORT_READY(0)) & BIT(0)) ||
		    gpiod_get_value_cansleep(pcie->perst[0]) != 1)
			panic("Apple ANS: outgoing powered port certificate failed; refusing kexec");
		dev_info(pcie->dev, "Apple ANS: STEP port 0 handoff-powered RC124=%#x RC114=%#x RC100=%#x PERST=asserted; incoming=powered-reset\n",
			 reset, pwr, ctl);
		pcie->enabled_ports &= ~BIT(0);
		return 0;
	}
	apple_s8003_port_off(pcie, 0);
	ctl = readl(rc + S8003_RC_PORT_CTL(0));
	pwr = readl(rc + S8003_RC_PORT_PWR(0));
	reset = readl(rc + S8003_RC_PORT_REFCLK(0));
	if ((ctl | pwr | reset) & BIT(0)) {
		if (fatal)
			panic("Apple ANS: outgoing port 0 not down ctl=%#x pwr=%#x reset=%#x; refusing kexec",
			      ctl, pwr, reset);
		return -EIO;
	}
	dev_info(pcie->dev, "Apple ANS: STEP port 0 handoff-down RC124=%#x RC114=%#x RC100=%#x PERST=asserted; incoming=full\n",
		 reset, pwr, ctl);
	pcie->enabled_ports &= ~BIT(0);
	if (!fatal)
		dev_info(pcie->dev, "Apple ANS: sleep port 0 fully down; resume=full bring-up\n");
	return 0;
}

/* System sleep, see h9p_suspend */

static void apple_h9p_pcie_unpin(struct apple_h9p_pcie *pcie)
{
	while (pcie->npinned) {
		struct pci_dev *pdev = pcie->pinned[--pcie->npinned];

		pdev->dev_flags &= ~PCI_DEV_FLAGS_NO_D3;
		pci_dev_put(pdev);
	}
}

/* Everything below stays in D0 over the sleep, see h9p_suspend */
static int apple_h9p_pcie_pin(struct pci_dev *pdev, void *data)
{
	struct apple_h9p_pcie *pcie = data;

	if (pdev->dev_flags & PCI_DEV_FLAGS_NO_D3)
		return 0;
	if (pcie->npinned == ARRAY_SIZE(pcie->pinned)) {
		pcie->pin_overflow = true;
		return 1;
	}

	pdev->dev_flags |= PCI_DEV_FLAGS_NO_D3;
	pcie->pinned[pcie->npinned++] = pci_dev_get(pdev);
	return 0;
}

static int apple_h9p_pcie_prepare(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(dev);
	unsigned int mode = READ_ONCE(h9p_suspend);
	unsigned int port;

	if (!pcie)
		return 0;

	if (mode == H9P_SLEEP_OFF) {
		dev_warn(dev, "system sleep refused: untested on this SoC (pcie_apple_h9p.suspend=1 allows it)\n");
		return -EBUSY;
	}
	if (pcie->late_ports || work_busy(&pcie->late_work.work)) {
		dev_warn(dev, "system sleep refused: ports %#x still coming up\n",
			 pcie->late_ports);
		return -EBUSY;
	}
	if (!pcie->pd_dev && dev->pm_domain) {
		dev_warn(dev, "system sleep refused: genpd would power the ports off under the links\n");
		return -EBUSY;
	}

	pcie->sleep_ports = 0;
	WRITE_ONCE(pcie->ans_sleep_stopped, false);
	if (mode > H9P_SLEEP_KEEP_LINKS && !pcie->hw->port_sleep)
		dev_info(dev, "no port sleep sequence for this SoC, links stay up\n");
	else if (mode >= H9P_SLEEP_L2_ALL)
		pcie->sleep_ports = pcie->enabled_ports;
	else if (mode == H9P_SLEEP_L2_POWERED)
		for (port = 0; port < H9P_NUM_PORTS; port++)
			if (pcie->pwren[port])
				pcie->sleep_ports |= BIT(port) & pcie->enabled_ports;

	/* Apple pairs a system-sleep controller stop with full port bring-up.
	 * Even keep-links/powered-only modes must cycle the J71t ANS port.
	 */
	if (of_machine_is_compatible("apple,j71t") && pcie->hw->port_sleep)
		pcie->sleep_ports |= pcie->enabled_ports & BIT(0);

	pcie->pin_overflow = false;
	pci_walk_bus(pcie->bridge->bus, apple_h9p_pcie_pin, pcie);
	if (pcie->pin_overflow) {
		apple_h9p_pcie_unpin(pcie);
		dev_warn(dev, "system sleep refused: too many PCI devices to hold in D0\n");
		return -EBUSY;
	}

	dev_info(dev, "system sleep (mode %u): ports %#x stay linked, ports %#x go to L2\n",
		 mode, pcie->enabled_ports & ~pcie->sleep_ports,
		 pcie->sleep_ports);
	return 0;
}

static void apple_h9p_pcie_complete(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(dev);

	if (!pcie)
		return;

	apple_h9p_pcie_unpin(pcie);
	pcie->sleep_ports = 0;
}

static int apple_h9p_pcie_resume_noirq(struct device *dev);

/* Runs after every endpoint and root port below has suspended */
static int apple_h9p_pcie_suspend_noirq(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(dev);
	unsigned long ports;
	unsigned int port;

	if (!pcie)
		return 0;

	ports = pcie->sleep_ports & pcie->enabled_ports;
	if (of_machine_is_compatible("apple,j71t") && (ports & BIT(0)) &&
	    !READ_ONCE(pcie->ans_sleep_stopped)) {
		dev_err(dev, "Apple ANS: sleep refused: missing endpoint stop certificate\n");
		return -EBUSY;
	}
	pcie->resume_ports = pcie->enabled_ports;
	if (of_machine_is_compatible("apple,j71t") && (ports & BIT(0))) {
		int ret = apple_s8003_ans_port_down(pcie, false);

		if (ret) {
			dev_err(dev, "Apple ANS: sleep port stop failed %d; restoring links\n",
				ret);
			/* Our failing noirq callback will not get a PM resume call. */
			apple_h9p_pcie_resume_noirq(dev);
			return -EBUSY;
		}
		ports &= ~BIT(0);
	}
	for_each_set_bit(port, &ports, H9P_NUM_PORTS) {
		/* Nothing may go out on the link from here on */
		WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(port));
		pcie->enabled_ports &= ~BIT(port);
		pcie->hw->port_sleep(pcie, port);
	}
	return 0;
}

/* PERST# deasserted and the link up, from SoC registers only */
static bool apple_h9p_port_alive(struct apple_h9p_pcie *pcie,
				 unsigned int port)
{
	if (gpiod_get_value_cansleep(pcie->perst[port]))
		return false;
	if (!pcie->hw->linksts)
		return apple_h9p_link_up(pcie, port);

	return readl(pcie->base_port[port] + pcie->hw->linksts) &
	       pcie->hw->linksts_up;
}

/*
 * A port that stays dead: PERST# on, port off, and every device below it
 * marked disconnected, so the PCI core and the drivers (nvme, brcmfmac)
 * leave its BARs alone. A BAR access over a dead link can stall the CPU.
 */
static void apple_h9p_port_cut_off(struct apple_h9p_pcie *pcie,
				   unsigned int port)
{
	struct pci_dev *rp;

	gpiod_direction_output(pcie->perst[port], 1);
	if (pcie->hw->port_off)
		pcie->hw->port_off(pcie, port);
	WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(port));
	pcie->enabled_ports &= ~BIT(port);

	rp = pci_get_slot(pcie->bridge->bus, PCI_DEVFN(port, 0));
	if (!rp)
		return;
	if (rp->subordinate)
		pci_walk_bus(rp->subordinate, pci_dev_set_disconnected, NULL);
	pci_dev_set_disconnected(rp, NULL);
	pci_dev_put(rp);
}

/*
 * Same bring-up as at boot, from a port that is off with PERST# asserted;
 * the endpoint stays powered.
 */
static int apple_h9p_port_wake(struct apple_h9p_pcie *pcie, unsigned int port)
{
	struct pci_dev *rp;
	unsigned int try;
	int ret = -ENODEV;

	for (try = 0; try < H9P_WAKE_LINK_TRIES; try++) {
		ret = pcie->hw->setup_port(pcie, port);
		if (!ret)
			break;
	}
	if (ret) {
		apple_h9p_port_cut_off(pcie, port);
		dev_err(pcie->dev, "port %u: no link after sleep (%d), its devices are cut off until reboot\n",
			port, ret);
		return ret;
	}

	pcie->enabled_ports |= BIT(port);
	WRITE_ONCE(pcie->down_ports, pcie->down_ports & ~BIT(port));

	/*
	 * The root port's routing (bus numbers, windows, command) is needed
	 * before its endpoint resumes, but for a root port left in D0 the PCI
	 * core restores it only in the later resume phase.
	 */
	rp = pci_get_slot(pcie->bridge->bus, PCI_DEVFN(port, 0));
	if (rp) {
		if (!rp->state_saved)
			dev_warn(pcie->dev, "port %u: root port state not saved\n",
				 port);
		pci_restore_state(rp);
		pci_dev_put(rp);
	}
	return 0;
}

/* The NVMMU sits in the always-on part of the RC; check, don't assume */
static void apple_h9p_nvmmu_resume(struct apple_h9p_pcie *pcie, bool retrained)
{
	struct apple_h9p_nvmmu *nvmmu = &pcie->nvmmu;
	struct device *dev = pcie->dev;
	bool kept;
	int ret;

	if (!nvmmu->ready || !(pcie->enabled_ports & BIT(0)))
		return;

	kept = readl(nvmmu->base + H9P_NVMMU_TCB_BASE_LO) ==
		lower_32_bits(nvmmu->tcb_dma) &&
	       readl(nvmmu->base + H9P_NVMMU_TCB_BASE_HI) ==
		upper_32_bits(nvmmu->tcb_dma) &&
	       readl(nvmmu->base + H9P_NVMMU_TCB_TABLE_LO) ==
		lower_32_bits(nvmmu->tcb_table_dma) &&
	       (!nvmmu->sart ||
		(readl(nvmmu->base + H9P_NVMMU_SART_CTRL) & 1));
	if (kept && !retrained) {
		dev_info(dev, "NVMMU: state kept over the sleep\n");
		return;
	}

	dev_info(dev, "NVMMU: %s, programming it again\n",
		 kept ? "port 0 was retrained" : "state lost over the sleep");
	ret = apple_h9p_nvmmu_program(pcie);
	if (ret) {
		/* apple_ans_nvmmu_map() then fails, and nvme gives up cleanly */
		nvmmu->ready = false;
		dev_err(dev, "NVMMU: TCB init timeout after sleep (%d), NVMe stays off\n",
			ret);
	}
}

/*
 * Runs before the root ports and endpoints resume: bring back the ports
 * that slept, and check that the others really kept their link (the PERST#
 * GPIOs live in a power domain of their own) before the PCI core and the
 * drivers touch anything behind them.
 */
static int apple_h9p_pcie_resume_noirq(struct device *dev)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(dev);
	bool retrained = false, port0 = false;
	unsigned long ports;
	unsigned int port;

	if (!pcie)
		return 0;

	apple_h9p_crash_resume_test(dev);
	ports = pcie->resume_ports;
	for_each_set_bit(port, &ports, H9P_NUM_PORTS) {
		bool asleep = pcie->down_ports & BIT(port);

		if (!asleep) {
			if (apple_h9p_port_alive(pcie, port)) {
				dev_info(dev, "port %u: link kept over the sleep\n",
					 port);
				continue;
			}
			dev_warn(dev, "port %u: link lost over the sleep (PERST# %d), retraining\n",
				 port, gpiod_get_value_cansleep(pcie->perst[port]));
			WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(port));
			pcie->enabled_ports &= ~BIT(port);
			gpiod_direction_output(pcie->perst[port], 1);
			if (pcie->hw->port_off)
				pcie->hw->port_off(pcie, port);
		}

		if (apple_h9p_port_wake(pcie, port))
			continue;
		retrained = true;
		port0 |= port == 0;
	}
	pcie->resume_ports = 0;
	WRITE_ONCE(pcie->ans_sleep_stopped, false);

	/* PCIe r6.0 6.6.1: 100 ms after link-up before the first config request */
	if (retrained)
		msleep(100);

	apple_h9p_nvmmu_resume(pcie, port0);
	return 0;
}

/*
 * Children have drained ANS, cleared CC.EN and blocked bus mastering.
 * Isolate under PERST before retiring SART, but keep clocks/port power on.
 * PME_TO/L2 and clock removal gained no DMA safety and broke retraining in
 * test 2. The incoming probe now restarts the port itself before scanning.
 */
static void apple_h9p_pcie_park_round7(struct apple_h9p_pcie *pcie,
				       unsigned long ports)
{
	unsigned int port;

	cancel_delayed_work_sync(&pcie->late_work);
	ports &= pcie->enabled_ports;
	for_each_set_bit(port, &ports, H9P_NUM_PORTS) {
		void __iomem *cfg = pcie->base_config + port * H9P_CFG_PORT_STRIDE;

		WRITE_ONCE(pcie->down_ports, pcie->down_ports | BIT(port));
		writew(0, cfg + PCI_COMMAND);
		h9p_rmw(cfg + PCI_PRIMARY_BUS, GENMASK(23, 8), 0);
		/* Keep the register block/refclk usable across the handoff. */
		h9p_rmw(pcie->base_rc + S8003_RC_PORT_PWR(port), 0,
			S8003_RC_PORT_REFCLK_NOGATE);
		gpiod_direction_output(pcie->perst[port], 1);
		apple_s8003_port_stop(pcie, port);
		udelay(S8003_PERST_TO_REFCLK_US);
		if (!port && pcie->nvmmu.ready) {
			h9p_writel_flush(0, pcie->nvmmu.base + H9P_NVMMU_SART_CTRL);
			pcie->nvmmu.ready = false;
		}
		apple_s8003_port_dump(pcie, port, "kexec parked (powered)");
		pcie->enabled_ports &= ~BIT(port);
	}
	dev_info(pcie->dev, "kexec: PCIe PERST asserted, LTSSM stopped, ports powered; NVMMU SART disabled\n");
}

static void apple_h9p_pcie_park(struct apple_h9p_pcie *pcie,
				unsigned long ports)
{
	if (pcie->ans_mode != APPLE_ANS_ROUND7 && (ports & BIT(0))) {
		cancel_delayed_work_sync(&pcie->late_work);
		dev_info(pcie->dev, "Apple ANS: kexec %s; CC.EN=0/RDY=0 certified by endpoint shutdown\n",
			 ans_boot_modes[pcie->ans_mode]);
		/* Endpoint shutdown retired DART after the CC-disable certificate.
		 * Both policies isolate with PERST/LTSSM before retiring SART.
		 * The opt-in handoff keeps only host power/refclk, never queues.
		 */
		apple_s8003_ans_port_down(pcie, true);
		ports &= ~BIT(0);
	}
	/* Other ports keep the established Round 7 shutdown. */
	apple_h9p_pcie_park_round7(pcie, ports);
}

static void apple_h9p_pcie_shutdown(struct platform_device *pdev)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(&pdev->dev);

	if (kexec_in_progress && pcie &&
	    of_device_is_compatible(pdev->dev.of_node, "apple,s8003-pcie"))
		apple_h9p_pcie_park(pcie, pcie->enabled_ports);
}

struct apple_h9p_crash_state {
	struct apple_h9p_pcie *pcie;
	int result;
	bool busy;
	bool found;
};

/* No NVMe driver, queues, SHN, or old DMA buffers are needed in phoenix. */
static int apple_h9p_crash_stop_ans(struct pci_dev *pdev, void *data)
{
	struct apple_h9p_crash_state *state = data;
	struct apple_h9p_pcie *pcie = state->pcie;
	void __iomem *bar;
	u32 csts = 0;
	int i, ret;

	if (pdev->vendor != PCI_VENDOR_ID_APPLE ||
	    ((pdev->class >> 8) != PCI_CLASS_STORAGE_EXPRESS &&
	     (pdev->class >> 8) != PCI_CLASS_STORAGE_OTHER))
		return 0;
	state->found = true;
	if (pdev->driver) {
		state->busy = true;
		return 1;
	}
	if (apple_ans_link_up(&pdev->dev) <= 0) {
		state->result = -ENOLINK;
		return 0;
	}
	ret = pci_enable_device_mem(pdev);
	if (ret) {
		state->result = ret;
		return 0;
	}
	bar = pci_iomap_range(pdev, 0, 0, SZ_8K);
	if (!bar) {
		ret = -ENOMEM;
		goto disable;
	}
	/* An unknown queue must never be rung or sent a shutdown notification. */
	dev_info(pcie->dev, "phoenix ANS: CC %#x CSTS %#x, clearing EN without SHN\n",
		 readl(bar + NVME_REG_CC), readl(bar + NVME_REG_CSTS));
	writel(0, bar + NVME_REG_CC);
	ret = -ETIMEDOUT;
	for (i = 0; i < 2000; i++) {
		if (apple_ans_link_up(&pdev->dev) <= 0) {
			ret = -ENOLINK;
			break;
		}
		csts = readl(bar + NVME_REG_CSTS);
		if (csts == U32_MAX) {
			ret = -EIO;
			break;
		}
		if (!(csts & NVME_CSTS_RDY)) {
			ret = 0;
			break;
		}
		usleep_range(1000, 1100);
	}
	dev_info(pcie->dev, "phoenix ANS: disable result %d CSTS %#x; blocking bus master\n",
		 ret, csts);
	pci_iounmap(pdev, bar);
disable:
	pci_clear_master(pdev);
	pci_disable_device(pdev);
	state->result = ret;
	return 0;
}

static ssize_t crash_quiesce_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(dev);
	struct apple_h9p_crash_state state = { .pcie = pcie };

	if (!is_kdump_kernel() || !of_machine_is_compatible("apple,j71t") ||
	    !pcie || !sysfs_streq(buf, "1"))
		return -EPERM;
	/* The endpoint gate must stay closed; don't park under a live driver. */
	if (pcie->late_ports || work_busy(&pcie->late_work.work))
		return -EBUSY;
	pci_walk_bus(pcie->bridge->bus, apple_h9p_crash_stop_ans, &state);
	if (state.busy)
		return -EBUSY;
	/* Isolation is still required after a bounded RDY timeout. */
	/* Wi-Fi stays linked until its own shutdown callback has run. */
	/* Panic recovery has no SHST-certified handoff. Keep its isolation. */
	apple_h9p_pcie_park_round7(pcie, BIT(0));
	dev_info(dev, "phoenix: ANS/PCIe quiesced (controller found %d result %d)\n",
		 state.found, state.result);
	return count;
}
static DEVICE_ATTR_WO(crash_quiesce);

static struct attribute *apple_h9p_crash_attrs[] = {
	&dev_attr_crash_quiesce.attr,
	NULL,
};

static const struct attribute_group apple_h9p_crash_group = {
	.attrs = apple_h9p_crash_attrs,
};

static const struct dev_pm_ops apple_h9p_pcie_pm_ops = {
	.prepare = apple_h9p_pcie_prepare,
	.complete = apple_h9p_pcie_complete,
	.suspend_noirq = apple_h9p_pcie_suspend_noirq,
	.resume_noirq = apple_h9p_pcie_resume_noirq,
};

static const struct apple_h9p_hw apple_h9p_hw_t8010 = {
	.has_phy_ip = true,
	.port0_default_speed = H9P_LINK_SPEED_8GT,
	.nvmmu_init_busy = 0x10,
	.lane_cfgs = GENMASK(15, 0),
	.init_common = apple_t8010_init_common,
	.setup_port = apple_t8010_setup_port,
};

static const struct apple_h9p_hw apple_h9p_hw_s8003 = {
	.has_phy_ip = true,
	.port0_default_speed = H9P_LINK_SPEED_5GT,
	.nvmmu_init_busy = H9P_NVMMU_TCB_CTRL_INIT,
	/* iOS panics on anything else ("X1 on port 0 not supported") */
	.lane_cfgs = BIT(1) | BIT(3),
	.linksts = S8003_PORT_LINKSTS,
	.linksts_up = S8003_PORT_LINKSTS_UP,
	.init_common = apple_s8003_init_common,
	.setup_port = apple_s8003_setup_port,
	.port_sleep = apple_s8003_port_sleep,
	.port_off = apple_s8003_port_off,
};

static const struct of_device_id apple_h9p_pcie_of_match[] = {
	{ .compatible = "apple,t8010-pcie", .data = &apple_h9p_hw_t8010 },
	{ .compatible = "apple,s8003-pcie", .data = &apple_h9p_hw_s8003 },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_h9p_pcie_of_match);

static struct platform_driver apple_h9p_pcie_driver = {
	.probe = apple_h9p_pcie_probe,
	.shutdown = apple_h9p_pcie_shutdown,
	.driver = {
		.name = "pcie-apple-h9p",
		.of_match_table = apple_h9p_pcie_of_match,
		.pm = pm_sleep_ptr(&apple_h9p_pcie_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(apple_h9p_pcie_driver);

MODULE_DESCRIPTION("Apple H8P/H9P (S8003/T8010) PCIe host bridge driver");
MODULE_LICENSE("GPL");

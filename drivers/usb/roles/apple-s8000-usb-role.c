// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple S8000 (A9) Lightning port USB role switch.
 *
 * The A9 USB complex has a device-only DWC2 (GHWCFG2.OP_MODE 4) and a
 * separate EHCI + OHCI host pair, all behind one AUSB PHY. For a USB device
 * on the port (cable type USBDevice) iOS's arbitrator applies publish mask
 * 5: EHCI and OHCI (publish-criteria 4) are published and the device nub
 * (criteria 3) stays. The PHY moves the pins with USB_CTL bit 0 and the
 * host tunables (AppleS5L8960XUSBPhy::enableHostMode); its power-up
 * (setPowerLevel >= 2) also sets USB_CTL bit 1, which iOS keeps set while
 * the PHY is powered.
 *
 * Device mode is the default. Host mode is entered on request through the
 * usb_role class and, unless confirmed, reverts to device mode after a
 * timeout: the gadget network link is down while the port is a host. A host
 * role claimed by the Lightning accessory policy does not revert; it lasts
 * until the accessory is unplugged.
 *
 * The D2365 LDO5 rows below come from the iOS PMU LDO table (ADT dock
 * function-acc_pwr = pmuL(0x5), function-acc_sw_en = pmuL(0x80000005),
 * function-acc_sleep_pwr = pmuL(0x220)). iOS powers the camera adapter in
 * active mode 4 and enables sleep power before the hub enumerates. The
 * Tristar bit is AppleTriStarBuiltIn::setVBUSSink; on this board setting it
 * makes the CBTL1610 report no VBUS, so the accessory path never uses it.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/usb/apple-s8000-role.h>
#include <linux/usb/gadget.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#include "apple-s8000-tristar-route.h"

/* USB_CTL (ADT otgphyctrl reg[1]) */
#define USB_CTL_HOST			0x00
#define USB_CTL_HOST_EN			BIT(0)
/* Set by AppleS5L8960XUSBPhy::setPowerLevel(>= 2), cleared below that. */
#define USB_CTL_PHY_ON			BIT(1)
/*
 * Not written by iOS: its device-mode enable is PHY SIG (+0x04) bit 1, the
 * bit phy-apple-ausb sets as VBUSDET_FORCE_EN. Kept for the old opt-in knob.
 */
#define USB_CTL_DEVICE			0x04
#define USB_CTL_DEVICE_EN		BIT(1)

/*
 * USB complex DMA remap words, owned and programmed at probe by the parent
 * usb-complex driver (drivers/bus/apple-usbcomplex.c), as m1n1 does too.
 */
#define REMAP_DEVICE			0x1c
#define REMAP_EHCI0			0x3c
#define REMAP_OHCI0			0x5c
#define REMAP_EHCI1			0x7c
#define REMAP_LAST			0x9c
#define REMAP_EN			BIT(8)
#define REMAP_UPPER			GENMASK(3, 0)
/* DRAM is at 0x8_0000_0000; the bus dma-ranges hand out 32-bit addresses. */
#define REMAP_TO_DRAM			(REMAP_EN | FIELD_PREP(REMAP_UPPER, 0x8))

/* Glue words: EHCI0 +0x00, OHCI0 +0x08, device +0x20 (ADT reg[1] entries) */
#define GLUE_SIZE			0x30

/* D2365 LDO5, iOS AppleD2365PMU LDO table rows 5 and 32 */
#define D2365_LDO5_ENABLE		0x0310
#define D2365_LDO5_EN			BIT(0)
/* ADT function-acc_sleep_pwr = pmuL(0x220): row 0x20, enable mask 0x0e. */
#define D2365_LDO5_SLEEP_EN		GENMASK(3, 1)
#define D2365_LDO5_VSEL			0x2600
#define D2365_LDO5_SWITCH		0x2603

/* CBTL1610 (Tristar) */
#define TRISTAR_DX_CONTROL		0x01
#define TRISTAR_VBUS_SINK		0x05
#define TRISTAR_VBUS_SINK_EN		BIT(0)
#define TRISTAR_ACC_CTRL		0x02
#define TRISTAR_STATUS			0x0c
#define TRISTAR_REVISION		0x0f
#define TRISTAR_J71T_REVISION		0x8a
#define TRISTAR_SINK_SETTLE_MS		18
#define TRISTAR_ROUTE_SETTLE_MS	5

/* Read-only mux snapshots. Do not read the event/ack register 0x0d here. */
static const u8 tristar_snapshot_regs[] = {
	TRISTAR_REVISION, TRISTAR_DX_CONTROL, 0x02, 0x03,
	TRISTAR_VBUS_SINK, TRISTAR_STATUS, 0x17, 0x0a,
};

#define GATE_PMU			BIT(0)
#define GATE_TRISTAR			BIT(1)

static const char * const gate_names[] = {
	[0] = "none",
	[GATE_PMU] = "pmu",
	[GATE_TRISTAR] = "tristar",
	[GATE_PMU | GATE_TRISTAR] = "both",
};

static unsigned int revert_timeout = 60;
module_param(revert_timeout, uint, 0644);
MODULE_PARM_DESC(revert_timeout,
		 "Default seconds before an unconfirmed host role reverts to device (0: never)");

static bool clear_device_enable;
module_param(clear_device_enable, bool, 0644);
MODULE_PARM_DESC(clear_device_enable,
		 "Also clear USB_CTL+4 bit 1 while in host mode (default: leave it; iOS never writes it)");

static bool usb_ctl_phy_on = true;
module_param(usb_ctl_phy_on, bool, 0644);
MODULE_PARM_DESC(usb_ctl_phy_on,
		 "Set USB_CTL+0 bit 1 in host mode, as iOS does while its PHY is powered (default: on)");

/*
 * Debug: once host mode is up, write this value to USB_CTL+0 and log the
 * readback. Only bits 0 (host select, always kept) and 1 are used.
 */
static unsigned int usb_ctl_force;
module_param(usb_ctl_force, uint, 0644);
MODULE_PARM_DESC(usb_ctl_force,
		 "Debug: after host entry write USB_CTL+0 = this | 1 (e.g. 3) and log readbacks (0: off)");

static bool ios_host_order = true;
module_param(ios_host_order, bool, 0644);
MODULE_PARM_DESC(ios_host_order,
		 "Create host controllers before USB_CTL host select, matching iOS (off: round-5b A/B)");

static bool accessory_sleep_power = true;
module_param(accessory_sleep_power, bool, 0644);
MODULE_PARM_DESC(accessory_sleep_power,
		 "Enable D2365 function-acc_sleep_pwr (0x0310 mask 0x0e) for accessory host mode");

static unsigned int accessory_acc_config = 2;
module_param(accessory_acc_config, uint, 0644);
MODULE_PARM_DESC(accessory_acc_config,
		 "TriStar writes iOS makes for the camera adapter before accessory power: 0 none, 1 ACC_CTRL 0x02 (idbs/apwr by orientation), 2 also dxctrl 0x01 (usb0+urt0) (default: 2)");

static bool gadget_soft_disconnect = true;
module_param(gadget_soft_disconnect, bool, 0644);
MODULE_PARM_DESC(gadget_soft_disconnect,
		 "Keep the gadget bound and use its pull-up for role changes (off: round-6 unbind/rebind)");

#define USB_CTL_TRACE_MAX		12
#define LINK_TRACE_MAX			16
#define PMU_TRACE_MAX			8
#define TRISTAR_TRACE_MAX		3

/* Only one switch exists on this SoC; the accessory API checks against it. */
static DEFINE_MUTEX(apple_usb_role_instance_lock);
static struct apple_usb_role *apple_usb_role_instance;

struct apple_usb_role {
	struct device *dev;
	void __iomem *usb_ctl;
	/* Shared, not claimed: the usb-complex driver owns the remap words. */
	void __iomem *remap;
	void __iomem *glue;
	/* Read-only views of registers that other drivers own */
	void __iomem *phy_regs;
	void __iomem *udc_regs;
	void __iomem *ehci_regs;
	void __iomem *ohci_regs;

	struct phy *phy;
	struct platform_device *udc;
	struct device_node *ehci_np;
	struct device_node *ohci_np;
	struct regmap *pmic;
	struct i2c_adapter *tristar;
	u16 tristar_addr;

	struct usb_role_switch *sw;
	struct delayed_work revert_work;
	struct dentry *debugfs;

	/* Serializes role changes, the gates and the state below. */
	struct mutex lock;
	enum usb_role role;
	struct device *gadget;
	struct platform_device *ehci;
	struct platform_device *ohci;
	unsigned long revert_at;
	unsigned int revert_timeout;
	unsigned int gates;
	unsigned int gates_on;
	/* Gates for a host role claimed by the accessory policy */
	unsigned int accessory_gates;
	bool revert_pending;
	bool phy_held;
	bool ctl_host;
	bool ctl_phy_on;
	bool cleared_device_enable;
	bool sleep_power_on;
	bool gadget_soft;
	/* The next host entry is the accessory policy's */
	bool accessory_claim;
	/* PMU accessory gates were applied before the post-GetID reads. */
	bool accessory_power_prepared;
	/* TriStar 0x01/0x02 were changed for the adapter; the saved bytes restore them. */
	bool acc_applied;
	u8 acc_saved;
	u8 dx_saved;
	/* The current host role is the accessory policy's: no revert */
	bool accessory;
	bool dying;
	bool monitor_fault;
	/* USB_CTL+0 readbacks at each step of the last host entry */
	const char *ctl_trace_what[USB_CTL_TRACE_MAX];
	u32 ctl_trace[USB_CTL_TRACE_MAX];
	unsigned int ctl_trace_n;
	/* Last debug write (usb_ctl_force or debugfs usb_ctl) and its readbacks */
	u32 ctl_forced;
	u32 ctl_forced_rb[3];
	bool ctl_force_done;
	/* PHY/DWC2 state at each role transition boundary. */
	const char *link_trace_what[LINK_TRACE_MAX];
	u32 link_trace_phy_sig[LINK_TRACE_MAX];
	u32 link_trace_dctl[LINK_TRACE_MAX];
	u32 link_trace_dsts[LINK_TRACE_MAX];
	unsigned int link_trace_n;
	/* D2365 0x0310 after each accessory-power operation. */
	const char *pmu_trace_what[PMU_TRACE_MAX];
	u32 pmu_trace[PMU_TRACE_MAX];
	unsigned int pmu_trace_n;
	/* Read-only Tristar mux snapshots: entry, host-ready, device-ready. */
	const char *tristar_trace_what[TRISTAR_TRACE_MAX];
	int tristar_trace[TRISTAR_TRACE_MAX][ARRAY_SIZE(tristar_snapshot_regs)];
	unsigned int tristar_trace_n;
};

static void apple_usb_role_iounmap(void *addr)
{
	iounmap(addr);
}

/* Map a register range without claiming it: its owner is another driver. */
static void __iomem *apple_usb_role_view(struct apple_usb_role *ur,
					 struct device_node *np, int index)
{
	void __iomem *base = of_iomap(np, index);

	if (!base || devm_add_action_or_reset(ur->dev, apple_usb_role_iounmap,
					      (void __force *)base))
		return NULL;
	return base;
}

static void apple_usb_role_link_trace(struct apple_usb_role *ur, const char *what)
{
	unsigned int n = ur->link_trace_n;

	if (n >= LINK_TRACE_MAX)
		return;
	ur->link_trace_what[n] = what;
	ur->link_trace_phy_sig[n] = ur->phy_regs ? readl(ur->phy_regs + 0x04) : U32_MAX;
	if (ur->udc_regs && ur->udc->dev.driver) {
		ur->link_trace_dctl[n] = readl(ur->udc_regs + 0x804);
		ur->link_trace_dsts[n] = readl(ur->udc_regs + 0x808);
	} else {
		ur->link_trace_dctl[n] = U32_MAX;
		ur->link_trace_dsts[n] = U32_MAX;
	}
	ur->link_trace_n++;
}

/* -------------------------------------------------------------------- */
/* Accessory power gates */

static int apple_usb_role_tristar_read(struct apple_usb_role *ur, u8 reg)
{
	union i2c_smbus_data data;
	int ret;

	/* Selector + STOP, then a separate read, as the Tristar driver does. */
	i2c_lock_bus(ur->tristar, I2C_LOCK_SEGMENT);
	ret = __i2c_smbus_xfer(ur->tristar, ur->tristar_addr, 0,
			       I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE, NULL);
	if (!ret)
		ret = __i2c_smbus_xfer(ur->tristar, ur->tristar_addr, 0,
				       I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	i2c_unlock_bus(ur->tristar, I2C_LOCK_SEGMENT);
	return ret ?: data.byte;
}

static void apple_usb_role_tristar_trace(struct apple_usb_role *ur, const char *what)
{
	unsigned int i, n = ur->tristar_trace_n;

	if (!ur->tristar || n >= TRISTAR_TRACE_MAX)
		return;
	ur->tristar_trace_what[n] = what;
	for (i = 0; i < ARRAY_SIZE(tristar_snapshot_regs); i++)
		ur->tristar_trace[n][i] =
			apple_usb_role_tristar_read(ur, tristar_snapshot_regs[i]);
	ur->tristar_trace_n++;
}

static int apple_usb_role_tristar_sink(struct apple_usb_role *ur, bool on)
{
	union i2c_smbus_data data;
	int ret;

	if (!ur->tristar)
		return -ENODEV;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_REVISION);
	if (ret < 0)
		return ret;
	if (ret != TRISTAR_J71T_REVISION)
		return -ENODEV;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_VBUS_SINK);
	if (ret < 0)
		return ret;
	data.byte = on ? ret | TRISTAR_VBUS_SINK_EN : ret & ~TRISTAR_VBUS_SINK_EN;
	ret = i2c_smbus_xfer(ur->tristar, ur->tristar_addr, 0, I2C_SMBUS_WRITE,
			     TRISTAR_VBUS_SINK, I2C_SMBUS_BYTE_DATA, &data);
	if (!ret && on)
		msleep(TRISTAR_SINK_SETTLE_MS);
	return ret;
}

static int apple_usb_role_ldo5(struct apple_usb_role *ur, bool on)
{
	if (!ur->pmic)
		return -ENODEV;
	return regmap_update_bits(ur->pmic, D2365_LDO5_ENABLE, D2365_LDO5_EN,
				  on ? D2365_LDO5_EN : 0);
}

static int apple_usb_role_ldo5_switch(struct apple_usb_role *ur, bool on)
{
	if (!ur->pmic)
		return -ENODEV;
	/* iOS writes the whole register: 1 or 0. */
	return regmap_write(ur->pmic, D2365_LDO5_SWITCH, on);
}

static int apple_usb_role_ldo5_sleep(struct apple_usb_role *ur, bool on)
{
	if (!ur->pmic)
		return -ENODEV;
	return regmap_update_bits(ur->pmic, D2365_LDO5_ENABLE,
				  D2365_LDO5_SLEEP_EN,
				  on ? D2365_LDO5_SLEEP_EN : 0);
}

static void apple_usb_role_pmu_trace(struct apple_usb_role *ur, const char *what)
{
	unsigned int n = ur->pmu_trace_n;
	unsigned int val;

	if (!ur->pmic || n >= PMU_TRACE_MAX)
		return;
	ur->pmu_trace_what[n] = what;
	if (regmap_read(ur->pmic, D2365_LDO5_ENABLE, &val))
		val = U32_MAX;
	ur->pmu_trace[n] = val;
	ur->pmu_trace_n++;
}

static int apple_usb_role_tristar_write(struct apple_usb_role *ur, u8 reg, u8 val)
{
	union i2c_smbus_data data = { .byte = val };
	int ret;

	ret = i2c_smbus_xfer(ur->tristar, ur->tristar_addr, 0, I2C_SMBUS_WRITE, reg,
			     I2C_SMBUS_BYTE_DATA, &data);
	if (ret)
		return ret;
	msleep(TRISTAR_ROUTE_SETTLE_MS);
	ret = apple_usb_role_tristar_read(ur, reg);
	if (ret < 0)
		return ret;
	return ret == val ? 0 : -EIO;
}

/*
 * iOS applies the ACC and route configuration when the accessory attaches and
 * again before it enables accessory power, so the ACC pin carries the power rail
 * (apwr) before LDO5 comes on. Linux used to leave both registers at zero. The
 * values and the dxctrl write order are in apple-s8000-tristar-route.h.
 */
static int apple_usb_role_acc_config(struct apple_usb_role *ur)
{
	unsigned int level = min(READ_ONCE(accessory_acc_config), 2U);
	struct tristar_route_plan plan;
	int ret, status;

	if (!level || ur->acc_applied)
		return 0;
	if (!ur->tristar)
		return -ENODEV;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_REVISION);
	if (ret < 0)
		return ret;
	if (ret != TRISTAR_J71T_REVISION)
		return -ENODEV;
	status = apple_usb_role_tristar_read(ur, TRISTAR_STATUS);
	if (status < 0)
		return status;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_DX_CONTROL);
	if (ret < 0)
		return ret;
	ur->dx_saved = ret;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_ACC_CTRL);
	if (ret < 0)
		return ret;
	ur->acc_saved = ret;
	if (!tristar_route_plan_for(status, ur->dx_saved, &plan)) {
		dev_warn(ur->dev, "TriStar ACC config: no plug (0x0c=%#04x)\n", status);
		return -ENOLINK;
	}
	/* From the first write on, teardown restores both registers. */
	ur->acc_applied = true;
	ret = 0;
	if (level >= 2 && plan.dx_write) {
		if (plan.dx_two_steps)
			ret = apple_usb_role_tristar_write(ur, TRISTAR_DX_CONTROL, plan.dx_first);
		if (!ret)
			ret = apple_usb_role_tristar_write(ur, TRISTAR_DX_CONTROL, plan.dx_target);
		dev_info(ur->dev, "TriStar dxctrl 0x01: %#04x -> %#04x -> %#04x (%d)\n",
			 ur->dx_saved, plan.dx_two_steps ? plan.dx_first : plan.dx_target,
			 plan.dx_target, ret);
		if (ret)
			return ret;
	}
	if (ur->acc_saved != plan.acc)
		ret = apple_usb_role_tristar_write(ur, TRISTAR_ACC_CTRL, plan.acc);
	dev_info(ur->dev,
		 "TriStar ACC config level %u, orientation %u (0x0c=%#04x): ACC_CTRL 0x02 %#04x -> %#04x (%d)\n",
		 level, plan.orientation, status, ur->acc_saved, plan.acc, ret);
	return ret;
}

static void apple_usb_role_acc_restore_reg(struct apple_usb_role *ur, u8 reg, u8 saved)
{
	int ret = apple_usb_role_tristar_read(ur, reg);

	if (ret == saved)
		return;
	if (ret >= 0)
		ret = apple_usb_role_tristar_write(ur, reg, saved);
	if (ret < 0)
		dev_err(ur->dev, "TriStar 0x%02x restore: %d\n", reg, ret);
}

static void apple_usb_role_acc_restore(struct apple_usb_role *ur)
{
	if (!ur->acc_applied)
		return;
	ur->acc_applied = false;
	/* Reverse of the apply order: ACC_CTRL, then dxctrl. */
	apple_usb_role_acc_restore_reg(ur, TRISTAR_ACC_CTRL, ur->acc_saved);
	apple_usb_role_acc_restore_reg(ur, TRISTAR_DX_CONTROL, ur->dx_saved);
	dev_info(ur->dev, "TriStar ACC_CTRL %#04x and dxctrl %#04x restored\n",
		 ur->acc_saved, ur->dx_saved);
}

static void apple_usb_role_gates_off(struct apple_usb_role *ur)
{
	int ret;

	if (ur->gates_on & GATE_TRISTAR) {
		ret = apple_usb_role_tristar_sink(ur, false);
		if (ret)
			dev_err(ur->dev, "Tristar VBUS sink off: %d\n", ret);
	}
	if (ur->gates_on & GATE_PMU) {
		if (ur->sleep_power_on) {
			ret = apple_usb_role_ldo5_sleep(ur, false);
			if (ret)
				dev_err(ur->dev, "LDO5 sleep power off: %d\n", ret);
			ur->sleep_power_on = false;
			apple_usb_role_pmu_trace(ur, "sleep-off");
		}
		ret = apple_usb_role_ldo5_switch(ur, false);
		if (ret)
			dev_err(ur->dev, "LDO5 switch off: %d\n", ret);
		apple_usb_role_pmu_trace(ur, "switch-off");
		ret = apple_usb_role_ldo5(ur, false);
		if (ret)
			dev_err(ur->dev, "LDO5 off: %d\n", ret);
		apple_usb_role_pmu_trace(ur, "power-off");
	}
	apple_usb_role_acc_restore(ur);
	ur->gates_on = 0;
}

static int apple_usb_role_gates_on(struct apple_usb_role *ur, unsigned int gates,
				   bool sleep_power, bool acc_config)
{
	int ret;

	if (acc_config && (gates & GATE_PMU)) {
		/* The ACC pin must carry apwr before LDO5 comes on, as in iOS. */
		ret = apple_usb_role_acc_config(ur);
		if (ret) {
			dev_err(ur->dev, "TriStar ACC config: %d\n", ret);
			return ret;
		}
	}
	if (gates & GATE_PMU) {
		/* Order is inferred: rail first, then the accessory switch. */
		ur->gates_on |= GATE_PMU;
		ret = apple_usb_role_ldo5(ur, true);
		apple_usb_role_pmu_trace(ur, "acc-pwr");
		if (!ret)
			ret = apple_usb_role_ldo5_switch(ur, true);
		apple_usb_role_pmu_trace(ur, "acc-sw");
		if (!ret && sleep_power) {
			ret = apple_usb_role_ldo5_sleep(ur, true);
			if (!ret)
				ur->sleep_power_on = true;
			apple_usb_role_pmu_trace(ur, "acc-sleep");
		}
		if (ret) {
			dev_err(ur->dev, "accessory power (LDO5): %d\n", ret);
			return ret;
		}
	}
	if (gates & GATE_TRISTAR) {
		ur->gates_on |= GATE_TRISTAR;
		ret = apple_usb_role_tristar_sink(ur, true);
		if (ret) {
			dev_err(ur->dev, "Tristar VBUS sink: %d\n", ret);
			return ret;
		}
	}
	return 0;
}

/* -------------------------------------------------------------------- */
/* Role changes */

static int apple_usb_role_is_gadget(struct device *dev, const void *data)
{
	return dev->bus && !strcmp(dev->bus->name, "gadget");
}

/*
 * iOS leaves the device nub published while its host nubs are active. Keep
 * configfs and its netdev bound, and normally drop only DWC2's pull-up. The
 * round-6 unbind/rebind path remains as a one-boot A/B control.
 */
static int apple_usb_role_release_gadget(struct apple_usb_role *ur)
{
	struct device *gadget;
	int ret;

	gadget = device_find_child(&ur->udc->dev, NULL, apple_usb_role_is_gadget);
	if (!gadget) {
		dev_info(ur->dev, "no gadget registered on %s\n", dev_name(&ur->udc->dev));
		return 0;
	}
	if (!gadget->driver) {
		/* Nothing to give back later either. */
		put_device(gadget);
		return 0;
	}
	if (READ_ONCE(gadget_soft_disconnect)) {
		dev_info(ur->dev, "soft-disconnecting gadget driver %s\n",
			 gadget->driver->name);
		ret = usb_gadget_disconnect(dev_to_usb_gadget(gadget));
		if (ret) {
			dev_err(ur->dev, "gadget soft disconnect: %d\n", ret);
			put_device(gadget);
			return ret;
		}
		ur->gadget_soft = true;
		ur->gadget = gadget;
		return 0;
	}
	dev_info(ur->dev, "unbinding gadget driver %s\n", gadget->driver->name);
	device_release_driver(gadget);
	ur->gadget = gadget;
	return 0;
}

static void apple_usb_role_restore_gadget(struct apple_usb_role *ur)
{
	int ret;

	if (!ur->gadget)
		return;
	if (ur->gadget_soft) {
		ret = usb_gadget_connect(dev_to_usb_gadget(ur->gadget));
		if (ret)
			dev_err(ur->dev, "gadget soft reconnect: %d\n", ret);
		else
			dev_info(ur->dev, "gadget driver %s soft-connected again\n",
				 ur->gadget->driver ? ur->gadget->driver->name : "none");
		ur->gadget_soft = false;
		put_device(ur->gadget);
		ur->gadget = NULL;
		return;
	}
	/* configfs keeps its UDC name, so the bus rematches the same gadget. */
	ret = device_attach(ur->gadget);
	if (ret < 0)
		dev_err(ur->dev, "gadget rebind: %d\n", ret);
	else if (!ret)
		dev_warn(ur->dev, "no gadget driver matched; write the configfs UDC\n");
	else
		dev_info(ur->dev, "gadget driver %s bound again\n", ur->gadget->driver->name);
	put_device(ur->gadget);
	ur->gadget = NULL;
}

/* Only repairs a value lost since the usb-complex driver's probe. */
static void apple_usb_role_remap(struct apple_usb_role *ur, unsigned int offset)
{
	const u32 want = REMAP_TO_DRAM;
	u32 val = readl(ur->remap + offset);

	if (val == want)
		return;
	dev_info(ur->dev, "DMA remap +%#x: %#x -> %#x\n", offset, val, want);
	writel(want, ur->remap + offset);
}

static struct platform_device *apple_usb_role_add_hc(struct apple_usb_role *ur,
						     struct device_node *np)
{
	struct platform_device *pdev = of_platform_device_create(np, NULL, ur->dev);

	if (!pdev) {
		dev_err(ur->dev, "%pOFn: could not create the controller\n", np);
		return NULL;
	}
	/* ehci/ohci-platform prefer asynchronous probing. */
	wait_for_device_probe();
	if (!pdev->dev.driver) {
		dev_err(ur->dev, "%s: no driver bound\n", dev_name(&pdev->dev));
		of_platform_device_destroy(&pdev->dev, NULL);
		return NULL;
	}
	return pdev;
}

/* Undo whatever enter_host() got to, in reverse order. Idempotent. */
static void apple_usb_role_host_off(struct apple_usb_role *ur)
{
	int ret;

	if (ur->ohci) {
		of_platform_device_destroy(&ur->ohci->dev, NULL);
		ur->ohci = NULL;
	}
	if (ur->ehci) {
		of_platform_device_destroy(&ur->ehci->dev, NULL);
		ur->ehci = NULL;
	}
	apple_usb_role_link_trace(ur, "hcs-off");
	/* iOS order: enableHostMode(0) clears bit 0, the power-down bit 1. */
	if (ur->ctl_host) {
		writel(readl(ur->usb_ctl + USB_CTL_HOST) & ~USB_CTL_HOST_EN,
		       ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_host = false;
	}
	if (ur->ctl_phy_on) {
		writel(readl(ur->usb_ctl + USB_CTL_HOST) & ~USB_CTL_PHY_ON,
		       ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_phy_on = false;
	}
	if (ur->cleared_device_enable) {
		writel(readl(ur->usb_ctl + USB_CTL_DEVICE) | USB_CTL_DEVICE_EN,
		       ur->usb_ctl + USB_CTL_DEVICE);
		ur->cleared_device_enable = false;
	}
	apple_usb_role_link_trace(ur, "ctl-off");
	if (ur->phy_held) {
		ret = phy_set_mode(ur->phy, PHY_MODE_USB_DEVICE);
		if (ret)
			dev_err(ur->dev, "PHY device mode: %d\n", ret);
		apple_usb_role_link_trace(ur, "phy-device");
		phy_power_off(ur->phy);
		phy_exit(ur->phy);
		ur->phy_held = false;
	}
	apple_usb_role_gates_off(ur);
	ur->accessory_power_prepared = false;
	apple_usb_role_link_trace(ur, "gates-off");
}

/* Record USB_CTL+0 after a step of the host entry (2026-10-27: bit 1 was lost). */
static void apple_usb_role_ctl_trace(struct apple_usb_role *ur, const char *what)
{
	if (ur->ctl_trace_n >= USB_CTL_TRACE_MAX)
		return;
	ur->ctl_trace_what[ur->ctl_trace_n] = what;
	ur->ctl_trace[ur->ctl_trace_n++] = readl(ur->usb_ctl + USB_CTL_HOST);
}

static void apple_usb_role_ctl_trace_log(struct apple_usb_role *ur)
{
	char line[192];
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ur->ctl_trace_n; i++)
		len += scnprintf(line + len, sizeof(line) - len, "%s%s %#x", i ? ", " : "",
				 ur->ctl_trace_what[i], ur->ctl_trace[i]);
	dev_info(ur->dev, "USB_CTL+0 trace: %s\n", len ? line : "none");
}

/* Debug write of USB_CTL+0 in host mode; bit 0 (host select) always kept. */
static void apple_usb_role_ctl_force(struct apple_usb_role *ur, u32 value)
{
	u32 val = (value & USB_CTL_PHY_ON) | USB_CTL_HOST_EN;

	writel(val, ur->usb_ctl + USB_CTL_HOST);
	ur->ctl_forced_rb[0] = readl(ur->usb_ctl + USB_CTL_HOST);
	usleep_range(1000, 1500);
	ur->ctl_forced_rb[1] = readl(ur->usb_ctl + USB_CTL_HOST);
	msleep(100);
	ur->ctl_forced_rb[2] = readl(ur->usb_ctl + USB_CTL_HOST);
	ur->ctl_forced = val;
	ur->ctl_force_done = true;
	/* Bit 1 written by us is cleared again when leaving host mode. */
	if (val & USB_CTL_PHY_ON)
		ur->ctl_phy_on = true;
	dev_info(ur->dev, "USB_CTL+0 forced to %#x: read back %#x, %#x after 1 ms, %#x after 100 ms\n",
		 val, ur->ctl_forced_rb[0], ur->ctl_forced_rb[1], ur->ctl_forced_rb[2]);
}

static int apple_usb_role_add_hcs(struct apple_usb_role *ur)
{
	/* EHCI before its OHCI companion, as for any companion pair. */
	ur->ehci = apple_usb_role_add_hc(ur, ur->ehci_np);
	if (!ur->ehci)
		return -ENODEV;
	apple_usb_role_ctl_trace(ur, "ehci");
	ur->ohci = apple_usb_role_add_hc(ur, ur->ohci_np);
	if (!ur->ohci)
		dev_warn(ur->dev, "no OHCI: low/full-speed devices will not work\n");
	apple_usb_role_ctl_trace(ur, "ohci");
	return 0;
}

static void apple_usb_role_set_phy_on(struct apple_usb_role *ur, const char *what)
{
	u32 ctl = readl(ur->usb_ctl + USB_CTL_HOST);

	if (usb_ctl_phy_on && !(ctl & USB_CTL_PHY_ON)) {
		writel(ctl | USB_CTL_PHY_ON, ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_phy_on = true;
	}
	apple_usb_role_ctl_trace(ur, what);
}

static int apple_usb_role_enter_host(struct apple_usb_role *ur, unsigned int gates)
{
	bool ios_order = READ_ONCE(ios_host_order);
	unsigned int force = READ_ONCE(usb_ctl_force);
	u32 ctl;
	int ret;

	ur->ctl_trace_n = 0;
	ur->ctl_force_done = false;
	ur->link_trace_n = 0;
	ur->pmu_trace_n = 0;
	ur->tristar_trace_n = 0;
	apple_usb_role_link_trace(ur, "entry");
	apple_usb_role_pmu_trace(ur, "entry");
	apple_usb_role_tristar_trace(ur, "entry");
	apple_usb_role_ctl_trace(ur, "entry");
	ret = apple_usb_role_release_gadget(ur);
	if (ret)
		goto fail;
	apple_usb_role_link_trace(ur, "gadget-off");
	apple_usb_role_ctl_trace(ur, "gadget-off");

	ret = apple_usb_role_gates_on(ur, gates,
				      ur->accessory_claim && READ_ONCE(accessory_sleep_power),
				      ur->accessory_claim);
	if (ret)
		goto fail;
	apple_usb_role_link_trace(ur, "gates-on");

	apple_usb_role_remap(ur, REMAP_EHCI0);
	apple_usb_role_remap(ur, REMAP_OHCI0);

	ret = phy_init(ur->phy);
	if (ret)
		goto fail;
	ret = phy_power_on(ur->phy);
	if (ret) {
		phy_exit(ur->phy);
		goto fail;
	}
	ur->phy_held = true;
	apple_usb_role_link_trace(ur, "phy-on");
	apple_usb_role_ctl_trace(ur, "phy-on");

	/*
	 * iOS publishes EHCI/OHCI before enableHostMode. Their power-domain
	 * callbacks and the PHY power callback therefore run before the direct
	 * USB_CTL bit-1 RMW at fffffff006cbf968..fffffff006cbf984. Round 5b
	 * wrote bit 1 while those domains were still off; hardware discarded it.
	 * Keep the old order as a writable, one-boot A/B control.
	 */
	if (ios_order) {
		ret = apple_usb_role_add_hcs(ur);
		if (ret)
			goto fail;
		apple_usb_role_set_phy_on(ur, "powered-bit1");
	} else {
		/* Round-5b control: bit 1 before host-controller power. */
		apple_usb_role_set_phy_on(ur, "early-bit1");
	}

	/* enableHostMode: host select, then tunables and the reset (set_mode). */
	ctl = readl(ur->usb_ctl + USB_CTL_HOST);
	if (usb_ctl_phy_on)
		ctl |= USB_CTL_PHY_ON;
	writel(ctl | USB_CTL_HOST_EN, ur->usb_ctl + USB_CTL_HOST);
	ur->ctl_host = true;
	apple_usb_role_ctl_trace(ur, "host");
	if (clear_device_enable) {
		writel(readl(ur->usb_ctl + USB_CTL_DEVICE) & ~USB_CTL_DEVICE_EN,
		       ur->usb_ctl + USB_CTL_DEVICE);
		ur->cleared_device_enable = true;
	}
	ret = phy_set_mode(ur->phy, PHY_MODE_USB_HOST);
	if (ret)
		goto fail;
	apple_usb_role_ctl_trace(ur, "phy-host");
	apple_usb_role_link_trace(ur, "phy-host");

	if (!ios_order) {
		ret = apple_usb_role_add_hcs(ur);
		if (ret)
			goto fail;
	}

	/* If something dropped bit 1 on the way, assert it once more. */
	ctl = readl(ur->usb_ctl + USB_CTL_HOST);
	if (usb_ctl_phy_on && !(ctl & USB_CTL_PHY_ON)) {
		writel(ctl | USB_CTL_PHY_ON, ur->usb_ctl + USB_CTL_HOST);
		apple_usb_role_ctl_trace(ur, "reassert");
	}
	apple_usb_role_ctl_trace_log(ur);
	if (force)
		apple_usb_role_ctl_force(ur, force);
	apple_usb_role_link_trace(ur, "host-ready");
	apple_usb_role_tristar_trace(ur, "host-ready");

	dev_info(ur->dev, "host mode%s (USB_CTL %#x/%#x, accessory power: %s, sleep power: %s)\n",
		 ur->accessory_claim ? " for the Lightning accessory" : "",
		 readl(ur->usb_ctl + USB_CTL_HOST), readl(ur->usb_ctl + USB_CTL_DEVICE),
		 gate_names[ur->gates_on], ur->sleep_power_on ? "on" : "off");
	return 0;

fail:
	apple_usb_role_host_off(ur);
	apple_usb_role_restore_gadget(ur);
	return ret;
}

static void apple_usb_role_enter_device(struct apple_usb_role *ur)
{
	apple_usb_role_host_off(ur);
	apple_usb_role_restore_gadget(ur);
	apple_usb_role_link_trace(ur, "gadget-on");
	apple_usb_role_tristar_trace(ur, "device-ready");
	dev_info(ur->dev, "device mode\n");
}

static void apple_usb_role_arm_revert(struct apple_usb_role *ur)
{
	if (!ur->revert_timeout) {
		ur->revert_pending = false;
		return;
	}
	ur->revert_at = jiffies + secs_to_jiffies(ur->revert_timeout);
	ur->revert_pending = true;
	mod_delayed_work(system_wq, &ur->revert_work,
			 secs_to_jiffies(ur->revert_timeout));
	dev_info(ur->dev, "returning to device mode in %u s unless confirmed\n",
		 ur->revert_timeout);
}

static int apple_usb_role_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct apple_usb_role *ur = usb_role_switch_get_drvdata(sw);
	int ret = 0;

	/* There is no idle state: the gadget is the resting role. */
	if (role == USB_ROLE_NONE)
		role = USB_ROLE_DEVICE;

	mutex_lock(&ur->lock);
	if (ur->dying) {
		ret = -ESHUTDOWN;
	} else if (role == USB_ROLE_HOST && ur->monitor_fault) {
		ret = -EUCLEAN;
	} else if (role == ur->role) {
		if (role == USB_ROLE_HOST && ur->accessory_claim) {
			/* The accessory takes over a manual host role. */
			ur->accessory = true;
			ur->revert_pending = false;
			cancel_delayed_work(&ur->revert_work);
		} else if (role == USB_ROLE_HOST && ur->revert_pending) {
			/* Asking for host again restarts an unconfirmed countdown. */
			apple_usb_role_arm_revert(ur);
		}
	} else if (role == USB_ROLE_HOST) {
		bool accessory = ur->accessory_claim;

		ret = apple_usb_role_enter_host(ur, accessory ? ur->accessory_gates : ur->gates);
		if (!ret) {
			WRITE_ONCE(ur->role, USB_ROLE_HOST);
			ur->accessory = accessory;
			ur->accessory_power_prepared = false;
			if (!accessory)
				apple_usb_role_arm_revert(ur);
		}
	} else {
		ur->revert_pending = false;
		/* Not _sync: this may be the revert work itself. */
		cancel_delayed_work(&ur->revert_work);
		apple_usb_role_enter_device(ur);
		WRITE_ONCE(ur->role, USB_ROLE_DEVICE);
		ur->accessory = false;
	}
	ur->accessory_claim = false;
	mutex_unlock(&ur->lock);
	return ret;
}

/* Caller holds apple_usb_role_instance_lock, which keeps the instance alive. */
static struct apple_usb_role *apple_usb_role_from_switch(struct usb_role_switch *sw)
{
	struct apple_usb_role *ur;

	lockdep_assert_held(&apple_usb_role_instance_lock);
	if (IS_ERR_OR_NULL(sw) || !apple_usb_role_instance)
		return NULL;
	ur = usb_role_switch_get_drvdata(sw);
	return ur == apple_usb_role_instance ? ur : NULL;
}

/**
 * apple_s8000_usb_role_claim - mark the next host entry as the accessory's
 * @sw: the Lightning role switch
 * @claim: true before usb_role_switch_set_role(USB_ROLE_HOST); false to drop
 *	   a claim that was not used
 *
 * A claimed host role has no revert timeout and uses the accessory power
 * gates. Any device-role request ends it.
 *
 * Return: 0, or -ENODEV if @sw is not this driver's switch.
 */
int apple_s8000_usb_role_claim(struct usb_role_switch *sw, bool claim)
{
	struct apple_usb_role *ur;
	int ret = 0;

	mutex_lock(&apple_usb_role_instance_lock);
	ur = apple_usb_role_from_switch(sw);
	if (ur) {
		mutex_lock(&ur->lock);
		if (claim && ur->dying)
			ret = -ESHUTDOWN;
		ur->accessory_claim = claim && !ret;
		mutex_unlock(&ur->lock);
	} else {
		ret = -ENODEV;
	}
	mutex_unlock(&apple_usb_role_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_s8000_usb_role_claim);

/**
 * apple_s8000_usb_role_accessory_power - apply/release local power mode 4
 * @sw: the Lightning role switch
 * @on: apply before ID-bus identification, or unwind after a failed sequence
 *
 * iOS sets IOAccessoryActivePowerMode=4 before it publishes the adapter's
 * information properties or changes USB mode. This is local PMU/TriStar gate
 * setup, not an ID-bus command. A successful host entry consumes the setup;
 * host teardown uses the ordinary reverse-order gate path.
 */
int apple_s8000_usb_role_accessory_power(struct usb_role_switch *sw, bool on)
{
	struct apple_usb_role *ur;
	int ret = -ENODEV;

	mutex_lock(&apple_usb_role_instance_lock);
	ur = apple_usb_role_from_switch(sw);
	if (ur) {
		mutex_lock(&ur->lock);
		if (ur->dying) {
			ret = -ESHUTDOWN;
		} else if (ur->role != USB_ROLE_DEVICE) {
			ret = -EBUSY;
		} else if (on && !ur->accessory_power_prepared) {
			ret = apple_usb_role_gates_on(ur, ur->accessory_gates,
						      READ_ONCE(accessory_sleep_power), true);
			if (ret) {
				apple_usb_role_gates_off(ur);
			} else {
				ur->accessory_power_prepared = true;
				dev_info(ur->dev, "accessory power mode 4 prepared before ID-bus reads\n");
			}
		} else if (!on && ur->accessory_power_prepared) {
			apple_usb_role_gates_off(ur);
			ur->accessory_power_prepared = false;
			dev_info(ur->dev, "accessory power mode 4 released\n");
			ret = 0;
		} else {
			ret = 0;
		}
		mutex_unlock(&ur->lock);
	}
	mutex_unlock(&apple_usb_role_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_s8000_usb_role_accessory_power);

/**
 * apple_s8000_usb_role_accessory - whether the accessory owns the host role
 * @sw: the Lightning role switch
 *
 * Return: true while the port is a host on the accessory policy's request.
 */
bool apple_s8000_usb_role_accessory(struct usb_role_switch *sw)
{
	struct apple_usb_role *ur;
	bool ret = false;

	mutex_lock(&apple_usb_role_instance_lock);
	ur = apple_usb_role_from_switch(sw);
	if (ur) {
		mutex_lock(&ur->lock);
		ret = ur->role == USB_ROLE_HOST && ur->accessory;
		mutex_unlock(&ur->lock);
	}
	mutex_unlock(&apple_usb_role_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_s8000_usb_role_accessory);

/**
 * apple_s8000_usb_role_monitor - serialize an analog monitor with role changes
 * @sw: the Lightning role switch
 * @sample: sleepable callback; must not query or change the USB role
 * @data: callback argument
 *
 * Lock order: instance, role hardware, consumer, gadget budget. The callback
 * must restore DX_CONTROL before returning. EUCLEAN inhibits host entry until
 * a subsequent callback repairs the route. No role transition can overlap it.
 *
 * Return: callback result, or an error without calling it outside device role.
 */
int apple_s8000_usb_role_monitor(struct usb_role_switch *sw,
				 int (*sample)(void *data), void *data)
{
	struct apple_usb_role *ur;
	int ret = -ENODEV;

	mutex_lock(&apple_usb_role_instance_lock);
	ur = apple_usb_role_from_switch(sw);
	if (ur) {
		mutex_lock(&ur->lock);
		if (ur->dying || ur->role != USB_ROLE_DEVICE) {
			ret = -EBUSY;
		} else {
			ret = sample(data);
			ur->monitor_fault = ret == -EUCLEAN;
		}
		mutex_unlock(&ur->lock);
	}
	mutex_unlock(&apple_usb_role_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_s8000_usb_role_monitor);

static enum usb_role apple_usb_role_get(struct usb_role_switch *sw)
{
	struct apple_usb_role *ur = usb_role_switch_get_drvdata(sw);

	return READ_ONCE(ur->role);
}

static void apple_usb_role_revert(struct work_struct *work)
{
	struct apple_usb_role *ur = container_of(to_delayed_work(work),
						 struct apple_usb_role, revert_work);

	dev_warn(ur->dev, "host mode was not confirmed, returning to device mode\n");
	usb_role_switch_set_role(ur->sw, USB_ROLE_DEVICE);
}

/* -------------------------------------------------------------------- */
/* sysfs */

static ssize_t revert_timeout_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(ur->revert_timeout));
}

static ssize_t revert_timeout_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	/* Takes effect at the next host entry or host re-request. */
	WRITE_ONCE(ur->revert_timeout, val);
	return count;
}
static DEVICE_ATTR_RW(revert_timeout);

static ssize_t revert_pending_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	long left = 0;

	mutex_lock(&ur->lock);
	if (ur->revert_pending)
		left = max_t(long, 0, (long)(ur->revert_at - jiffies));
	mutex_unlock(&ur->lock);
	return sysfs_emit(buf, "%u\n", jiffies_to_msecs(left) / 1000);
}
static DEVICE_ATTR_RO(revert_pending);

static ssize_t confirm_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	if (!val)
		return count;
	mutex_lock(&ur->lock);
	if (ur->role != USB_ROLE_HOST) {
		ret = -EINVAL;
	} else if (ur->revert_pending) {
		ur->revert_pending = false;
		cancel_delayed_work(&ur->revert_work);
		dev_info(ur->dev, "host mode confirmed\n");
	}
	mutex_unlock(&ur->lock);
	return ret ?: count;
}
static DEVICE_ATTR_WO(confirm);

static ssize_t vbus_gates_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	unsigned int i, gates = READ_ONCE(ur->gates);
	ssize_t len = 0;

	for (i = 0; i < ARRAY_SIZE(gate_names); i++)
		len += sysfs_emit_at(buf, len, i == gates ? "[%s] " : "%s ", gate_names[i]);
	buf[len - 1] = '\n';
	return len;
}

static ssize_t vbus_gates_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	int gates = sysfs_match_string(gate_names, buf);

	if (gates < 0)
		return gates;
	if (((gates & GATE_PMU) && !ur->pmic) ||
	    ((gates & GATE_TRISTAR) && !ur->tristar))
		return -ENODEV;
	/* Takes effect at the next host entry. */
	WRITE_ONCE(ur->gates, gates);
	return count;
}
static DEVICE_ATTR_RW(vbus_gates);

static ssize_t accessory_gates_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	unsigned int i, gates = READ_ONCE(ur->accessory_gates);
	ssize_t len = 0;

	for (i = 0; i <= GATE_PMU; i++)
		len += sysfs_emit_at(buf, len, i == gates ? "[%s] " : "%s ", gate_names[i]);
	buf[len - 1] = '\n';
	return len;
}

/* The Tristar bit removes VBUS from the CBTL1610 status: never for the accessory. */
static ssize_t accessory_gates_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	int gates = sysfs_match_string(gate_names, buf);

	if (gates < 0)
		return gates;
	if (gates & GATE_TRISTAR)
		return -EINVAL;
	if ((gates & GATE_PMU) && !ur->pmic)
		return -ENODEV;
	WRITE_ONCE(ur->accessory_gates, gates);
	return count;
}
static DEVICE_ATTR_RW(accessory_gates);

static ssize_t accessory_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);
	bool accessory;

	mutex_lock(&ur->lock);
	accessory = ur->role == USB_ROLE_HOST && ur->accessory;
	mutex_unlock(&ur->lock);
	return sysfs_emit(buf, "%d\n", accessory);
}
static DEVICE_ATTR_RO(accessory);

static struct attribute *apple_usb_role_attrs[] = {
	&dev_attr_revert_timeout.attr,
	&dev_attr_revert_pending.attr,
	&dev_attr_confirm.attr,
	&dev_attr_vbus_gates.attr,
	&dev_attr_accessory_gates.attr,
	&dev_attr_accessory.attr,
	NULL
};
ATTRIBUTE_GROUPS(apple_usb_role);

/* -------------------------------------------------------------------- */
/* debugfs */

static void apple_usb_role_dump(struct seq_file *s, const char *name,
				void __iomem *base, unsigned int first,
				unsigned int last)
{
	unsigned int off;

	seq_printf(s, "%s:", name);
	for (off = first; off <= last; off += 4)
		seq_printf(s, " +%02x=%08x", off, readl(base + off));
	seq_putc(s, '\n');
}

static void apple_usb_role_dump_pmic(struct seq_file *s, struct apple_usb_role *ur,
				     unsigned int reg)
{
	unsigned int val;
	int ret = regmap_read(ur->pmic, reg, &val);

	if (ret)
		seq_printf(s, " %04x=err%d", reg, ret);
	else
		seq_printf(s, " %04x=%02x", reg, val);
}

static int apple_usb_role_regs_show(struct seq_file *s, void *unused)
{
	struct apple_usb_role *ur = s->private;
	unsigned int i, j;
	int ret;

	mutex_lock(&ur->lock);
	seq_printf(s, "role: %s%s%s, accessory power: %s (armed for host: %s, accessory: %s)\n",
		   usb_role_string(ur->role),
		   ur->revert_pending ? " (revert pending)" : "",
		   ur->role == USB_ROLE_HOST && ur->accessory ? " (Lightning accessory)" : "",
		   gate_names[ur->gates_on], gate_names[ur->gates],
		   gate_names[ur->accessory_gates]);
	seq_printf(s, "phy: init %d power %d, held by switch: %d\n",
		   ur->phy->init_count, ur->phy->power_count, ur->phy_held);
	seq_printf(s, "usb_ctl: +00=%08x +04=%08x (host select %d, phy-on bit1 %d set by switch %d)\n",
		   readl(ur->usb_ctl + USB_CTL_HOST), readl(ur->usb_ctl + USB_CTL_DEVICE),
		   ur->ctl_host, !!(readl(ur->usb_ctl + USB_CTL_HOST) & USB_CTL_PHY_ON),
		   ur->ctl_phy_on);
	seq_puts(s, "usb_ctl trace (last host entry):");
	for (i = 0; i < ur->ctl_trace_n; i++)
		seq_printf(s, " %s=%x", ur->ctl_trace_what[i], ur->ctl_trace[i]);
	seq_putc(s, '\n');
	if (ur->ctl_force_done)
		seq_printf(s, "usb_ctl forced: wrote %x, read %x, %x after 1 ms, %x after 100 ms\n",
			   ur->ctl_forced, ur->ctl_forced_rb[0], ur->ctl_forced_rb[1],
			   ur->ctl_forced_rb[2]);
	seq_puts(s, "link trace (PHY +04 / DWC2 DCTL,DSTS):\n");
	for (i = 0; i < ur->link_trace_n; i++)
		seq_printf(s, " %s: %08x %08x %08x\n", ur->link_trace_what[i],
			   ur->link_trace_phy_sig[i], ur->link_trace_dctl[i],
			   ur->link_trace_dsts[i]);
	seq_puts(s, "pmu 0310 trace:");
	for (i = 0; i < ur->pmu_trace_n; i++)
		seq_printf(s, " %s=%02x", ur->pmu_trace_what[i], ur->pmu_trace[i]);
	seq_putc(s, '\n');
	apple_usb_role_dump(s, "remap", ur->remap, REMAP_DEVICE, REMAP_LAST);

	if (ur->phy_regs) {
		ret = phy_pm_runtime_get_sync(ur->phy);
		/* Without runtime PM (the default) the PHY domain stays on. */
		if (ret >= 0 || !pm_runtime_enabled(&ur->phy->dev))
			apple_usb_role_dump(s, "phy", ur->phy_regs, 0, 0x1c);
		else
			seq_printf(s, "phy: runtime resume failed: %d\n", ret);
		if (ret >= 0)
			phy_pm_runtime_put(ur->phy);
	}

	/*
	 * Controller registers and their glue words only while the controller's
	 * driver holds its power domain: the domain may be off otherwise.
	 */
	if (ur->udc_regs && ur->udc->dev.driver) {
		seq_printf(s, "dwc2: gotgctl=%08x gusbcfg=%08x ghwcfg2=%08x hprt0=%08x dctl=%08x dsts=%08x\n",
			   readl(ur->udc_regs + 0x000), readl(ur->udc_regs + 0x00c),
			   readl(ur->udc_regs + 0x048), readl(ur->udc_regs + 0x440),
			   readl(ur->udc_regs + 0x804), readl(ur->udc_regs + 0x808));
		apple_usb_role_dump(s, "device glue", ur->glue, 0x20, GLUE_SIZE - 4);
	}
	if (ur->ehci_regs && ur->ehci && ur->ehci->dev.driver) {
		u32 cap = readl(ur->ehci_regs);
		void __iomem *op = ur->ehci_regs + (cap & 0xff);

		seq_printf(s, "ehci: cap=%08x hcs=%08x hcc=%08x cmd=%08x sts=%08x seg=%08x cf=%08x portsc1=%08x\n",
			   cap, readl(ur->ehci_regs + 0x04), readl(ur->ehci_regs + 0x08),
			   readl(op + 0x00), readl(op + 0x04), readl(op + 0x10),
			   readl(op + 0x40), readl(op + 0x44));
		apple_usb_role_dump(s, "ehci insnreg", ur->ehci_regs, 0x90, 0xa4);
		apple_usb_role_dump(s, "ehci glue", ur->glue, 0x00, 0x04);
	}
	if (ur->ohci_regs && ur->ohci && ur->ohci->dev.driver) {
		seq_printf(s, "ohci: rev=%08x ctrl=%08x sts=%08x rhdesca=%08x rhdescb=%08x rhsts=%08x port1=%08x\n",
			   readl(ur->ohci_regs + 0x00), readl(ur->ohci_regs + 0x04),
			   readl(ur->ohci_regs + 0x08), readl(ur->ohci_regs + 0x48),
			   readl(ur->ohci_regs + 0x4c), readl(ur->ohci_regs + 0x50),
			   readl(ur->ohci_regs + 0x54));
		apple_usb_role_dump(s, "ohci glue", ur->glue, 0x08, 0x0c);
	}

	if (ur->tristar) {
		seq_puts(s, "tristar:");
		for (i = 0; i < ARRAY_SIZE(tristar_snapshot_regs); i++) {
			ret = apple_usb_role_tristar_read(ur, tristar_snapshot_regs[i]);
			if (ret < 0)
				seq_printf(s, " %02x=err%d", tristar_snapshot_regs[i], ret);
			else
				seq_printf(s, " %02x=%02x", tristar_snapshot_regs[i], ret);
		}
		seq_putc(s, '\n');
		for (i = 0; i < ur->tristar_trace_n; i++) {
			seq_printf(s, "tristar %s:", ur->tristar_trace_what[i]);
			for (j = 0; j < ARRAY_SIZE(tristar_snapshot_regs); j++) {
				ret = ur->tristar_trace[i][j];
				if (ret < 0)
					seq_printf(s, " %02x=err%d", tristar_snapshot_regs[j], ret);
				else
					seq_printf(s, " %02x=%02x", tristar_snapshot_regs[j], ret);
			}
			seq_putc(s, '\n');
		}
	}
	if (ur->pmic) {
		seq_puts(s, "pmu ldo5:");
		apple_usb_role_dump_pmic(s, ur, D2365_LDO5_ENABLE);
		apple_usb_role_dump_pmic(s, ur, D2365_LDO5_VSEL);
		apple_usb_role_dump_pmic(s, ur, D2365_LDO5_SWITCH);
		seq_putc(s, '\n');
	}
	mutex_unlock(&ur->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_usb_role_regs);

/* Raw gate knobs for measurements; they bypass the host-session bookkeeping. */
static int apple_usb_role_ldo5_set(void *data, u64 val)
{
	struct apple_usb_role *ur = data;
	int ret;

	mutex_lock(&ur->lock);
	ret = apple_usb_role_ldo5(ur, val);
	mutex_unlock(&ur->lock);
	return ret;
}

static int apple_usb_role_ldo5_get(void *data, u64 *val)
{
	struct apple_usb_role *ur = data;
	unsigned int reg;
	int ret = regmap_read(ur->pmic, D2365_LDO5_ENABLE, &reg);

	if (ret)
		return ret;
	*val = reg & D2365_LDO5_EN;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(apple_usb_role_ldo5_fops, apple_usb_role_ldo5_get,
			 apple_usb_role_ldo5_set, "%llu\n");

static int apple_usb_role_ldo5_switch_set(void *data, u64 val)
{
	struct apple_usb_role *ur = data;
	int ret;

	mutex_lock(&ur->lock);
	ret = apple_usb_role_ldo5_switch(ur, val);
	mutex_unlock(&ur->lock);
	return ret;
}

static int apple_usb_role_ldo5_switch_get(void *data, u64 *val)
{
	struct apple_usb_role *ur = data;
	unsigned int reg;
	int ret = regmap_read(ur->pmic, D2365_LDO5_SWITCH, &reg);

	if (ret)
		return ret;
	*val = reg;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(apple_usb_role_ldo5_switch_fops,
			 apple_usb_role_ldo5_switch_get,
			 apple_usb_role_ldo5_switch_set, "%llu\n");

static int apple_usb_role_sink_set(void *data, u64 val)
{
	struct apple_usb_role *ur = data;
	int ret;

	mutex_lock(&ur->lock);
	ret = apple_usb_role_tristar_sink(ur, val);
	mutex_unlock(&ur->lock);
	return ret;
}

static int apple_usb_role_sink_get(void *data, u64 *val)
{
	struct apple_usb_role *ur = data;
	int ret = apple_usb_role_tristar_read(ur, TRISTAR_VBUS_SINK);

	if (ret < 0)
		return ret;
	*val = ret & TRISTAR_VBUS_SINK_EN;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(apple_usb_role_sink_fops, apple_usb_role_sink_get,
			 apple_usb_role_sink_set, "%llu\n");

/* Debug: write USB_CTL+0 (bit 1, bit 0 kept) while in host mode; read: current value. */
static int apple_usb_role_usb_ctl_set(void *data, u64 val)
{
	struct apple_usb_role *ur = data;
	int ret = 0;

	if (val & ~(u64)(USB_CTL_HOST_EN | USB_CTL_PHY_ON))
		return -EINVAL;
	mutex_lock(&ur->lock);
	if (ur->role != USB_ROLE_HOST || !ur->ctl_host)
		ret = -EBUSY;
	else
		apple_usb_role_ctl_force(ur, val);
	mutex_unlock(&ur->lock);
	return ret;
}

static int apple_usb_role_usb_ctl_get(void *data, u64 *val)
{
	struct apple_usb_role *ur = data;

	*val = readl(ur->usb_ctl + USB_CTL_HOST);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(apple_usb_role_usb_ctl_fops, apple_usb_role_usb_ctl_get,
			 apple_usb_role_usb_ctl_set, "0x%llx\n");

static void apple_usb_role_debugfs(struct apple_usb_role *ur)
{
	ur->debugfs = debugfs_create_dir("apple-usb-role", NULL);
	debugfs_create_file("regs", 0400, ur->debugfs, ur, &apple_usb_role_regs_fops);
	debugfs_create_file_unsafe("usb_ctl", 0600, ur->debugfs, ur,
				   &apple_usb_role_usb_ctl_fops);
	if (ur->pmic) {
		debugfs_create_file_unsafe("pmu_ldo5", 0600, ur->debugfs, ur,
					   &apple_usb_role_ldo5_fops);
		debugfs_create_file_unsafe("pmu_ldo5_switch", 0600, ur->debugfs, ur,
					   &apple_usb_role_ldo5_switch_fops);
	}
	if (ur->tristar)
		debugfs_create_file_unsafe("tristar_vbus_sink", 0600, ur->debugfs, ur,
					   &apple_usb_role_sink_fops);
}

/* -------------------------------------------------------------------- */
/* Probe */

static void apple_usb_role_put_device(void *data)
{
	put_device(data);
}

static void apple_usb_role_put_node(void *data)
{
	of_node_put(data);
}

static void apple_usb_role_put_adapter(void *data)
{
	i2c_put_adapter(data);
}

static void apple_usb_role_unregister(void *data)
{
	usb_role_switch_unregister(data);
}

static int apple_usb_role_get_pmic(struct apple_usb_role *ur)
{
	struct device_node *np;
	struct i2c_client *client;
	int ret;

	np = of_parse_phandle(ur->dev->of_node, "apple,pmic", 0);
	if (!np)
		return 0;
	client = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!client)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(ur->dev, apple_usb_role_put_device, &client->dev);
	if (ret)
		return ret;
	ur->pmic = dev_get_regmap(&client->dev, NULL);
	if (!ur->pmic)
		return -EPROBE_DEFER;
	if (!device_link_add(ur->dev, &client->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -EINVAL;
	return 0;
}

static int apple_usb_role_get_tristar(struct apple_usb_role *ur)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_fixed_args(ur->dev->of_node, "apple,tristar",
					       1, 0, &args);
	if (ret)
		return ret == -ENOENT ? 0 : ret;
	ur->tristar = of_get_i2c_adapter_by_node(args.np);
	of_node_put(args.np);
	if (!ur->tristar)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(ur->dev, apple_usb_role_put_adapter, ur->tristar);
	if (ret)
		return ret;
	if (args.args[0] > 0x7f)
		return -EINVAL;
	ur->tristar_addr = args.args[0];
	return 0;
}

/*
 * The remap words belong to the parent usb-complex node, whose driver
 * claims them; the glue words may be claimed by a controller node later.
 * Both are mapped without requesting the region.
 */
static int apple_usb_role_get_complex(struct apple_usb_role *ur)
{
	struct device_node *np = ur->dev->of_node;
	struct device_node *complex;
	int index;

	complex = of_get_parent(np);
	if (complex && of_device_is_compatible(complex, "apple,s5l8960x-usb-complex"))
		ur->remap = apple_usb_role_view(ur, complex, 0);
	of_node_put(complex);
	if (!ur->remap)
		return -ENODEV;

	index = of_property_match_string(np, "reg-names", "glue");
	if (index < 0)
		return index;
	ur->glue = apple_usb_role_view(ur, np, index);
	return ur->glue ? 0 : -ENOMEM;
}

static int apple_usb_role_get_udc(struct apple_usb_role *ur)
{
	struct device_node *np;
	int ret;

	np = of_parse_phandle(ur->dev->of_node, "apple,usb-device", 0);
	if (!np)
		return -EINVAL;
	ur->udc = of_find_device_by_node(np);
	if (ur->udc)
		ur->udc_regs = apple_usb_role_view(ur, np, 0);
	of_node_put(np);
	if (!ur->udc)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(ur->dev, apple_usb_role_put_device, &ur->udc->dev);
	return ret;
}

static int apple_usb_role_get_hcs(struct apple_usb_role *ur)
{
	struct device_node *np = ur->dev->of_node;
	int ret;

	ur->ehci_np = of_get_compatible_child(np, "generic-ehci");
	if (!ur->ehci_np)
		return -ENODEV;
	ret = devm_add_action_or_reset(ur->dev, apple_usb_role_put_node, ur->ehci_np);
	if (ret)
		return ret;
	ur->ohci_np = of_get_compatible_child(np, "generic-ohci");
	if (!ur->ohci_np)
		return -ENODEV;
	ret = devm_add_action_or_reset(ur->dev, apple_usb_role_put_node, ur->ohci_np);
	if (ret)
		return ret;
	/* Their power domains are off until host mode: only read when bound. */
	ur->ehci_regs = apple_usb_role_view(ur, ur->ehci_np, 0);
	ur->ohci_regs = apple_usb_role_view(ur, ur->ohci_np, 0);
	return 0;
}

static void apple_usb_role_teardown(void *data)
{
	struct apple_usb_role *ur = data;

	mutex_lock(&apple_usb_role_instance_lock);
	if (apple_usb_role_instance == ur)
		apple_usb_role_instance = NULL;
	mutex_unlock(&apple_usb_role_instance_lock);

	mutex_lock(&ur->lock);
	ur->dying = true;
	ur->revert_pending = false;
	mutex_unlock(&ur->lock);
	cancel_delayed_work_sync(&ur->revert_work);
	debugfs_remove_recursive(ur->debugfs);

	mutex_lock(&ur->lock);
	if (ur->role == USB_ROLE_HOST) {
		apple_usb_role_enter_device(ur);
		WRITE_ONCE(ur->role, USB_ROLE_DEVICE);
	}
	mutex_unlock(&ur->lock);
}

static int apple_usb_role_probe(struct platform_device *pdev)
{
	struct usb_role_switch_desc desc = {};
	struct device *dev = &pdev->dev;
	struct apple_usb_role *ur;
	int ret;

	ur = devm_kzalloc(dev, sizeof(*ur), GFP_KERNEL);
	if (!ur)
		return -ENOMEM;
	ur->dev = dev;
	ur->role = USB_ROLE_DEVICE;
	ur->revert_timeout = revert_timeout;
	/* iOS powers the camera adapter (IOAccessoryActivePowerMode 4). */
	ur->accessory_gates = GATE_PMU;
	platform_set_drvdata(pdev, ur);
	ret = devm_mutex_init(dev, &ur->lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&ur->revert_work, apple_usb_role_revert);

	ur->usb_ctl = devm_platform_ioremap_resource_byname(pdev, "usb-ctl");
	if (IS_ERR(ur->usb_ctl))
		return PTR_ERR(ur->usb_ctl);
	ret = apple_usb_role_get_complex(ur);
	if (ret)
		return dev_err_probe(dev, ret, "usb-complex registers\n");

	ur->phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(ur->phy))
		return dev_err_probe(dev, PTR_ERR(ur->phy), "PHY\n");
	if (ur->phy->dev.parent)
		ur->phy_regs = apple_usb_role_view(ur, ur->phy->dev.parent->of_node, 0);

	ret = apple_usb_role_get_udc(ur);
	if (ret)
		return dev_err_probe(dev, ret, "device controller\n");
	ret = apple_usb_role_get_hcs(ur);
	if (ret)
		return dev_err_probe(dev, ret, "host controller nodes\n");
	ret = apple_usb_role_get_pmic(ur);
	if (ret)
		return dev_err_probe(dev, ret, "PMIC\n");
	if (!ur->pmic)
		ur->accessory_gates = 0;
	ret = apple_usb_role_get_tristar(ur);
	if (ret)
		return dev_err_probe(dev, ret, "Tristar\n");

	/*
	 * No register access here: device mode must keep working exactly as
	 * before even if one of these banks turns out to need a clock.
	 */
	desc.fwnode = dev_fwnode(dev);
	desc.name = "lightning";
	desc.set = apple_usb_role_set;
	desc.get = apple_usb_role_get;
	desc.allow_userspace_control = true;
	desc.driver_data = ur;
	ur->sw = usb_role_switch_register(dev, &desc);
	if (IS_ERR(ur->sw))
		return dev_err_probe(dev, PTR_ERR(ur->sw), "role switch\n");
	ret = devm_add_action_or_reset(dev, apple_usb_role_unregister, ur->sw);
	if (ret)
		return ret;
	/* Runs before the switch goes away: no role change can race it. */
	ret = devm_add_action_or_reset(dev, apple_usb_role_teardown, ur);
	if (ret)
		return ret;

	apple_usb_role_debugfs(ur);
	mutex_lock(&apple_usb_role_instance_lock);
	apple_usb_role_instance = ur;
	mutex_unlock(&apple_usb_role_instance_lock);
	dev_info(dev, "device role; host on request (revert after %u s) or for the Lightning accessory\n",
		 ur->revert_timeout);
	return 0;
}

/*
 * Kexec and reboot: the next kernel's gadget needs the device routing and
 * tunables. The host controllers were already shut down (children go
 * first) and are left registered; only the shared hardware is put back.
 */
static void apple_usb_role_shutdown(struct platform_device *pdev)
{
	struct apple_usb_role *ur = platform_get_drvdata(pdev);

	mutex_lock(&ur->lock);
	ur->dying = true;
	ur->revert_pending = false;
	cancel_delayed_work(&ur->revert_work);
	if (ur->ctl_host) {
		writel(readl(ur->usb_ctl + USB_CTL_HOST) & ~USB_CTL_HOST_EN,
		       ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_host = false;
	}
	if (ur->ctl_phy_on) {
		writel(readl(ur->usb_ctl + USB_CTL_HOST) & ~USB_CTL_PHY_ON,
		       ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_phy_on = false;
	}
	if (ur->cleared_device_enable) {
		writel(readl(ur->usb_ctl + USB_CTL_DEVICE) | USB_CTL_DEVICE_EN,
		       ur->usb_ctl + USB_CTL_DEVICE);
		ur->cleared_device_enable = false;
	}
	if (ur->phy_held)
		phy_set_mode(ur->phy, PHY_MODE_USB_DEVICE);
	apple_usb_role_gates_off(ur);
	mutex_unlock(&ur->lock);
}

static int apple_usb_role_prepare(struct device *dev)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);

	/* Rebinding the gadget is not possible mid-suspend. */
	if (READ_ONCE(ur->role) == USB_ROLE_HOST) {
		dev_warn(dev, "host mode: return to device mode before suspending\n");
		return -EBUSY;
	}
	return 0;
}

static const struct dev_pm_ops apple_usb_role_pm_ops = {
	.prepare = pm_sleep_ptr(apple_usb_role_prepare),
};

static const struct of_device_id apple_usb_role_of_match[] = {
	{ .compatible = "apple,s8000-usb-role-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_usb_role_of_match);

static struct platform_driver apple_usb_role_driver = {
	.probe = apple_usb_role_probe,
	.shutdown = apple_usb_role_shutdown,
	.driver = {
		.name = "apple-s8000-usb-role",
		.of_match_table = apple_usb_role_of_match,
		.dev_groups = apple_usb_role_groups,
		.pm = pm_ptr(&apple_usb_role_pm_ops),
	},
};
module_platform_driver(apple_usb_role_driver);

MODULE_DESCRIPTION("Apple S8000 Lightning port USB role switch");
MODULE_LICENSE("GPL");

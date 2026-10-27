// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/reset-controller.h>
#include <linux/module.h>
#include <linux/soc/apple/pmgr-observe.h>

#define APPLE_PMGR_RESET        BIT(31)
#define APPLE_PMGR_AUTO_ENABLE  BIT(28)
#define APPLE_PMGR_PS_AUTO      GENMASK(27, 24)
#define APPLE_PMGR_PS_MIN       GENMASK(19, 16)
#define APPLE_PMGR_PARENT_OFF   BIT(11)
#define APPLE_PMGR_DEV_DISABLE  BIT(10)
#define APPLE_PMGR_WAS_CLKGATED BIT(9)
#define APPLE_PMGR_WAS_PWRGATED BIT(8)
#define APPLE_PMGR_PS_ACTUAL    GENMASK(7, 4)
#define APPLE_PMGR_PS_TARGET    GENMASK(3, 0)

#define APPLE_PMGR_FLAGS        (APPLE_PMGR_WAS_CLKGATED | APPLE_PMGR_WAS_PWRGATED)

#define APPLE_PMGR_PS_ACTIVE    0xf
#define APPLE_PMGR_PS_CLKGATE   0x4
#define APPLE_PMGR_PS_PWRGATE   0x0

#define APPLE_PMGR_PS_SET_TIMEOUT 100
#define APPLE_PMGR_RESET_TIME 1

/* DT permission is also required, and only the j71t experiment is supported. */
static bool idle_power_gate;
module_param(idle_power_gate, bool, 0444);
MODULE_PARM_DESC(idle_power_gate,
		 "Lower DT-selected j71t domain idle floors (default: off)");

struct apple_pmgr_ps {
	struct device *dev;
	struct generic_pm_domain genpd;
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
	u32 offset;
	u32 min_state;
	bool warm;
	bool keep_on;
	u32 active_min_state;
	bool idle_power_gate;
	bool retain;	/* apple,no-auto-pm: the inherited word is never written */
};

#define genpd_to_apple_pmgr_ps(_genpd) container_of(_genpd, struct apple_pmgr_ps, genpd)
#define rcdev_to_apple_pmgr_ps(_rcdev) container_of(_rcdev, struct apple_pmgr_ps, rcdev)

/* j71t warm entry does not have iBoot's device/domain reset baseline. */
static bool kexec_keep_on = true;
static unsigned int ps_trace;

static int __init apple_pmgr_keep_on_setup(char *arg)
{
	return kstrtobool(arg, &kexec_keep_on);
}
early_param("apple_pmgr.kexec_keep_on", apple_pmgr_keep_on_setup);

static int __init apple_pmgr_trace_setup(char *arg)
{
	unsigned int count;
	int ret = kstrtouint(arg, 0, &count);

	if (!ret)
		WRITE_ONCE(ps_trace, min(count, 1024U));
	return ret;
}
early_param("apple_pmgr.trace", apple_pmgr_trace_setup);

static void apple_pmgr_ps_trace(struct apple_pmgr_ps *ps, const char *stage, u32 reg)
{
	unsigned int left = READ_ONCE(ps_trace);

	while (left) {
		if (try_cmpxchg(&ps_trace, &left, left - 1)) {
			dev_info(ps->dev, "PMGR %s %s @%x reg=%08x warm=%u hold=%u\n",
				 ps->genpd.name, stage, ps->offset, reg,
				 ps->warm, ps->keep_on);
			break;
		}
	}
}

static int apple_pmgr_ps_set(struct generic_pm_domain *genpd, u32 pstate, bool auto_enable)
{
	int ret;
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	u32 reg;

	apple_pmgr_ps_trace(ps, pstate ? "on begin" : "off begin", 0);
	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret < 0)
		return ret;

	/* Resets are synchronous, and only work if the device is powered and clocked. */
	if (reg & APPLE_PMGR_RESET && pstate != APPLE_PMGR_PS_ACTIVE)
		dev_err(ps->dev, "PS %s: powering off with RESET active\n",
			genpd->name);

	reg &= ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS | APPLE_PMGR_PS_TARGET);
	reg |= FIELD_PREP(APPLE_PMGR_PS_TARGET, pstate);

	/* Only genpd can reach this path, after all child/consumer holds drop. */
	if (ps->idle_power_gate) {
		reg &= ~APPLE_PMGR_PS_MIN;
		if (pstate == APPLE_PMGR_PS_ACTIVE)
			reg |= FIELD_PREP(APPLE_PMGR_PS_MIN, ps->active_min_state);
	}

	dev_dbg(ps->dev, "PS %s: pwrstate = 0x%x: 0x%x\n", genpd->name, pstate, reg);

	ret = regmap_write(ps->regmap, ps->offset, reg);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout_atomic(
		ps->regmap, ps->offset, reg,
		(FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == pstate), 1,
		APPLE_PMGR_PS_SET_TIMEOUT);
	if (ret < 0) {
		dev_err(ps->dev, "PS %s @%x: Failed to reach power state 0x%x (now: 0x%x, error %d)\n",
			genpd->name, ps->offset, pstate, reg, ret);
		/* Never enable AUTO on a failed handshake. */
		return ret;
	}

	if (auto_enable && !ps->keep_on) {
		/* Not all devices implement this; this is a no-op where not implemented. */
		reg &= ~APPLE_PMGR_FLAGS;
		reg |= APPLE_PMGR_AUTO_ENABLE;
		ret = regmap_write(ps->regmap, ps->offset, reg);
	}

	apple_pmgr_ps_trace(ps, "set end", reg);
	return ret;
}

/* A kexec preserves the faulted GPU. Cycle its ADT-identified outer domain
 * before publishing the provider, hence before a GPU consumer can load firmware.
 * Explicit opt-in until this has been validated on hardware. No GPU MMIO here.
 */
static int apple_pmgr_ps_power_on(struct generic_pm_domain *genpd);
static int apple_pmgr_ps_power_off(struct generic_pm_domain *genpd);
static bool kexec_gpu_cycle;

static int __init apple_pmgr_kexec_gpu_cycle(char *arg)
{
	return kstrtobool(arg, &kexec_gpu_cycle);
}
early_param("apple_pmgr.kexec_gpu_cycle", apple_pmgr_kexec_gpu_cycle);

static int apple_pmgr_ps_kexec_gpu_cycle(struct apple_pmgr_ps *ps, bool *active)
{
	u32 reg;
	int ret;

	if (!kexec_gpu_cycle || !of_machine_is_compatible("apple,j71t") ||
	    ps->offset != 0x80340 || strcmp(ps->genpd.name, "gfx") ||
	    !of_device_is_compatible(ps->dev->of_node, "apple,s8000-pmgr-pwrstate") ||
	    !of_property_read_bool(of_chosen, "linux,booted-from-kexec"))
		return 0;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret)
		return ret;
	/* A non-zero floor or reset held by the old kernel prevents a real off.
	 * Refuse rather than silently calling a clock gate a power cycle.
	 */
	if (reg & (APPLE_PMGR_PS_MIN | APPLE_PMGR_RESET))
		return dev_err_probe(ps->dev, -EBUSY,
			"kexec GFX cycle refused: min/reset set (%08x)\n", reg);

	dev_info(ps->dev, "kexec GFX power cycle start: %08x\n", reg);
	/* Explicit recovery is allowed even when ordinary warm gating is held. */
	ret = apple_pmgr_ps_set(&ps->genpd, APPLE_PMGR_PS_PWRGATE, false);
	if (ret)
		return ret;
	udelay(10);
	ret = apple_pmgr_ps_power_on(&ps->genpd);
	if (ret)
		return ret;
	*active = true;
	dev_info(ps->dev, "kexec GFX power cycle complete (actual 0 -> f)\n");
	return 0;
}

static int apple_pmgr_ps_is_active(struct apple_pmgr_ps *ps, bool *active)
{
	u32 reg = 0;
	int ret;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret)
		return ret;
	apple_pmgr_ps_trace(ps, "inherited", reg);
	/*
	 * We consider domains as active if they are actually on, or if they have auto-PM
	 * enabled and the intended target is on.
	 */
	*active = (FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE ||
		(FIELD_GET(APPLE_PMGR_PS_TARGET, reg) == APPLE_PMGR_PS_ACTIVE &&
		 reg & APPLE_PMGR_AUTO_ENABLE));
	return 0;
}

static int apple_pmgr_ps_power_on(struct generic_pm_domain *genpd)
{
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	u32 reg;
	int ret;

	/* A retained domain is only ever reported, never requested. */
	if (ps->retain) {
		ret = regmap_read(ps->regmap, ps->offset, &reg);
		if (ret)
			return ret;
		if (FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE)
			return 0;
		dev_err(ps->dev, "PS %s: retained domain is off (%08x), not powering on\n",
			genpd->name, reg);
		return -EPERM;
	}
	return apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_ACTIVE, true);
}

/*
 * apple,no-auto-pm (j71t ps_sep): keep the word the bootloader chain left
 * exactly as it is. pongoOS hands sepOS over with SEP at 0x4f0000ff; the cold
 * path below would add APPLE_PMGR_AUTO_ENABLE (0x5f0000ff), and with that set
 * sepOS stopped consuming the AP->SEP mailbox (2026-10-26 15:42). No AUTO, no
 * floor, no warm re-confirm, no power-off and no reset: the register is only
 * read. A domain inherited off stays off and is published without ALWAYS_ON.
 */
static int apple_pmgr_ps_init_retained(struct apple_pmgr_ps *ps, bool *active)
{
	u32 reg;
	int ret;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret)
		return ret;
	*active = FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE;
	if (!*active) {
		dev_err(ps->dev, "PMGR %s retained @%x reg=%08x is off: left untouched, not always-on\n",
			ps->genpd.name, ps->offset, reg);
		return 0;
	}
	if (of_property_read_bool(ps->dev->of_node, "apple,always-on"))
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
	dev_info(ps->dev, "PMGR %s retained @%x reg=%08x: no AUTO, never written\n",
		 ps->genpd.name, ps->offset, reg);
	return 0;
}

static int apple_pmgr_ps_init_state(struct apple_pmgr_ps *ps, bool *active)
{
	struct device_node *node = ps->dev->of_node;
	int ret;

	if (ps->retain)
		return apple_pmgr_ps_init_retained(ps, active);

	ret = apple_pmgr_ps_is_active(ps, active);
	if (ret)
		return ret;
	/* Actual ON with target OFF is not a stable ON request. AUTO may
	 * also have gated an intended ON domain. Confirm ON before publishing.
	 */
	if (ps->warm && *active) {
		ret = apple_pmgr_ps_power_on(&ps->genpd);
		if (ret)
			return ret;
	}
	if (of_property_read_bool(node, "apple,always-on")) {
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		if (!*active) {
			dev_warn(ps->dev, "always-on domain %s is not on at boot\n",
				 ps->genpd.name);
			ret = apple_pmgr_ps_power_on(&ps->genpd);
			if (ret)
				return ret;
			*active = true;
		}
	}
	/* Preserve the cold boot sequence, including AUTO for active domains. */
	if (*active && !ps->warm)
		return regmap_update_bits(ps->regmap, ps->offset,
				APPLE_PMGR_FLAGS | APPLE_PMGR_AUTO_ENABLE,
				APPLE_PMGR_AUTO_ENABLE);
	return 0;
}

static int apple_pmgr_ps_power_off(struct generic_pm_domain *genpd)
{
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	int ret, restore;

	if (ps->retain) {
		apple_pmgr_ps_trace(ps, "off refused (retained)", 0);
		return -EBUSY;
	}

	/* Do not gate retained firmware/DMA before its driver has reset it.
	 * A failed off keeps genpd ON, including parent use counts. Domains
	 * inherited OFF remain OFF until a consumer requests power normally.
	 */
	if (ps->keep_on) {
		apple_pmgr_ps_trace(ps, "off held", 0);
		return -EBUSY;
	}

	ret = apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_PWRGATE, false);
	if (ret && ps->idle_power_gate) {
		/* Keep genpd's failed-off state consistent with powered hardware. */
		restore = apple_pmgr_ps_power_on(genpd);
		if (restore)
			dev_err(ps->dev, "idle gate rollback failed: %d\n", restore);
	}
	return ret;
}

static int apple_pmgr_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;
	int ret;

	if (ps->retain) {
		dev_err(ps->dev, "PS 0x%x: reset refused on a retained domain\n", ps->offset);
		return -EPERM;
	}

	spin_lock_irqsave(&ps->genpd.slock, flags);

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err(ps->dev, "PS 0x%x: asserting RESET while powered down\n", ps->offset);

	dev_dbg(ps->dev, "PS 0x%x: assert reset\n", ps->offset);
	/* Quiesce device before asserting reset */
	ret = regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE,
			   APPLE_PMGR_DEV_DISABLE);
	if (ret)
		goto out;
	ret = regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET,
			   APPLE_PMGR_RESET);

out:
	spin_unlock_irqrestore(&ps->genpd.slock, flags);

	return ret;
}

static int apple_pmgr_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;
	int ret;

	if (ps->retain) {
		dev_err(ps->dev, "PS 0x%x: reset refused on a retained domain\n", ps->offset);
		return -EPERM;
	}

	spin_lock_irqsave(&ps->genpd.slock, flags);

	dev_dbg(ps->dev, "PS 0x%x: deassert reset\n", ps->offset);
	ret = regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET, 0);
	if (ret)
		goto out;
	ret = regmap_update_bits(ps->regmap, ps->offset,
				 APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE, 0);

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err(ps->dev, "PS 0x%x: RESET was deasserted while powered down\n", ps->offset);

out:
	spin_unlock_irqrestore(&ps->genpd.slock, flags);

	return ret;
}

static int apple_pmgr_reset_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	int ret;

	ret = apple_pmgr_reset_assert(rcdev, id);
	if (ret)
		return ret;

	usleep_range(APPLE_PMGR_RESET_TIME, 2 * APPLE_PMGR_RESET_TIME);

	return apple_pmgr_reset_deassert(rcdev, id);
}

static int apple_pmgr_reset_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	u32 reg = 0;
	int ret;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret)
		return ret;

	return !!(reg & APPLE_PMGR_RESET);
}

static const struct reset_control_ops apple_pmgr_reset_ops = {
	.assert		= apple_pmgr_reset_assert,
	.deassert	= apple_pmgr_reset_deassert,
	.reset		= apple_pmgr_reset_reset,
	.status		= apple_pmgr_reset_status,
};

static int apple_pmgr_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *reset_spec)
{
	return 0;
}

static int apple_pmgr_ps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_pmgr_ps *ps;
	struct regmap *regmap;
	struct of_phandle_iterator it;
	int ret;
	const char *name;
	bool active;
	u32 reg;

	regmap = syscon_node_to_regmap(node->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ps = devm_kzalloc(dev, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return -ENOMEM;

	ps->dev = dev;
	ps->regmap = regmap;

	ret = of_property_read_string(node, "label", &name);
	if (ret < 0) {
		dev_err(dev, "missing label property\n");
		return ret;
	}

	ret = of_property_read_u32(node, "reg", &ps->offset);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	/*
	 * Keep a domain on over system sleep while a device in it is on the
	 * wakeup path (e.g. a USB gadget kept connected in s2idle).
	 */
	ps->genpd.flags |= GENPD_FLAG_IRQ_SAFE | GENPD_FLAG_ACTIVE_WAKEUP;
	ps->genpd.name = name;
	ps->genpd.power_on = apple_pmgr_ps_power_on;
	ps->genpd.power_off = apple_pmgr_ps_power_off;
	ps->warm = of_machine_is_compatible("apple,j71t") &&
		of_device_is_compatible(node, "apple,s8000-pmgr-pwrstate") &&
		of_property_read_bool(of_chosen, "linux,booted-from-kexec");
	ps->keep_on = ps->warm && kexec_keep_on;
	ps->retain = of_property_read_bool(node, "apple,no-auto-pm");
	apple_pmgr_ps_trace(ps, "probe begin", 0);
	if (READ_ONCE(ps_trace)) {
		u32 inherited;

		ret = regmap_read(regmap, ps->offset, &inherited);
		if (ret)
			return ret;
		apple_pmgr_ps_trace(ps, "raw before floor", inherited);
	}

	ret = of_property_read_u32(node, "apple,min-state", &ps->min_state);
	if (ret == 0 && ps->min_state <= APPLE_PMGR_PS_ACTIVE && !ps->retain) {
		ret = regmap_update_bits(regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_PS_MIN,
				   FIELD_PREP(APPLE_PMGR_PS_MIN, ps->min_state));
		if (ret)
			return ret;
	}

	if (idle_power_gate && of_machine_is_compatible("apple,j71t") &&
	    of_device_is_compatible(node, "apple,s8000-pmgr-pwrstate") &&
	    of_property_read_bool(node, "apple,idle-power-gate") &&
	    !of_property_read_bool(node, "apple,always-on") && !ps->retain) {
		ret = regmap_read(regmap, ps->offset, &reg);
		if (ret)
			return ret;
		ps->active_min_state = FIELD_GET(APPLE_PMGR_PS_MIN, reg);
		ps->idle_power_gate = true;
		dev_info(dev, "idle floor opt-in: %s minimum %u -> 0 only at genpd off\n",
			 name, ps->active_min_state);
	}

	ret = apple_pmgr_ps_init_state(ps, &active);
	if (ret)
		return ret;

	/* Keep boot recovery independent of the idle-floor policy above. */
	ret = apple_pmgr_ps_kexec_gpu_cycle(ps, &active);
	if (ret)
		return ret;

	ret = pm_genpd_init(&ps->genpd, NULL, !active);
	if (ret < 0) {
		dev_err(dev, "pm_genpd_init failed\n");
		return ret;
	}

	ret = of_genpd_add_provider_simple(node, &ps->genpd);
	if (ret < 0) {
		dev_err(dev, "of_genpd_add_provider_simple failed\n");
		return ret;
	}

	of_for_each_phandle(&it, ret, node, "power-domains", "#power-domain-cells", -1) {
		struct of_phandle_args parent, child;

		parent.np = it.node;
		parent.args_count = of_phandle_iterator_args(&it, parent.args, MAX_PHANDLE_ARGS);
		child.np = node;
		child.args_count = 0;
		ret = of_genpd_add_subdomain(&parent, &child);

		if (ret == -EPROBE_DEFER) {
			of_node_put(parent.np);
			goto err_remove;
		} else if (ret < 0) {
			dev_err(dev, "failed to add to parent domain: %d (%s -> %s)\n",
				ret, it.node->name, node->name);
			of_node_put(parent.np);
			goto err_remove;
		}
	}

	/*
	 * Do not participate in regular PM; parent power domains are handled via the
	 * genpd hierarchy.
	 */
	pm_genpd_remove_device(dev);

	ps->rcdev.owner = THIS_MODULE;
	ps->rcdev.nr_resets = 1;
	ps->rcdev.ops = &apple_pmgr_reset_ops;
	ps->rcdev.of_node = dev->of_node;
	ps->rcdev.of_reset_n_cells = 0;
	ps->rcdev.of_xlate = apple_pmgr_reset_xlate;

	ret = devm_reset_controller_register(dev, &ps->rcdev);
	if (ret < 0)
		goto err_remove;

	platform_set_drvdata(pdev, ps);

	return 0;
err_remove:
	of_genpd_del_provider(node);
	pm_genpd_remove(&ps->genpd);
	return ret;
}

/* A9 AOP observer API: BEGIN */
#if IS_ENABLED(CONFIG_APPLE_A9_AOP)
/*
 * Read-only observer for bring-up. The caller's nodes follow the same
 * child -> parent order as genpd power transitions. No domain is attached,
 * resumed, pinned on, reset, or written by this function.
 */
int apple_pmgr_observe(struct device_node **nodes, unsigned int count,
		       struct apple_pmgr_observation *states,
		       void (*read)(void *), void *data)
{
	struct platform_device *providers[5];
	struct apple_pmgr_ps *ps[5];
	unsigned long flags;
	unsigned int i, j, found = 0;
	int ret = 0;
	bool accessible = true;

	if (!count || count > ARRAY_SIZE(providers) || !read)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		for (j = 0; j < i; j++)
			if (nodes[i] == nodes[j]) {
				ret = -EINVAL;
				goto put;
			}
		providers[i] = of_find_device_by_node(nodes[i]);
		if (!providers[i]) {
			ret = -EPROBE_DEFER;
			goto put;
		}
		device_lock(&providers[i]->dev);
		found++;
		if (!providers[i]->dev.driver ||
		    strcmp(providers[i]->dev.driver->name, "apple-pmgr-pwrstate")) {
			ret = -EPROBE_DEFER;
			goto put;
		}
		ps[i] = platform_get_drvdata(providers[i]);
		if (!ps[i]) {
			ret = -EPROBE_DEFER;
			goto put;
		}
	}

	/* PMGR domains use GENPD_FLAG_IRQ_SAFE, i.e. genpd.slock. */
	local_irq_save(flags);
	for (i = 0; i < count; i++)
		spin_lock_nested(&ps[i]->genpd.slock, i);
	for (i = 0; i < count; i++) {
		states[i].genpd_status = ps[i]->genpd.status;
		ret = regmap_read(ps[i]->regmap, ps[i]->offset, &states[i].raw);
		if (ret)
			goto unlock;
		if (states[i].genpd_status != GENPD_STATE_ON ||
		    FIELD_GET(APPLE_PMGR_PS_ACTUAL, states[i].raw) != APPLE_PMGR_PS_ACTIVE ||
		    FIELD_GET(APPLE_PMGR_PS_TARGET, states[i].raw) != APPLE_PMGR_PS_ACTIVE ||
		    states[i].raw & (APPLE_PMGR_RESET | APPLE_PMGR_DEV_DISABLE |
				     APPLE_PMGR_PARENT_OFF))
			accessible = false;
	}
	if (accessible)
		read(data);
	else
		ret = -EHOSTDOWN;
unlock:
	for (i = count; i > 0; i--)
		spin_unlock(&ps[i - 1]->genpd.slock);
	local_irq_restore(flags);
put:
	while (found) {
		found--;
		device_unlock(&providers[found]->dev);
		put_device(&providers[found]->dev);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmgr_observe);
#endif
/* A9 AOP observer API: END */

static const struct of_device_id apple_pmgr_ps_of_match[] = {
	{ .compatible = "apple,t8103-pmgr-pwrstate" },
	{ .compatible = "apple,pmgr-pwrstate" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_ps_of_match);

static struct platform_driver apple_pmgr_ps_driver = {
	.probe = apple_pmgr_ps_probe,
	.driver = {
		.name = "apple-pmgr-pwrstate",
		.of_match_table = apple_pmgr_ps_of_match,
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR power state driver for Apple SoCs");

module_platform_driver(apple_pmgr_ps_driver);

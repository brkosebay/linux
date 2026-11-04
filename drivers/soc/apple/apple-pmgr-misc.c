// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define APPLE_CLKGEN_PSTATE 0
#define APPLE_CLKGEN_PSTATE_DESIRED GENMASK(3, 0)
#define APPLE_CLKGEN_PSTATE_PENDING BIT(31)

#define DCS_DEV_PSTATE_MIN_T600X 7
#define SYS_DEV_PSTATE_SUSPEND 1

/*
 * S8000/S8003 (A9) SoC and memory state. iOS's PMGR driver maps the SoC
 * domain's performance index i to state 13 - i, so 13 is the state the
 * system boots in and 12 the fast one (memory and fabric clocks together).
 * Each state has a record at 0x10 * state; byte 1 of its word at 0x18 is
 * the SoC rail's voltage code, which PMGR's voltage sequencer applies by
 * itself on a state change: iOS writes the state and waits for the change
 * to complete, nothing else. The sequencer runs as iOS runs it only with
 * its hold bits clear and the routing the boot firmware set up; the live
 * SoC code then follows the state's record, which is checked after every
 * change.
 */
#define S8000_SOC_PSTATE_FAST		12
#define S8000_SOC_PSTATE_SLOW		13
#define S8000_SOC_RECORD(state)		(0x18 + 0x10 * (state))
#define S8000_SOC_RECORD_CODE		GENMASK(15, 8)
#define S8000_VOLMAN_CTL		0x0
#define S8000_VOLMAN_CTL_HOLD		GENMASK(12, 8)
#define S8000_VOLMAN_ROUTING		0x4
#define S8000_VOLMAN_ROUTING_BOOT	0x1782
#define S8000_VOLMAN_SOC_CODE		0x8
#define S8000_VOLMAN_SOC_CODE_MASK	GENMASK(7, 0)
#define S8000_PENDING_TIMEOUT_US	8000	/* iOS: 0x2ee00 ticks of 24 MHz */
#define S8000_CODE_TIMEOUT_US		20000

static unsigned int s8000_soc_pstate = S8000_SOC_PSTATE_FAST;
module_param(s8000_soc_pstate, uint, 0444);
MODULE_PARM_DESC(s8000_soc_pstate,
		 "A9 SoC/memory state at boot: 12 (fast, default) or 13 (as booted); 0 leaves it alone");

enum sys_device {
	DEV_FABRIC,
	DEV_DCS,
	DEV_SOC,
	DEV_MAX,
};

static const char *const sys_device_names[DEV_MAX] = {
	[DEV_FABRIC] = "fabric",
	[DEV_DCS] = "dcs",
	[DEV_SOC] = "soc",
};

struct apple_pmgr_sys_device {
	void __iomem *base;
	u32 active_state;
	u32 suspend_state;
};

struct apple_pmgr_misc_hw {
	unsigned long devices;		/* BIT(enum sys_device) */
	u32 dev_min_ps[DEV_MAX];
	bool s8000_soc;			/* SoC state through the checked A9 sequence */
};

struct apple_pmgr_misc {
	struct device *dev;
	const struct apple_pmgr_misc_hw *hw;
	struct apple_pmgr_sys_device devices[DEV_MAX];
	void __iomem *volman;
	struct mutex lock;		/* SoC state changes */
};

static void apple_pmgr_sys_dev_set_pstate(struct apple_pmgr_misc *misc,
					  enum sys_device dev, bool active)
{
	u32 pstate;
	u32 val;

	if (!misc->devices[dev].base)
		return;

	if (active)
		pstate = misc->devices[dev].active_state;
	else
		pstate = misc->devices[dev].suspend_state;

	dev_dbg(misc->dev, "set %d ps to pstate %d\n", dev, pstate);

	val = readl_relaxed(misc->devices[dev].base + APPLE_CLKGEN_PSTATE);
	FIELD_MODIFY(APPLE_CLKGEN_PSTATE_DESIRED, &val, pstate);
	writel_relaxed(val, misc->devices[dev].base + APPLE_CLKGEN_PSTATE);
}

static u32 s8000_soc_code(struct apple_pmgr_misc *misc, u32 state)
{
	return FIELD_GET(S8000_SOC_RECORD_CODE,
			 readl_relaxed(misc->devices[DEV_SOC].base + S8000_SOC_RECORD(state)));
}

static u32 s8000_soc_live_code(struct apple_pmgr_misc *misc)
{
	return FIELD_GET(S8000_VOLMAN_SOC_CODE_MASK,
			 readl_relaxed(misc->volman + S8000_VOLMAN_SOC_CODE));
}

/* One read-modify-write of the state, then wait for the change to complete */
static int s8000_soc_write(struct apple_pmgr_misc *misc, u32 state)
{
	void __iomem *reg = misc->devices[DEV_SOC].base + APPLE_CLKGEN_PSTATE;
	u32 val;

	val = readl_relaxed(reg);
	FIELD_MODIFY(APPLE_CLKGEN_PSTATE_DESIRED, &val, state);
	writel_relaxed(val, reg);
	return readl_relaxed_poll_timeout_atomic(reg, val, !(val & APPLE_CLKGEN_PSTATE_PENDING),
						 1, S8000_PENDING_TIMEOUT_US);
}

static int s8000_soc_set_pstate(struct apple_pmgr_misc *misc, u32 state)
{
	void __iomem *reg = misc->devices[DEV_SOC].base + APPLE_CLKGEN_PSTATE;
	u32 val, cur, ctl, routing, fast, slow, code;
	int ret;

	lockdep_assert_held(&misc->lock);

	if (state != S8000_SOC_PSTATE_FAST && state != S8000_SOC_PSTATE_SLOW)
		return -EINVAL;

	val = readl_relaxed(reg);
	cur = FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED, val);
	if (val & APPLE_CLKGEN_PSTATE_PENDING) {
		dev_warn(misc->dev, "soc-perf: a change is pending (%#x)\n", val);
		return -EBUSY;
	}
	if (cur != S8000_SOC_PSTATE_FAST && cur != S8000_SOC_PSTATE_SLOW) {
		dev_warn(misc->dev, "soc-perf: unexpected state %u\n", cur);
		return -ENODEV;
	}
	if (cur == state)
		return 0;

	ctl = readl_relaxed(misc->volman + S8000_VOLMAN_CTL);
	routing = readl_relaxed(misc->volman + S8000_VOLMAN_ROUTING);
	fast = s8000_soc_code(misc, S8000_SOC_PSTATE_FAST);
	slow = s8000_soc_code(misc, S8000_SOC_PSTATE_SLOW);
	code = s8000_soc_live_code(misc);
	if ((ctl & S8000_VOLMAN_CTL_HOLD) || routing != S8000_VOLMAN_ROUTING_BOOT ||
	    fast <= slow || code != (cur == S8000_SOC_PSTATE_FAST ? fast : slow)) {
		dev_warn(misc->dev,
			 "soc-perf: voltage sequencer not as iOS runs it (ctl %#x routing %#x codes %u/%u live %u), staying in %u\n",
			 ctl, routing, fast, slow, code, cur);
		return -ENODEV;
	}

	ret = s8000_soc_write(misc, state);
	if (ret) {
		dev_err(misc->dev, "soc-perf: state %u -> %u did not complete (%#x)\n",
			cur, state, readl_relaxed(reg));
		return ret;
	}

	code = state == S8000_SOC_PSTATE_FAST ? fast : slow;
	ret = readl_relaxed_poll_timeout_atomic(misc->volman + S8000_VOLMAN_SOC_CODE, val,
						FIELD_GET(S8000_VOLMAN_SOC_CODE_MASK, val) == code,
						1, S8000_CODE_TIMEOUT_US);
	if (ret) {
		dev_err(misc->dev, "soc-perf: live code %u did not reach %u, back to %u\n",
			s8000_soc_live_code(misc), code, cur);
		if (s8000_soc_write(misc, cur))
			dev_err(misc->dev, "soc-perf: return to %u did not complete\n", cur);
		return -EIO;
	}

	return 0;
}

/* The state in force and whether the live SoC code matches its record */
static void s8000_soc_report(struct apple_pmgr_misc *misc)
{
	u32 state = FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED,
			      readl_relaxed(misc->devices[DEV_SOC].base + APPLE_CLKGEN_PSTATE));
	u32 code = s8000_soc_live_code(misc);
	bool ok = (state == S8000_SOC_PSTATE_FAST || state == S8000_SOC_PSTATE_SLOW) &&
		  code == s8000_soc_code(misc, state);

	dev_info(misc->dev, "soc-perf state=%u code=%u %s\n", state, code, ok ? "ok" : "MISMATCH");
}

static int s8000_soc_apply(struct apple_pmgr_misc *misc, bool active)
{
	struct apple_pmgr_sys_device *soc = &misc->devices[DEV_SOC];
	int ret;

	if (!soc->base || !soc->active_state)
		return 0;

	mutex_lock(&misc->lock);
	ret = s8000_soc_set_pstate(misc, active ? soc->active_state : soc->suspend_state);
	mutex_unlock(&misc->lock);
	return ret;
}

static int __maybe_unused apple_pmgr_misc_suspend_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < DEV_MAX; i++) {
		if (i == DEV_SOC && misc->hw->s8000_soc)
			s8000_soc_apply(misc, false);
		else
			apple_pmgr_sys_dev_set_pstate(misc, i, false);
	}

	return 0;
}

static int __maybe_unused apple_pmgr_misc_resume_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < DEV_MAX; i++) {
		if (i == DEV_SOC && misc->hw->s8000_soc)
			s8000_soc_apply(misc, true);
		else
			apple_pmgr_sys_dev_set_pstate(misc, i, true);
	}

	return 0;
}

static ssize_t soc_pstate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	u32 val = readl_relaxed(misc->devices[DEV_SOC].base + APPLE_CLKGEN_PSTATE);

	return sysfs_emit(buf, "%lu\n", FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED, val));
}

static ssize_t soc_pstate_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	unsigned int state;
	int ret;

	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;
	if (state != S8000_SOC_PSTATE_FAST && state != S8000_SOC_PSTATE_SLOW)
		return -EINVAL;

	mutex_lock(&misc->lock);
	ret = s8000_soc_set_pstate(misc, state);
	if (!ret)
		misc->devices[DEV_SOC].active_state = state;
	s8000_soc_report(misc);
	mutex_unlock(&misc->lock);

	return ret ?: count;
}
static DEVICE_ATTR_RW(soc_pstate);

static struct attribute *apple_pmgr_misc_attrs[] = {
	&dev_attr_soc_pstate.attr,
	NULL
};

static umode_t apple_pmgr_misc_attr_visible(struct kobject *kobj, struct attribute *attr, int n)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(kobj_to_dev(kobj));

	return misc && misc->hw->s8000_soc ? attr->mode : 0;
}

static const struct attribute_group apple_pmgr_misc_group = {
	.attrs = apple_pmgr_misc_attrs,
	.is_visible = apple_pmgr_misc_attr_visible,
};
__ATTRIBUTE_GROUPS(apple_pmgr_misc);

static int apple_pmgr_init_device(struct apple_pmgr_misc *misc,
				  const struct apple_pmgr_misc_hw *hw,
				  enum sys_device dev)
{
	void __iomem *base;
	char name[32];
	u32 val;

	snprintf(name, sizeof(name), "%s-ps", sys_device_names[dev]);

	base = devm_platform_ioremap_resource_byname(
		to_platform_device(misc->dev), name);
	if (IS_ERR(base))
		return PTR_ERR(base);

	val = readl_relaxed(base + APPLE_CLKGEN_PSTATE);

	misc->devices[dev].base = base;
	misc->devices[dev].active_state =
		FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED, val);
	misc->devices[dev].suspend_state = hw->dev_min_ps[dev];

	return 0;
}

static int apple_pmgr_misc_s8000_init(struct apple_pmgr_misc *misc)
{
	struct apple_pmgr_sys_device *soc = &misc->devices[DEV_SOC];

	misc->volman = devm_platform_ioremap_resource_byname(to_platform_device(misc->dev),
							     "volman");
	if (IS_ERR(misc->volman))
		return PTR_ERR(misc->volman);

	if (s8000_soc_pstate != S8000_SOC_PSTATE_FAST &&
	    s8000_soc_pstate != S8000_SOC_PSTATE_SLOW) {
		if (s8000_soc_pstate)
			dev_warn(misc->dev, "soc-perf: state %u is not 12 or 13, left alone\n",
				 s8000_soc_pstate);
		soc->active_state = 0;	/* left alone, also across suspend */
		s8000_soc_report(misc);
		return 0;
	}

	/* A refused or failed change keeps the boot state; the driver stays bound */
	mutex_lock(&misc->lock);
	if (!s8000_soc_set_pstate(misc, s8000_soc_pstate))
		soc->active_state = s8000_soc_pstate;
	s8000_soc_report(misc);
	mutex_unlock(&misc->lock);

	return 0;
}

static int apple_pmgr_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct apple_pmgr_misc_hw *hw;
	struct apple_pmgr_misc *misc;
	int ret, i;

	misc = devm_kzalloc(dev, sizeof(*misc), GFP_KERNEL);
	if (!misc)
		return -ENOMEM;

	misc->dev = dev;
	hw = of_device_get_match_data(dev);
	if (!hw)
		return -EINVAL;
	misc->hw = hw;

	ret = devm_mutex_init(dev, &misc->lock);
	if (ret)
		return ret;

	for (i = 0; i < DEV_MAX; i++) {
		if (!(hw->devices & BIT(i)))
			continue;
		ret = apple_pmgr_init_device(misc, hw, i);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, misc);

	if (hw->s8000_soc)
		return apple_pmgr_misc_s8000_init(misc);

	return 0;
}

static const struct apple_pmgr_misc_hw apple_pmgr_misc_hw_t600x = {
	.devices = BIT(DEV_FABRIC) | BIT(DEV_DCS),
	.dev_min_ps = {
		[DEV_FABRIC] = SYS_DEV_PSTATE_SUSPEND,
		[DEV_DCS] = DCS_DEV_PSTATE_MIN_T600X,
	},
};

static const struct apple_pmgr_misc_hw apple_pmgr_misc_hw_t602x = {
	.devices = BIT(DEV_FABRIC) | BIT(DEV_DCS),
	.dev_min_ps = {
		[DEV_FABRIC] = SYS_DEV_PSTATE_SUSPEND,
		[DEV_DCS] = SYS_DEV_PSTATE_SUSPEND,
	},
};

static const struct apple_pmgr_misc_hw apple_pmgr_misc_hw_s8000 = {
	.devices = BIT(DEV_SOC),
	.dev_min_ps = {
		[DEV_SOC] = S8000_SOC_PSTATE_SLOW,
	},
	.s8000_soc = true,
};

static const struct of_device_id apple_pmgr_misc_of_match[] = {
	{ .compatible = "apple,s8000-pmgr-misc", .data = &apple_pmgr_misc_hw_s8000 },
	{ .compatible = "apple,t6000-pmgr-misc", .data = &apple_pmgr_misc_hw_t600x },
	{ .compatible = "apple,t6020-pmgr-misc", .data = &apple_pmgr_misc_hw_t602x },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_misc_of_match);

static const struct dev_pm_ops apple_pmgr_misc_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(apple_pmgr_misc_suspend_noirq,
				      apple_pmgr_misc_resume_noirq)
};

static struct platform_driver apple_pmgr_misc_driver = {
	.probe = apple_pmgr_misc_probe,
	.driver = {
		.name = "apple-pmgr-misc",
		.of_match_table = apple_pmgr_misc_of_match,
		.dev_groups = apple_pmgr_misc_groups,
		.pm = pm_ptr(&apple_pmgr_misc_pm_ops),
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR misc driver for Apple SoCs");
MODULE_LICENSE("GPL");

module_platform_driver(apple_pmgr_misc_driver);

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Shared private bring-up implementation for mutually exclusive frontends. */
#include <crypto/sha2.h>
#include <clocksource/arm_arch_timer.h>
#include <linux/vmalloc.h>
#include <linux/bitmap.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
/* perf60 clock: which GPC states license an R0 access (a bell, a TLB pass, a register sample). */
/* release8 default */

/* The GPC is powered and not mid-transition: state 1 only, or (gpc_any_pstate) any nonzero
 * pstate nibble with no request in flight.  A parked GPC (nibble 0) never qualifies.
 */
static inline bool g5p_gpc_ok(u32 raw)
{
	;
	;
	return !(raw & 0x10) && (raw & 0xf);
}

/* The pure headers (readback.h, r74e112.h, render-user-wake.h, dirtystart108.h) judge the GPC state
 * through G5P_GPC_ON(); their default is the old == 1, for the host tests.  The kernel takes the rule.
 */
#define G5P_GPC_ON(raw) g5p_gpc_ok(raw)

#include "pack.h"
#include "work-layout.h"
#include "mmio-licence.h"
#include "gpu-oc.h"
#include <linux/thermal.h>
#include "readback.h"

/* nopatch: the TYPED_V3 boot path (r74/r74f/r74g, meta6c8=1) on Apple's original image.  The release8
 * default since its device pass (2026-11-03, release8-nopatch a10a8556b9): an explicit firmware patch
 * request (boot_r73f_case 1..4, or the r75/r79 cases that ride on it) takes the patched path instead,
 * so the patched fallback is boot_r73f_case 1 alone (g5p_fw_unpatched_want()).
 */
/* release8 default */

/* The effective nopatch latch: the parameter, unless a patched image is asked for. */
static u32 g5p_fw_unpatched_want(u32 param, u32 r73f, u32 r75, u32 r79)
{
	return param == 1 && !r73f && !r75 && !r79;
}

#define G5P_MSLVDATAX 0x0a00
#define G5P_MSLVDATAT 0x0a08
#define G5P_MSLVCTRL0 0x0a10
#define G5P_MSLVCTRL1 0x0a18
#define G5P_MSLVIRQSTATUS 0x0ac8
#define G5P_SP_READY (BIT(24) | BIT(26))
#define G5P_FW_NAME "apple/agx-g5p-c0.ldr"

struct g5p_boot;

struct g5p_diag {
	struct g5p_diagnostics *diagnostics;
	struct g5p_boot *boot;

	spinlock_t slot_lock;
	DECLARE_BITMAP(user_slots, 256);
	struct device *dev;
	void __iomem *window[2];
	void __iomem *rgx;
	struct regmap *pmgr;
	/* Serializes commands and all hardware access. */
	struct mutex lock;
	bool powered;
	bool loaded;

	struct g5p_ldr_image image;
	bool fender_ready;
	bool id_checked;
	int last_result;
	u32 words_written;
	u32 core_id, core_revision;
	u32 last_sp_addr;
	bool reset_transition;
	void (*event_notify)(void *cookie, const u32 words[11]);
	void *event_cookie;
	/* The GPU register licence (mmio.inc, docs/gpu/experiments/mmio-access.md §4).  mmio_lock is
	 * a leaf: taken inside event_lock where both are held, never the reverse, nothing inside.
	 * mmio_off forbids R1/R0 through the accessor from before any PMGR gfx gate until Fender is
	 * up again (g5p_mmio_gate(), cleared by g5p_fender_wake_once()).
	 */
	spinlock_t mmio_lock;
	bool mmio_off;
	u32 mmio_max_pstate;
	struct g5p_mmio_self *mmio_self;
};

/* Before every PMGR gfx gate (boot R33 off, dirty-start, kexec quiesce and in-boot recovery
 * cycles): no accessor MMIO from here until g5p_fender_wake_once() brings Fender up again.
 * Apple clears fender+0xc inside setPlatformPower(0), before its gate (G5).
 */
static void g5p_mmio_gate(struct g5p_diag *g)
{
	unsigned long flags;

	spin_lock_irqsave(&g->mmio_lock, flags);
	g->mmio_off = true;
	spin_unlock_irqrestore(&g->mmio_lock, flags);
}

/* gpu-recovery108: retire predicates (teardown.inc) and dirty-start hooks (dirtystart108.inc). */
static bool g5p_r8_r1_allowed(void);
static bool g5p_r8_is_retired(void);
static void g5p_ds_hook_probe(struct g5p_diag *g);
static void g5p_ds_hook_power(struct g5p_diag *g);
static int g5p_ds_hook_reset(struct g5p_diag *g);

static void g5p_ds_hook_wake(struct g5p_diag *g, int ret);
static void g5p_ds_hook_maps(struct g5p_diag *g, int ret);
static void g5p_ds_hook_load(struct g5p_diag *g, int ret);
static int g5p_ds_hook_release_gate(struct g5p_diag *g);
static void g5p_ds_hook_release(struct g5p_diag *g, int ret);
static void g5p_ds_hook_alive(struct g5p_diag *g, int ret);
/* wake112: the reset generation of a deferred R74E proof (dirtystart108.inc). */
static u32 g5p_ds_gen(void);

#include "diagnostics.inc"

/* The GPU ADT clock/power gate124 resolves to ps-regs[4]+index8*8.
 * Read via its existing syscon, sharing the provider mapping without writes.
 */
static void g5p_power_observer_init(struct g5p_diag *g)
{
	struct device_node *domain, *parent;
	u32 offset;

	domain = of_parse_phandle(g->dev->of_node, "power-domains", 0);
	if (!domain)
		return;
	parent = of_get_parent(domain);
	if (of_device_is_compatible(domain, "apple,s8000-pmgr-pwrstate") &&
	    !of_property_read_u32(domain, "reg", &offset) &&
	    offset == 0x80340 && parent &&
	    of_device_is_compatible(parent, "apple,s8000-pmgr")) {
		g->pmgr = syscon_node_to_regmap(parent);
		if (IS_ERR(g->pmgr))
			g->pmgr = NULL;
	}
	of_node_put(parent);
	of_node_put(domain);
}

/* pmgr-pwrstate.c masks; history flags are sticky, not current clock state. */
static void g5p_power_snapshot(struct g5p_diag *g, const char *stage)
{
	u32 raw;
	int ret;

	if (!g->pmgr) {
		dev_info(
			g->dev,
			"POWER %s PMGR gfx unavailable: provider not resolved\n",
			stage);
		return;
	}
	dev_info(g->dev, "POWER %s reading PMGR gfx20e080340 gate124\n", stage);
	ret = regmap_read(g->pmgr, 0x80340, &raw);
	if (ret) {
		dev_info(g->dev, "POWER %s PMGR gfx read error=%d\n", stage,
			 ret);
		return;
	}
	dev_info(
		g->dev,
		"POWER %s PMGR gfx20e080340=%08x target=%x actual=%x auto=%u auto_state=%x "
		"min=%x reset=%u parent_off=%u disabled=%u; current outer-domain fields only\n",
		stage, raw, raw & 0xf, (raw >> 4) & 0xf, (raw >> 28) & 1,
		(raw >> 24) & 0xf, (raw >> 16) & 0xf, raw >> 31,
		(raw >> 11) & 1, (raw >> 10) & 1);
	dev_info(
		g->dev,
		"POWER %s PMGR history was_clkgated=%u was_pwrgated=%u; sticky history, not "
		"current state\n",
		stage, (raw >> 9) & 1, (raw >> 8) & 1);
}

static int g5p_power(struct g5p_diag *g)
{
	int ret;

	if (g->powered)
		return 0;
	dev_info(g->dev, "request gfx power domain\n");
	ret = pm_runtime_resume_and_get(g->dev);
	if (ret < 0)
		return ret;
	g->powered = true;
	dev_info(g->dev, "gfx runtime PM active\n");
	g5p_ds_hook_power(g);
	return 0;
}

#include "power-r33.inc"
#include "power-r41.inc"

static bool boot_gfx_off_done;

static int g5p_fender_wake_once(struct g5p_diag *g)
{
	void __iomem *fender = g->window[1];
	u32 value;
	int ret;

	ret = g5p_power(g);
	if (ret)
		return ret;
	/* recovery108 mode 3: the one bounded incoming cycle, before any GPU MMIO. */
	ret = g5p_ds_hook_reset(g);
	if (ret)
		return ret;
	/* R33: once per boot, before the first Fender write (no GPU MMIO yet). */
	;
	ret = g5p_r41_bridge_prepare(g);
	if (ret)
		return ret;
	dev_info(g->dev, "W R1+10000=ffff0000 (Apple ...6c93fe8)\n");
	writel(0xffff0000, fender + 0x10000);
	dev_info(g->dev, "W R1+00000=40000000 (Apple ...6c93ff0)\n");
	writel(0x40000000, fender);
	dev_info(g->dev, "W R1+01000=00000011 (Apple ...6c93ff8)\n");
	writel(0x11, fender + 0x1000);
	dev_info(g->dev, "poll R1+01000 bit4 clear\n");
	ret = readl_poll_timeout(fender + 0x1000, value, !(value & BIT(4)), 10,
				 100000);
	dev_info(g->dev, "Fender wake status=%08x result=%d\n", value, ret);
	if (ret)
		return ret;
	usleep_range(10, 20);
	g->fender_ready = true;
	{
		unsigned long flags;

		spin_lock_irqsave(&g->mmio_lock, flags);
		g->mmio_off = false;
		spin_unlock_irqrestore(&g->mmio_lock, flags);
	}
	return 0;
}

static int g5p_fender_wake(struct g5p_diag *g)
{
	int ret;

	if (g->fender_ready)
		return 0;
	ret = g5p_fender_wake_once(g);
	g5p_ds_hook_wake(g, ret);
	return ret;
}

static int g5p_sp_wait(struct g5p_diag *g)
{
	u32 value;
	int ret;

	if (!g5p_slave_access_allowed(READ_ONCE(g->reset_transition)))
		return -EPERM;
	ret = readl_poll_timeout(g->rgx + G5P_MSLVCTRL1, value,
				 (value & G5P_SP_READY) == G5P_SP_READY, 10,
				 100000);
	if (ret)
		dev_err(g->dev, "slave port timeout CTRL1=%08x words=%u\n",
			value, g->words_written);
	if (ret)
		dev_err(g->dev, "slave target=%08x\n", g->last_sp_addr);
	return ret;
}

static int g5p_meta_read(struct g5p_diag *g, u32 addr, u32 *out)
{
	int ret;

	g->last_sp_addr = addr;
	ret = g5p_sp_wait(g);

	if (ret)
		return ret;
	writel(addr | 1, g->rgx + G5P_MSLVCTRL0);
	readl(g->rgx + G5P_MSLVCTRL0);
	ret = g5p_sp_wait(g);
	if (ret)
		return ret;
	*out = readl(g->rgx + G5P_MSLVDATAX);
	return 0;
}

static int g5p_firmware(struct g5p_diag *g, bool prepare)
{
	static const u8 expected[SHA256_DIGEST_SIZE] = {
		0x33, 0xae, 0x20, 0xcb, 0xbe, 0x24, 0x98, 0xe4,
		0xf1, 0x19, 0xba, 0x4e, 0x9a, 0x55, 0x46, 0x7e,
		0x4e, 0x01, 0x92, 0xd4, 0xc8, 0x6d, 0x5b, 0xf1,
		0x90, 0x3a, 0xdc, 0xe6, 0x3f, 0xfa, 0x5d, 0xca,
	};
	/* SHA-256 of the packed image of that LDR, unmodified (nopatch's receipt) */
	static const u8 g5p_fw_packed_sha[SHA256_DIGEST_SIZE] = {
		0x48, 0xea, 0x03, 0x14, 0x08, 0x23, 0x96, 0x23,
		0xd8, 0x6f, 0x79, 0xb8, 0xa5, 0xda, 0x55, 0x7e,
		0x25, 0x3d, 0xb1, 0x6f, 0xb6, 0x23, 0x47, 0x95,
		0xe8, 0xed, 0xba, 0xa5, 0xcd, 0x9b, 0xb9, 0x1a,
	};
	const struct firmware *fw;
	u8 digest[SHA256_DIGEST_SIZE];
	int ret;

	ret = request_firmware_direct(&fw, G5P_FW_NAME, g->dev);
	if (ret)
		return ret;
	sha256(fw->data, fw->size, digest);
	if (fw->size != 119112 || memcmp(digest, expected, sizeof(expected))) {
		ret = -EINVAL;
		goto out;
	}
	ret = g5p_ldr_walk(fw->data, fw->size, NULL, NULL);
	if (ret)
		goto out;
	if (prepare && !g->loaded) {
		g->image.data = kvzalloc(G5P_IMAGE_SIZE, GFP_KERNEL);
		if (!g->image.data) {
			ret = -ENOMEM;
			goto out;
		}
		ret = g5p_ldr_walk(fw->data, fw->size, g5p_pack_record,
				   &g->image);
		if (!ret && g->image.config_count != G5P_CONFIG_WRITES)
			ret = -EINVAL;
		if (!ret) {
			sha256(g->image.data, G5P_IMAGE_SIZE, digest);
			if (memcmp(digest, g5p_fw_packed_sha, sizeof(digest)))
				ret = -EBADMSG;
		}
		if (ret) {
			kvfree(g->image.data);
			memset(&g->image, 0, sizeof(g->image));
			goto out;
		}
		g->loaded = true;
		dev_info(
			g->dev,
			"NOPATCH packed apple_original=1 packed_sha256=%32phN end=nopatch\n",
			digest);
	}
out:
	release_firmware(fw);
	return ret;
}

static int g5p_sample(struct g5p_diag *g)
{
	u32 value;
	unsigned int thread, reg;
	int ret;

	if (!g5p_slave_access_allowed(READ_ONCE(g->reset_transition)))
		return -EPERM;
	if (!g->rgx)
		return -EINVAL;
	dev_info(g->dev, "RGX CTRL1=%08x IRQSTATUS=%08x\n",
		 readl(g->rgx + G5P_MSLVCTRL1),
		 readl(g->rgx + G5P_MSLVIRQSTATUS));
	for (thread = 0; thread < 2; thread++) {
		for (reg = 0; reg <= 0x10; reg += 0x10) {
			ret = g5p_meta_read(
				g, 0x04800000 + thread * 0x1000 + reg, &value);
			if (ret)
				return ret;
			dev_info(g->dev, "META T%u %s=%08x\n", thread,
				 reg ? "STATUS" : "ENABLE", value);
		}
	}
	return 0;
}

static int g5p_read_id(struct g5p_diag *g)
{
	u32 words[2], version;
	static const u32 offsets[] = { 0x20, 0x18 };
	unsigned int i;

	if (!g->fender_ready || READ_ONCE(g->reset_transition))
		return -EPERM;
	g->id_checked = false;
	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		dev_info(g->dev, "R R0+%05x\n", offsets[i]);
		words[i] = readl(g->rgx + offsets[i]);
		dev_info(g->dev, "R0+%05x=%08x\n", offsets[i], words[i]);
	}
	g->core_revision = words[0];
	g->core_id = words[1];
	if ((!words[0] && !words[1]) || words[0] == ~0U || words[1] == ~0U)
		return -ENODEV;
	/* Match Apple readChipInfo at ...6ca644c before deeper META accesses. */
	version = (words[0] >> 8) & 0xff;
	if ((words[0] & 0xff0000) != 0x90000 ||
	    (version != 7 && version != 10 && version != 13 && version != 15))
		return -ENODEV;
	g->id_checked = true;
	return 0;
}

#include "boot.inc"
#include "mmio.inc"
#include "kick.inc"
#include "lifecycle.inc"
#include "teardown.inc"
#include "dirtystart108.inc"
#include "recover.inc"
#include "transport-budget.inc"
#include "render-layout-r22.h"
#include "render-variants-r25.h"

static ssize_t result_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct g5p_diag *g = dev_get_drvdata(dev);
	struct g5p_boot *b = READ_ONCE(g->boot);
	return sysfs_emit(
		buf,
		"last=%d loaded=%u maps=%u armed=%u copied=%u released=%u alive=%u "
		"control_complete=%u failed=%u\n",
		READ_ONCE(g->last_result), READ_ONCE(g->loaded),
		b && b->maps_ready, b && b->armed, b && b->copied,
		b && b->released, b && b->alive, b && b->control_complete,
		b && b->failed);
}
static DEVICE_ATTR_RO(result);

/* Cached raw words from the last explicit/automatic snapshot: no GPU MMIO. */

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct g5p_diag *g = dev_get_drvdata(dev);
	int ret;
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (sysfs_streq(buf, "td-kexec"))
		return g5p_td_command(g, buf) ?: count;
	if (g5p_r8_is_retired())
		return -ESHUTDOWN;
	guard(mutex)
		(&g->lock);
	if (g->boot && (g->boot->stopped || g->boot->user_compute_owned))
		return -EBUSY;
	if (sysfs_streq(buf, "validate") || sysfs_streq(buf, "prepare"))
		ret = g5p_firmware(g, sysfs_streq(buf, "prepare"));
	else if (sysfs_streq(buf, "wake"))
		ret = g5p_fender_wake(g);
	else if (sysfs_streq(buf, "id"))
		ret = g5p_read_id(g);
	else if (sysfs_streq(buf, "sample"))
		ret = g->id_checked ? g5p_sample(g) : -EPERM;
	else if (sysfs_streq(buf, "control-prepare"))
		ret = g5p_control_prepare(g);
	else if (sysfs_streq(buf, "control-submit"))
		ret = g5p_control_submit(g);
	else if (sysfs_streq(buf, "control-wait"))
		ret = g5p_control_wait(g);
	else if (sysfs_streq(buf, "map") || sysfs_streq(buf, "arm") ||
		 sysfs_streq(buf, "load") || sysfs_streq(buf, "release") ||
		 sysfs_streq(buf, "events"))
		ret = g5p_boot_command(g, buf);
	else
		return -EOPNOTSUPP;
	g->last_result = ret;
	dev_info(dev, "command %.*s result=%d\n", (int)count, buf, ret);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(command);

static struct attribute *g5p_attrs[] = {
	&dev_attr_command.attr,
	&dev_attr_result.attr,
	&dev_attr_recovery_status.attr,
	NULL,
};
static const struct attribute_group g5p_group = { .attrs = g5p_attrs };
__ATTRIBUTE_GROUPS(g5p);

static int g5p_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct g5p_diag *g;
	unsigned int i;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = dev;
	if (g5p_diagnostics_init(g))
		return -ENOMEM;
	mutex_init(&g->lock);
	spin_lock_init(&g->slot_lock);
	spin_lock_init(&g->mmio_lock);
	g->mmio_max_pstate = 1;
	bitmap_set(g->user_slots, 0, 7);
	for (i = 0; i < 2; i++) {
		g->window[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(g->window[i]))
			return PTR_ERR(g->window[i]);
	}
	g->rgx = g->window[0];
	g5p_power_observer_init(g);
	platform_set_drvdata(pdev, g);
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40)))
		return -ENODEV;
	pm_runtime_enable(dev);
	g5p_td_attach(g);
	g5p_ds_hook_probe(g);
	;
	dev_info(dev, "mapped both ADT windows; awaiting explicit command\n");
	return 0;
}

static void g5p_remove(struct platform_device *pdev)
{
	struct g5p_diag *g = platform_get_drvdata(pdev);
	/* With teardown on, keep the PM reference: an outgoing PMGR off is unvalidated. */
	bool keep_power;

	;
	keep_power = g5p_td_remove(g);
	if (g->powered && !keep_power) {
		g5p_mmio_gate(g);
		pm_runtime_put_sync(g->dev);
	}
	pm_runtime_disable(g->dev);
	kvfree(g->image.data);
}

/* Registration itself must not claim a device owned by another frontend. */
static int g5p_frontend_available(void)
{
	struct device *dev;
	int ret;

	if (!of_machine_is_compatible("apple,j71t"))
		return -ENODEV;
	dev = bus_find_device_by_name(&platform_bus_type, NULL,
				      "201000000.gpu");
	if (!dev)
		return -ENODEV;
	device_lock(dev);
	ret = dev->driver ? -EBUSY : 0;
	device_unlock(dev);
	put_device(dev);
	return ret;
}

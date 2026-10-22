// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * A9/G5P firmware diagnostic, not a DRM or rendering driver.
 * Default enable=0 prevents even platform-driver registration / genpd attach.
 * RE evidence: linux-a9/docs/gpu/agx-g5p-bringup.md (20H350 j71t).
 */
#include <crypto/sha2.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include "ldr.h"

#define G5P_MSLVDATAX 0x0a00
#define G5P_MSLVDATAT 0x0a08
#define G5P_MSLVCTRL0 0x0a10
#define G5P_MSLVCTRL1 0x0a18
#define G5P_MSLVIRQSTATUS 0x0ac8
#define G5P_SP_READY (BIT(24) | BIT(26))
#define G5P_FW_NAME "apple/agx-g5p-c0.ldr"

static bool enable;
module_param(enable, bool, 0400);
MODULE_PARM_DESC(enable, "Register experimental GPU diagnostic (default false)");

/* Still unresolved offline; never silently choose an address space. */
static unsigned int rgx_window = 2;
module_param(rgx_window, uint, 0400);
MODULE_PARM_DESC(rgx_window, "RGX DT register window (0 or 1, default unset)");

struct g5p_diag {
	struct device *dev;
	void __iomem *window[2];
	void __iomem *rgx;
	/* Serializes commands and all hardware access. */
	struct mutex lock;
	bool powered;
	bool loaded;
	int last_result;
	u32 words_written;
};

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
	return 0;
}

static int g5p_sp_wait(struct g5p_diag *g)
{
	u32 value;
	int ret;

	ret = readl_poll_timeout(g->rgx + G5P_MSLVCTRL1, value,
				 (value & G5P_SP_READY) == G5P_SP_READY, 10, 100000);
	if (ret)
		dev_err(g->dev, "slave port timeout CTRL1=%08x words=%u\n",
			value, g->words_written);
	return ret;
}

static int g5p_meta_read(struct g5p_diag *g, u32 addr, u32 *out)
{
	int ret = g5p_sp_wait(g);

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

static int g5p_meta_write(struct g5p_diag *g, u32 addr, u32 value)
{
	int ret = g5p_sp_wait(g);

	if (ret)
		return ret;
	writel(addr, g->rgx + G5P_MSLVCTRL0);
	readl(g->rgx + G5P_MSLVCTRL0);
	writel(value, g->rgx + G5P_MSLVDATAT);
	g->words_written++;
	return g5p_sp_wait(g);
}

static int g5p_emit_ldr(void *ctx, enum g5p_ldr_op op, u32 addr, u32 size,
			const u8 *data)
{
	struct g5p_diag *g = ctx;
	u32 i;
	int ret;

	/* Starting threads is a separate operation after init publication. */
	if (op == G5P_START)
		return 0;
	dev_info(g->dev, "LDR op=%u addr=%08x size=%u\n", op, addr, size);
	for (i = 0; i < size; i += op == G5P_CONFIG ? 12 : 4) {
		u32 dest = op == G5P_CONFIG ? g5p_le32(data + i + 4) : addr + i;
		u32 value = op == G5P_CONFIG ? g5p_le32(data + i + 8) :
			op == G5P_ZERO ? 0 : g5p_le32(data + i);

		ret = g5p_meta_write(g, dest, value);
		if (ret)
			return ret;
		if (!(i & 0xfff))
			cond_resched();
	}
	return 0;
}

static int g5p_firmware(struct g5p_diag *g, bool upload)
{
	static const u8 expected[SHA256_DIGEST_SIZE] = {
		0x33, 0xae, 0x20, 0xcb, 0xbe, 0x24, 0x98, 0xe4,
		0xf1, 0x19, 0xba, 0x4e, 0x9a, 0x55, 0x46, 0x7e,
		0x4e, 0x01, 0x92, 0xd4, 0xc8, 0x6d, 0x5b, 0xf1,
		0x90, 0x3a, 0xdc, 0xe6, 0x3f, 0xfa, 0x5d, 0xca,
	};
	const struct firmware *fw;
	u8 digest[SHA256_DIGEST_SIZE];
	int ret;

	ret = request_firmware_direct(&fw, G5P_FW_NAME, g->dev);
	if (ret)
		return ret;
	sha256(fw->data, fw->size, digest);
	if (fw->size != 119112 || memcmp(digest, expected, sizeof(expected))) {
		dev_err(g->dev, "wrong G5P C0 firmware bytes\n");
		ret = -EINVAL;
		goto out;
	}
	ret = g5p_ldr_walk(fw->data, fw->size, NULL, NULL);
	if (ret)
		goto out;
	dev_info(g->dev, "firmware SHA256 and all 94 LDR records validated\n");
	if (upload) {
		ret = g5p_ldr_walk(fw->data, fw->size, g5p_emit_ldr, g);
		g->loaded = !ret;
	}
out:
	release_firmware(fw);
	return ret;
}

static int __maybe_unused g5p_sample(struct g5p_diag *g)
{
	u32 value;
	unsigned int thread, reg;
	int ret;

	if (!g->rgx)
		return -EINVAL;
	dev_info(g->dev, "RGX CTRL1=%08x IRQSTATUS=%08x\n",
		 readl(g->rgx + G5P_MSLVCTRL1),
		 readl(g->rgx + G5P_MSLVIRQSTATUS));
	for (thread = 0; thread < 2; thread++) {
		for (reg = 0; reg <= 0x10; reg += 0x10) {
			ret = g5p_meta_read(g, 0x04800000 + thread * 0x1000 + reg, &value);
			if (ret)
				return ret;
			dev_info(g->dev, "META T%u %s=%08x\n", thread,
				 reg ? "STATUS" : "ENABLE", value);
		}
	}
	return 0;
}

static ssize_t result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct g5p_diag *g = dev_get_drvdata(dev);

	return sysfs_emit(buf, "last=%d powered=%u loaded=%u words=%u\n",
		READ_ONCE(g->last_result), READ_ONCE(g->powered),
		READ_ONCE(g->loaded), READ_ONCE(g->words_written));
}
static DEVICE_ATTR_RO(result);

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct g5p_diag *g = dev_get_drvdata(dev);
	int ret = 0;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	guard(mutex)(&g->lock);
	if (sysfs_streq(buf, "validate")) {
		ret = g5p_firmware(g, false);
		goto out;
	}
	if (!sysfs_streq(buf, "id") && !sysfs_streq(buf, "sample") &&
	    !sysfs_streq(buf, "load")) {
		ret = -EINVAL;
		goto out;
	}
	ret = g5p_power(g);
	if (ret)
		goto out;
	if (sysfs_streq(buf, "id")) {
		dev_err(dev, "ID reads withheld: experiment 01 wedged R0 before Fender setup\n");
		ret = -EOPNOTSUPP;
	} else if (sysfs_streq(buf, "sample")) {
		dev_err(dev, "sample withheld: Fender setup not yet established\n");
		ret = -EOPNOTSUPP;
	} else {
		/* External backing / reset sequence must be proven first. */
		dev_err(dev, "load withheld: META backing/MMU setup not established\n");
		ret = -EOPNOTSUPP;
	}
out:
	g->last_result = ret;
	dev_info(dev, "command %.*s result=%d\n", (int)count, buf, ret);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(command);

static struct attribute *g5p_attrs[] = {
	&dev_attr_command.attr, &dev_attr_result.attr, NULL,
};
ATTRIBUTE_GROUPS(g5p);

static int g5p_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct g5p_diag *g;
	unsigned int i;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = dev;
	mutex_init(&g->lock);
	for (i = 0; i < 2; i++) {
		g->window[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(g->window[i]))
			return PTR_ERR(g->window[i]);
	}
	if (rgx_window < 2)
		g->rgx = g->window[rgx_window];
	platform_set_drvdata(pdev, g);
	pm_runtime_enable(dev);
	dev_info(dev, "mapped both ADT windows; awaiting explicit command\n");
	return 0;
}

static void g5p_remove(struct platform_device *pdev)
{
	struct g5p_diag *g = platform_get_drvdata(pdev);

	if (g->powered)
		pm_runtime_put_sync(g->dev);
	pm_runtime_disable(g->dev);
}

static const struct of_device_id g5p_match[] = {
	{ .compatible = "gpu,s8003" }, { },
};
MODULE_DEVICE_TABLE(of, g5p_match);

static struct platform_driver g5p_driver = {
	.probe = g5p_probe,
	.remove = g5p_remove,
	.driver = {
		.name = "a9-gpu-diag",
		.of_match_table = g5p_match,
		.dev_groups = g5p_groups,
		.suppress_bind_attrs = true,
	},
};

static int __init g5p_init(void)
{
	if (!enable) {
		pr_info("a9-gpu-diag: inert (enable=0)\n");
		return 0;
	}
	if (!of_machine_is_compatible("apple,j71t"))
		return -ENODEV;
	return platform_driver_register(&g5p_driver);
}
module_init(g5p_init);

static void __exit g5p_exit(void)
{
	if (enable)
		platform_driver_unregister(&g5p_driver);
}
module_exit(g5p_exit);

MODULE_DESCRIPTION("Inert-by-default Apple A9 G5P META diagnostic");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_FIRMWARE(G5P_FW_NAME);

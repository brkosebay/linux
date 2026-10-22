// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * A9/G5P firmware diagnostic, not a DRM or rendering driver.
 * Default enable=0 prevents even platform-driver registration / genpd attach.
 * RE evidence: linux-a9/docs/gpu/agx-g5p-bringup.md (20H350 j71t).
 */
#include <crypto/sha2.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include "pack.h"

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

struct g5p_diag {
	struct device *dev;
	void __iomem *window[2];
	void __iomem *rgx;
	/* Serializes commands and all hardware access. */
	struct mutex lock;
	bool powered;
	bool loaded;
	struct g5p_ldr_image image;
	dma_addr_t image_dma;
	bool fender_ready;
	bool id_checked;
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

static int g5p_fender_wake(struct g5p_diag *g)
{
	void __iomem *fender = g->window[1];
	u32 value;
	int ret;

	if (g->fender_ready)
		return 0;
	ret = g5p_power(g);
	if (ret)
		return ret;
	dev_info(g->dev, "W R1+10000=ffff0000 (Apple ...6c93fe8)\n");
	writel(0xffff0000, fender + 0x10000);
	dev_info(g->dev, "W R1+00000=40000000 (Apple ...6c93ff0)\n");
	writel(0x40000000, fender);
	dev_info(g->dev, "W R1+01000=00000011 (Apple ...6c93ff8)\n");
	writel(0x11, fender + 0x1000);
	dev_info(g->dev, "poll R1+01000 bit4 clear\n");
	ret = readl_poll_timeout(fender + 0x1000, value, !(value & BIT(4)),
				 10, 100000);
	dev_info(g->dev, "Fender wake status=%08x result=%d\n", value, ret);
	if (ret)
		return ret;
	usleep_range(10, 20);
	g->fender_ready = true;
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

static int __maybe_unused g5p_meta_write(struct g5p_diag *g, u32 addr, u32 value)
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

static int g5p_firmware(struct g5p_diag *g, bool prepare)
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
	if (prepare && !g->loaded) {
		g->image.data = dmam_alloc_coherent(g->dev, G5P_IMAGE_SIZE,
						    &g->image_dma, GFP_KERNEL);
		if (!g->image.data) {
			ret = -ENOMEM;
			goto out;
		}
		memset(g->image.data, 0, G5P_IMAGE_SIZE);
		ret = g5p_ldr_walk(fw->data, fw->size, g5p_pack_record, &g->image);
		if (!ret && g->image.config_count != G5P_CONFIG_WRITES)
			ret = -EINVAL;
		if (ret) {
			dmam_free_coherent(g->dev, G5P_IMAGE_SIZE,
					   g->image.data, g->image_dma);
			memset(&g->image, 0, sizeof(g->image));
			goto out;
		}
		g->loaded = true;
		dev_info(g->dev, "LDR packed into %x DMA bytes at %pad; T1 PC=%08x SP=%08x\n",
			 G5P_IMAGE_SIZE, &g->image_dma, g->image.t1_pc, g->image.t1_sp);
		dev_info(g->dev, "not executable: GPU PTEs, boot config and shared init not installed\n");
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

static int g5p_read_id(struct g5p_diag *g)
{
	u32 words[2], version;
	static const u32 offsets[] = { 0x20, 0x18 };
	unsigned int i;

	if (!g->fender_ready)
		return -EPERM;
	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		dev_info(g->dev, "R R0+%05x\n", offsets[i]);
		words[i] = readl(g->rgx + offsets[i]);
		dev_info(g->dev, "R0+%05x=%08x\n", offsets[i], words[i]);
	}
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

static ssize_t result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct g5p_diag *g = dev_get_drvdata(dev);

	return sysfs_emit(buf, "last=%d powered=%u fender=%u id=%u loaded=%u words=%u\n",
		READ_ONCE(g->last_result), READ_ONCE(g->powered),
		READ_ONCE(g->fender_ready), READ_ONCE(g->id_checked),
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
	if (sysfs_streq(buf, "validate") || sysfs_streq(buf, "prepare")) {
		ret = g5p_firmware(g, sysfs_streq(buf, "prepare"));
		goto out;
	}
	if (!sysfs_streq(buf, "id") && !sysfs_streq(buf, "sample") &&
	    !sysfs_streq(buf, "load") && !sysfs_streq(buf, "wake")) {
		ret = -EINVAL;
		goto out;
	}
	if (sysfs_streq(buf, "wake")) {
		ret = g5p_fender_wake(g);
	} else if (sysfs_streq(buf, "id")) {
		ret = g5p_read_id(g);
	} else if (sysfs_streq(buf, "sample")) {
		ret = g->id_checked ? g5p_sample(g) : -EPERM;
	} else {
		/* External backing / reset sequence must be proven first. */
		dev_err(dev, "load withheld: GPU PTEs and shared init not installed\n");
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
	g->rgx = g->window[0];
	platform_set_drvdata(pdev, g);
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40)))
		return -ENODEV;
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

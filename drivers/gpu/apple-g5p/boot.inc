/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Experimental G2 stages. Included after the bounded META slave primitives. */
#include <linux/dma-direct.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/vmalloc.h>
#include "boot-layout.h"

static bool boot_experiment;
module_param(boot_experiment, bool, 0400);
MODULE_PARM_DESC(boot_experiment, "Permit explicit experimental G2 boot stages (default false)");

struct g5p_dma_page {
	u8 *cpu;
	dma_addr_t dma;
};

struct g5p_boot {
	struct g5p_boot_layout layout;
	struct g5p_dma_page image[G5P_IMAGE_SIZE / G5P_GPU_PAGE];
	struct g5p_dma_page root[G5P_ROOT_SIZE / G5P_GPU_PAGE];
	struct g5p_dma_page shared[G5P_SHARED_SIZE / G5P_GPU_PAGE];
	struct g5p_dma_page catalogue, directory, zero, leaf[4];
	u32 leaf_index[4];
	u32 leaves, pages, mapped, fifo, events, alive_word;
	int irq;
	atomic_t irqs;
	spinlock_t event_lock;
	void __iomem *swirq;
	bool maps_ready, armed, copied, released, alive, failed, irq_ready;
};

static int g5p_boot_alloc_page(struct g5p_diag *g, struct g5p_dma_page *p)
{
	p->cpu = dmam_alloc_coherent(g->dev, G5P_GPU_PAGE, &p->dma, GFP_KERNEL);
	if (!p->cpu)
		return -ENOMEM;
	/* There is no GPU bus translation in the recovered GART path. */
	if ((p->dma & (G5P_GPU_PAGE - 1)) || p->dma >= BIT_ULL(40) ||
	    dma_to_phys(g->dev, p->dma) != p->dma) {
		dev_err(g->dev, "unsupported DMA translation/alignment at %pad\n", &p->dma);
		return -ERANGE;
	}
	memset(p->cpu, 0, G5P_GPU_PAGE);
	g->boot->pages++;
	return 0;
}

static int g5p_boot_map_page(struct g5p_diag *g, u64 va, u64 pa, u32 attributes)
{
	struct g5p_boot *b = g->boot;
	u32 ci = (va >> 30) & 1023, di = (va >> 21) & 511;
	u32 ti = (va >> 14) & 127, i;
	int ret;

	if ((va | pa) & (G5P_GPU_PAGE - 1))
		return -EINVAL;
	/* This experiment only owns the recovered firmware heap's catalogue slot. */
	if (ci != ((G5P_IMAGE_VA >> 30) & 1023))
		return -ERANGE;
	for (i = 0; i < b->leaves; i++)
		if (b->leaf_index[i] == di)
			break;
	if (i == b->leaves) {
		if (i == ARRAY_SIZE(b->leaf))
			return -ENOSPC;
		ret = g5p_boot_alloc_page(g, &b->leaf[i]);
		if (ret)
			return ret;
		b->leaf_index[i] = di;
		g5p_put64(b->directory.cpu + di * 8, g5p_directory_entry(b->leaf[i].dma));
		b->leaves++;
	}
	if (g5p_le32(b->leaf[i].cpu + ti * 8))
		return -EEXIST;
	g5p_put64(b->leaf[i].cpu + ti * 8, g5p_leaf_entry(pa, attributes));
	g5p_put32(b->catalogue.cpu + ci * 4, g5p_catalogue_entry(b->directory.dma));
	b->mapped++;
	return 0;
}

static int g5p_boot_region(struct g5p_diag *g, struct g5p_dma_page *p, u32 size, u64 va)
{
	u32 offset;
	u32 attributes = va == G5P_IMAGE_VA ? 0x10 : 0x14;
	int ret;

	for (offset = 0; offset < size; offset += G5P_GPU_PAGE, p++) {
		ret = g5p_boot_alloc_page(g, p);
		if (ret)
			return ret;
		ret = g5p_boot_map_page(g, va + offset, p->dma, attributes);
		if (ret)
			return ret;
	}
	return 0;
}

static void g5p_boot_copy(struct g5p_dma_page *p, const u8 *data, u32 size)
{
	u32 offset;

	for (offset = 0; offset < size; offset += G5P_GPU_PAGE, p++)
		memcpy(p->cpu, data + offset, G5P_GPU_PAGE);
}

static u8 *g5p_boot_shared(struct g5p_boot *b, u32 slot)
{
	u32 off = b->layout.offset[slot];

	return b->shared[off / G5P_GPU_PAGE].cpu + off % G5P_GPU_PAGE;
}

static int g5p_boot_maps(struct g5p_diag *g)
{
	struct g5p_boot *b;
	u8 *root = NULL, *shared = NULL;
	u32 i;
	int ret;

	if (!g->loaded || !g->id_checked || PAGE_SIZE != G5P_GPU_PAGE)
		return -EPERM;
	if (g->core_id != 0x011b0602 || g->core_revision != 0x00090f00)
		return -ENODEV;
	if (g->boot)
		return g->boot->maps_ready ? 0 : -EIO;
	b = devm_kzalloc(g->dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	g->boot = b;
	spin_lock_init(&b->event_lock);
	atomic_set(&b->irqs, 0);
	ret = g5p_plan_layout(&b->layout);
	if (ret)
		goto out;
	root = kvzalloc(G5P_ROOT_SIZE, GFP_KERNEL);
	shared = kvzalloc(G5P_SHARED_SIZE, GFP_KERNEL);
	if (!root || !shared) {
		ret = -ENOMEM;
		goto out;
	}
	ret = g5p_boot_alloc_page(g, &b->catalogue);
	if (ret)
		goto out;
	ret = g5p_boot_alloc_page(g, &b->directory);
	if (ret)
		goto out;
	ret = g5p_boot_alloc_page(g, &b->zero);
	if (ret)
		goto out;
	ret = g5p_boot_region(g, b->image, G5P_IMAGE_SIZE, G5P_IMAGE_VA);
	if (ret)
		goto out;
	ret = g5p_boot_region(g, b->root, G5P_ROOT_SIZE, G5P_ROOT_VA);
	if (ret)
		goto out;
	ret = g5p_boot_region(g, b->shared, G5P_SHARED_SIZE, G5P_SHARED_VA);
	if (ret)
		goto out;
	/* c80758: Fender (2 pages), AIC timer, AIC software interrupt. */
	for (i = 0; i < 4; i++) {
		u64 pa = i < 2 ? 0x201e00000ULL + i * G5P_GPU_PAGE :
			 i == 2 ? 0x20e100000ULL : 0x20e104000ULL;

		ret = g5p_boot_map_page(g, G5P_PIO_VA + i * G5P_GPU_PAGE, pa, 0x18);
		if (ret)
			goto out;
	}
	g5p_make_boot_config(&g->image, &b->layout);
	g5p_make_init(&b->layout, root, shared, &g->image, b->catalogue.dma,
		       arch_timer_read_counter());
	g5p_boot_copy(b->image, g->image.data, G5P_IMAGE_SIZE);
	g5p_boot_copy(b->root, root, G5P_ROOT_SIZE);
	g5p_boot_copy(b->shared, shared, G5P_SHARED_SIZE);
	dma_wmb();
	b->maps_ready = true;
	dev_info(g->dev, "G2 maps: %u x 16KiB allocations; %u PTEs, %u leaves, PC=%pad\n",
		 b->pages, b->mapped, b->leaves, &b->catalogue.dma);
	dev_info(g->dev, "G2 init: root=%llx control=%llx event-ctl=%llx event-data=%llx\n",
		 g5p_object_va(&b->layout, 0x50), g5p_object_va(&b->layout, 0x5f),
		 g5p_object_va(&b->layout, 0x78), g5p_object_va(&b->layout, 0x7c));
	dev_info(g->dev, "G2 experimental zero policy fields; no GPU register access in map\n");
out:
	kvfree(root);
	kvfree(shared);
	if (ret)
		b->failed = true;
	return ret;
}

static int g5p_boot_drain(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 *control = (u32 *)g5p_boot_shared(b, 0x70);
	u32 consumer, producer, n;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&b->event_lock, flags);
	consumer = READ_ONCE(control[0]);
	producer = READ_ONCE(control[4]);
	if (consumer > 255 || producer > 255) {
		dev_err(g->dev, "G2 invalid event indices %u/%u\n", consumer, producer);
		ret = -EPROTO;
		goto out;
	}
	dma_rmb();
	for (n = 0; n < 16 && consumer != producer; n++) {
		u32 words[11], off = b->layout.offset[0x74] + consumer * 44;
		u8 *src;
		unsigned int i;

		if (b->events >= 256) {
			ret = -EOVERFLOW;
			goto out;
		}
		/* Entries may straddle DMA pages. */
		for (i = 0; i < ARRAY_SIZE(words); i++) {
			u32 at = off + i * 4;

			src = b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE;
			words[i] = g5p_le32(src);
		}
		dev_info(g->dev, "G2 event index=%u type=%u word1=%08x\n",
			 consumer, words[0], words[1]);
		print_hex_dump(KERN_INFO, "G2 event raw: ", DUMP_PREFIX_NONE, 16, 4,
			       words, sizeof(words), false);
		if (words[0] == 0 && READ_ONCE(b->released)) {
			b->alive_word = words[1];
			WRITE_ONCE(b->alive, true);
		}
		b->events++;
		consumer = (consumer + 1) & 255;
		dma_wmb();
		WRITE_ONCE(control[0], consumer);
	}
out:
	spin_unlock_irqrestore(&b->event_lock, flags);
	return ret;
}

static irqreturn_t g5p_boot_irq(int irq, void *data)
{
	struct g5p_diag *g = data;
	struct g5p_boot *b = g->boot;

	atomic_inc(&b->irqs);
	if (atomic_read(&b->irqs) > 256) {
		WRITE_ONCE(b->failed, true);
		disable_irq_nosync(irq);
		dev_err(g->dev, "G2 IRQ174 storm capped; reboot required\n");
		return IRQ_HANDLED;
	}
	if (READ_ONCE(b->released))
		g5p_boot_drain(g);
	/* ca28d8 / ADT meta-sw-interrupt; Linux AIC owns its normal EOI. */
	writel(0x4000, b->swirq + 0x94);
	return IRQ_HANDLED;
}

static int g5p_boot_irq_setup(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	int ret;

	if (b->irq_ready)
		return 0;
	b->swirq = devm_ioremap(g->dev, 0x20e104000ULL, 0x100);
	if (!b->swirq)
		return -ENOMEM;
	b->irq = platform_get_irq(to_platform_device(g->dev), 4);
	if (b->irq < 0)
		return b->irq;
	ret = devm_request_threaded_irq(g->dev, b->irq, NULL, g5p_boot_irq,
				       IRQF_ONESHOT, "g5p-meta", g);
	if (ret)
		return ret;
	b->irq_ready = true;
	dev_info(g->dev, "G2 IRQ174 registered as Linux IRQ %d\n", b->irq);
	return 0;
}

static void g5p_fifo32(struct g5p_diag *g, u32 reg, u32 value, u32 flags)
{
	struct g5p_boot *b = g->boot;
	void __iomem *f = g->window[1];

	dev_info(g->dev, "G2 FIFO R0+%05x=%08x flags=%02x\n", reg, value, flags);
	writel((reg & 0xff00ffff) | flags << 16, f + b->fifo);
	readl(f + b->fifo);
	b->fifo += 4;
	writel(value, f + b->fifo);
	readl(f + b->fifo);
	b->fifo += 4;
}

static void g5p_fifo64(struct g5p_diag *g, u32 reg, u64 value)
{
	g5p_fifo32(g, reg + 4, value >> 32, 0);
	g5p_fifo32(g, reg, value, 0xff);
}

static int g5p_fifo_run(struct g5p_diag *g)
{
	void __iomem *f = g->window[1];
	u32 status;
	int ret;

	writel(0xffff0000, f + g->boot->fifo);
	readl(f + g->boot->fifo);
	dma_wmb();
	dev_info(g->dev, "G2 W R1+20000=0; W R1+1000=11\n");
	writel(0, f + 0x20000);
	readl(f + 0x20000);
	ret = readl_poll_timeout(f + 0x1000, status, !(status & BIT(4)), 10, 100000);
	if (ret)
		return ret;
	writel(0x11, f + 0x1000);
	readl(f + 0x1000);
	ret = readl_poll_timeout(f + 0x1000, status, !(status & BIT(4)), 10, 100000);
	dev_info(g->dev, "G2 FIFO complete status=%08x ret=%d\n", status, ret);
	usleep_range(10, 20);
	return ret;
}

static int g5p_boot_arm(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	void __iomem *f = g->window[1];
	int ret;

	if (b->armed)
		return -EALREADY;
	ret = g5p_boot_irq_setup(g);
	if (ret)
		return ret;
	dev_info(g->dev, "G2 arm: META held, master boot selected; init FIFO reset\n");
	writel(0, f + 0x10);
	readl(f + 0x10);
	writel(0x40000000, f);
	readl(f);
	b->fifo = 0x10000;
	g5p_fifo64(g, 0x100, 0x00e7fffffffffc1dULL);
	g5p_fifo64(g, 0x100, 0x00e7fe071ffffc1dULL);
	g5p_fifo64(g, 0x100, 0x100000000ULL);
	g5p_fifo32(g, 0xbf8, 1, 0xff);
	g5p_fifo32(g, 0xe140, 0, 0xff);
	g5p_fifo64(g, 0xe148, b->catalogue.dma >> 12);
	g5p_fifo64(g, 0xe200, 3);
	g5p_fifo64(g, 0xe208, 0xb42dd24b4bd22db4ULL);
	g5p_fifo64(g, 0xe210, 0xb42dd24b4bd22db4ULL);
	dev_info(g->dev, "G2 W R1+2010=bf; R1+2014=233\n");
	writel(0xbf, f + 0x2010);
	writel(0x233, f + 0x2014);
	g5p_fifo64(g, 0xe160, b->zero.dma);
	g5p_fifo64(g, 0xbb8, 0);
	ret = g5p_fifo_run(g);
	if (ret)
		return ret;
	dev_info(g->dev, "G2 R R0+100/e148 (reset-held/catalogue readback)\n");
	if (readq(g->rgx + 0x100) != 0x100000000ULL ||
	    readq(g->rgx + 0xe148) != b->catalogue.dma >> 12)
		return -EIO;
	b->armed = true;
	return 0;
}

static int g5p_boot_load(struct g5p_diag *g)
{
	static const struct {
		u32 addr, offset, size;
	} segments[] = {
		{ 0xc0000000, 0, 0x4000 }, { 0x10000000, 0x4000, 0x18000 },
		{ 0x20000000, 0x1c000, 0x4000 }, { 0xe0001000, 0x20000, 0x4000 },
	};
	unsigned long deadline = jiffies + 10 * HZ;
	static const u32 probes[] = { 0, 0x200, 0x204, 0x3ff8, 0x3ffc };
	u32 i, off, value, expected;
	int ret;

	/* First install only the 13 segment pairs; the LDR config follows the copy. */
	for (i = 0; i < 104; i += 8) {
		ret = g5p_meta_write(g, g5p_le32(g->image.data + 0x200 + i),
				      g5p_le32(g->image.data + 0x204 + i));
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(probes); i++) {
		dev_info(g->dev, "G2 SP mapping probe %08x (before RAM writes)\n",
			 0xc0000000 + probes[i]);
		ret = g5p_meta_read(g, 0xc0000000 + probes[i], &value);
		if (ret)
			return ret;
		expected = g5p_le32(g->image.data + probes[i]);
		dev_info(g->dev, "G2 SP probe actual=%08x expected=%08x\n", value, expected);
		if (value != expected)
			return -EIO;
	}
	for (i = 0; i < ARRAY_SIZE(segments); i++) {
		dev_info(g->dev, "G2 SP copy %08x bytes=%x\n", segments[i].addr, segments[i].size);
		for (off = 0; off < segments[i].size; off += 4) {
			if (time_after(jiffies, deadline))
				return -ETIMEDOUT;
			value = g5p_le32(g->image.data + segments[i].offset + off);
			ret = g5p_meta_write(g, segments[i].addr + off, value);
			if (ret)
				return ret;
		}
	}
	for (i = 0; i < g->image.config_count; i++) {
		ret = g5p_meta_write(g, g->image.config[i][0], g->image.config[i][1]);
		if (ret)
			return ret;
	}
	dma_wmb();
	g->boot->copied = true;
	dev_info(g->dev, "G2 SP load complete words=%u; core payload remains GPU RAM for bootloader\n",
		 g->words_written);
	return 0;
}

static int g5p_boot_release(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	int ret;

	if (b->released)
		return -EALREADY;
	dev_info(g->dev, "G2 release: META master boot; no GPU job submitted\n");
	b->fifo = 0x10000;
	g5p_fifo64(g, 0x100, 0);
	WRITE_ONCE(b->released, true);
	ret = g5p_fifo_run(g);
	if (ret)
		return ret;
	return 0;
}

static int g5p_boot_events(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	unsigned long deadline = jiffies + 3 * HZ;
	int ret;

	do {
		if (READ_ONCE(b->failed))
			return -EIO;
		ret = g5p_boot_drain(g);
		if (ret)
			return ret;
		if (READ_ONCE(b->alive)) {
			dev_info(g->dev, "G2 ALIVE type=0 word1=%08x events=%u IRQ174=%d\n",
				 b->alive_word, b->events, atomic_read(&b->irqs));
			return 0;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	dev_err(g->dev, "G2 no alive event: events=%u IRQ174=%d; preserve baseline\n",
		 b->events, atomic_read(&b->irqs));
	return -ETIMEDOUT;
}

static int g5p_boot_command(struct g5p_diag *g, const char *command)
{
	struct g5p_boot *b = g->boot;
	int ret;

	if (sysfs_streq(command, "map"))
		return g5p_boot_maps(g);
	if (!boot_experiment || !b || !b->maps_ready || b->failed)
		return -EPERM;
	if (sysfs_streq(command, "arm")) {
		ret = g5p_boot_arm(g);
	} else if (sysfs_streq(command, "load") && b->armed && !b->released && !b->copied) {
		ret = g5p_boot_load(g);
	} else if (sysfs_streq(command, "release") && b->copied) {
		ret = g5p_boot_release(g);
	} else if (sysfs_streq(command, "events") && b->released) {
		return g5p_boot_events(g);
	} else {
		return -EPERM;
	}
	if (ret && ret != -EALREADY)
		b->failed = true;
	return ret;
}

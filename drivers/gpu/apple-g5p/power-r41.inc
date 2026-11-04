/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* J71t live ADT + ApplePMGR_restoreDeviceBridgeSettings (5a39918).
 * Volatile fabric MMIO only. GFX selects bridge 8, then bridge 9; the two
 * version-0 counter controls follow bridge 8's table, before bridge 9.
 * Keep devm_ioremap/readl/writel identical to boot.inc's swirq mapping.
 */
static const unsigned int gfx_bridge_replay = 0;

struct g5p_r41_write {
	u32 bridge, offset, value;
};

static const struct g5p_r41_write g5p_r41_writes[] = {
	{ 8, 0x0000, 0x00000120 }, { 8, 0x0018, 0x80010005 },
	{ 8, 0x001c, 0x00010001 }, { 8, 0x410c, 0x00000003 },
	{ 8, 0x4100, 0x00000003 }, { 9, 0x0000, 0x00697101 },
	{ 9, 0x0004, 0x00697101 }, { 9, 0x0008, 0x00697101 },
	{ 9, 0x000c, 0x00697101 }, { 9, 0x0010, 0x00697101 },
	{ 9, 0x0014, 0x00697101 }, { 9, 0x0018, 0x00697101 },
	{ 9, 0x001c, 0x00697101 }, { 9, 0x0020, 0x00697101 },
	{ 9, 0x0024, 0x00697101 }, { 9, 0x0028, 0x00697101 },
	{ 9, 0x002c, 0x00697101 }, { 9, 0x0030, 0x00697101 },
	{ 9, 0x0034, 0x00697101 }, { 9, 0x0038, 0x00697101 },
	{ 9, 0x003c, 0x00697101 }, { 9, 0x0040, 0x00697101 },
	{ 9, 0x0044, 0x00697101 }, { 9, 0x0048, 0x00697101 },
	{ 9, 0x004c, 0x00697101 }, { 9, 0x0050, 0x00697101 },
	{ 9, 0x0054, 0x00697101 }, { 9, 0x0058, 0x00697101 },
	{ 9, 0x005c, 0x00697101 }, { 9, 0x0060, 0x00697101 },
	{ 9, 0x0064, 0x00697101 }, { 9, 0x0068, 0x00697101 },
	{ 9, 0x006c, 0x00697101 }, { 9, 0x0070, 0x00697101 },
	{ 9, 0x0074, 0x00697101 }, { 9, 0x0078, 0x00697101 },
	{ 9, 0x007c, 0x00697101 },
};

static struct {
	void __iomem *window[2];
	unsigned int mode;
	int result;
	bool attempted, ready, firmware_logged, ta_logged;
} g5p_r41_bridge;

/* Reject anything outside the exact write allowlist, including unaligned
 * offsets and other registers inside the two mapped ADT ranges.
 */
static bool g5p_r41_valid(const struct g5p_r41_write *w)
{
	if (w->offset & 3)
		return false;
	if (w->bridge == 8)
		return w->offset < 0x5000 &&
		       (w->offset == 0 || w->offset == 0x18 ||
			w->offset == 0x1c || w->offset == 0x410c ||
			w->offset == 0x4100);
	return w->bridge == 9 && w->offset <= 0x7c;
}

static void g5p_r41_bridge_snapshot(struct g5p_diag *g, const char *stage)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(g5p_r41_writes); i++) {
		const struct g5p_r41_write *w = &g5p_r41_writes[i];
		u32 value =
			readl(g5p_r41_bridge.window[w->bridge - 8] + w->offset);

		dev_info(
			g->dev,
			"BRIDGE r41 stage=%s mode=%u index=%u bridge=%u off=%04x value=%08x want=%08x\n",
			stage, g5p_r41_bridge.mode, i, w->bridge, w->offset,
			value, w->value);
	}
}

/* Called under g->lock after ACTIVE (and any r33 off/on), before Fender.
 * Latch errors too: never retry a partial experiment in the same boot.
 */
static int g5p_r41_bridge_prepare(struct g5p_diag *g)
{
	unsigned int i, mode = gfx_bridge_replay;

	if (!mode)
		return 0;
	if (g5p_r41_bridge.attempted)
		return g5p_r41_bridge.result;
	g5p_r41_bridge.attempted = true;
	g5p_r41_bridge.mode = mode;
	g5p_r41_bridge.result = -EINVAL;
	if (mode > 4)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(g5p_r41_writes); i++)
		if (!g5p_r41_valid(&g5p_r41_writes[i]))
			return -EINVAL;
	g5p_r41_bridge.result = -ENOMEM;
	g5p_r41_bridge.window[0] = devm_ioremap(g->dev, 0x201f00000ULL, 0x5000);
	if (!g5p_r41_bridge.window[0])
		return -ENOMEM;
	g5p_r41_bridge.window[1] = devm_ioremap(g->dev, 0x201f80000ULL, 0x1000);
	if (!g5p_r41_bridge.window[1])
		return -ENOMEM;
	/* Read all 37 even in split modes; never read unmapped/partial state. */
	g5p_r41_bridge_snapshot(g, "before");
	for (i = 0; i < ARRAY_SIZE(g5p_r41_writes); i++) {
		const struct g5p_r41_write *w = &g5p_r41_writes[i];

		if (mode == 1 || (mode == 3 && w->bridge != 8) ||
		    (mode == 4 && w->bridge != 9))
			continue;
		writel(w->value,
		       g5p_r41_bridge.window[w->bridge - 8] + w->offset);
		dev_info(
			g->dev,
			"BRIDGE r41 write mode=%u index=%u bridge=%u off=%04x value=%08x\n",
			mode, i, w->bridge, w->offset, w->value);
	}
	g5p_r41_bridge_snapshot(g, "after");
	g5p_r41_bridge.ready = true;
	g5p_r41_bridge.result = 0;
	return 0;
}

static void g5p_r41_bridge_firmware(struct g5p_diag *g)
{
	if (g5p_r41_bridge.ready && !g5p_r41_bridge.firmware_logged) {
		g5p_r41_bridge.firmware_logged = true;
		g5p_r41_bridge_snapshot(g, "firmware-alive");
	}
}

static void g5p_r41_bridge_ta(struct g5p_diag *g)
{
	if (g5p_r41_bridge.ready && !g5p_r41_bridge.ta_logged) {
		g5p_r41_bridge.ta_logged = true;
		g5p_r41_bridge_snapshot(g, "first-TA-kick");
	}
}

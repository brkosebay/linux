/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Shared pre-kick step recovered from ca0358/ca05d4 and ca6644. */
static int g5p_prepare_kick(struct g5p_diag *g, const char *name)
{
	u32 raw, state;

	if (gfx_bridge_replay && !strcmp(name, "RENDER-TA"))
		g5p_r41_bridge_ta(g);
	/* ca05d4 always invokes Fender +80 before inspecting GPC state.
	 * ca6644: W R1+14=7, then read fence. Hardware bit names unknown.
	 * This is distinct from acknowledging the host AIC interrupt.
	 */
	dev_info(g->dev, "%s Fender W R1+00014=00000007 before MTS kick\n",
		 name);
	writel(7, g->window[1] + 0x14);
	raw = readl(g->window[1] + 0x14);
	dev_info(g->dev, "%s Fender raw R1+00014=%08x (write fence)\n", name,
		 raw);
	if (raw & ~7U)
		return -EPROTO;
	state = readl(g->window[1] + 0x1000);
	if (!g5p_gpc_ok(state))
		return -EHOSTDOWN;
	if (READ_ONCE(g->boot->failed))
		return -EIO;
	return 0;
}

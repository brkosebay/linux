// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Kernel glue for the incoming dirty-start machine (dirtystart108.h), series
 * gpu-recovery108 (0007).  Design: docs/gpu/experiments/recovery108-driver.md.
 *
 * Included from core.inc after teardown.inc, so it may use the teardown ring
 * read (RAM), g5p_r33_gfx_off() and the retire layer's state.  Patch 0008 calls
 * the hooks from the driver steps; until then nothing here runs.
 *
 * Register access added by this file: one PMGR read of the gfx word after the
 * PM reference (POWER, the same read g5p_power_snapshot() makes), and in mode 3
 * only one bounded PMGR cycle through g5p_r33_gfx_off() before the first
 * wrapper wake.  Every other input is RAM or a step's own return value.
 */
#include <linux/of.h>
#include "dirtystart108.h"

static struct g5p_ds g5p_ds_state = { .last = G5P_DSE_N };
static DEFINE_SPINLOCK(g5p_ds_lock);

static u32 g5p_ds_mode(void)
{
	return min_t(u32, (1), G5P_DS_MODE_MAX);
}

/* One event, one DS108 line (dev_emerg: netconsole at any loglevel). */
static int g5p_ds_feed(struct g5p_diag *g, enum g5p_ds_ev e,
		       const struct g5p_ds_in *in)
{
	unsigned long flags;
	char line[224];
	int ret;

	spin_lock_irqsave(&g5p_ds_lock, flags);
	ret = g5p_ds_event(&g5p_ds_state, e, in, g5p_ds_mode(), line,
			   sizeof(line));
	spin_unlock_irqrestore(&g5p_ds_lock, flags);
	dev_emerg(g->dev, "%s\n", line);
	return ret;
}

/* wake112: the current reset generation, recorded with a deferred R74E proof (RAM, leaf lock). */
static u32 g5p_ds_gen(void)
{
	unsigned long flags;
	u32 gen;

	spin_lock_irqsave(&g5p_ds_lock, flags);
	gen = g5p_ds_state.gen;
	spin_unlock_irqrestore(&g5p_ds_lock, flags);
	return gen;
}

/* The last apple_pmgr.kexec_gpu_cycle= token of the chosen bootargs (the one the kernel obeys). */
static bool g5p_ds_cycle_requested(struct device_node *chosen)
{
	static const char key[] = "apple_pmgr.kexec_gpu_cycle=";
	const char *args, *p, *last = NULL;
	char tok[8];
	bool v = false;
	size_t n;

	if (!chosen || of_property_read_string(chosen, "bootargs", &args))
		return false;
	for (p = strstr(args, key); p; p = strstr(p + 1, key))
		if (p == args || p[-1] == ' ')
			last = p;
	if (!last)
		return false;
	last += sizeof(key) - 1;
	n = strcspn(last, " ");
	if (!n || n >= sizeof(tok))
		return false;
	memcpy(tok, last, n);
	tok[n] = 0;
	return !kstrtobool(tok, &v) && v;
}

static void g5p_ds_hook_probe(struct g5p_diag *g)
{
	struct device_node *chosen = of_find_node_by_path("/chosen");
	struct g5p_ds_in in = {};

	in.kexec = chosen &&
		   of_property_read_bool(chosen, "linux,booted-from-kexec");
	in.cycle_requested = g5p_ds_cycle_requested(chosen);
	of_node_put(chosen);
	g5p_ds_feed(g, G5P_DSE_PROBE, &in);
}

/* After the PM reference is taken: one PMGR read (the gfx word), no write. */
static void g5p_ds_hook_power(struct g5p_diag *g)
{
	struct g5p_ds_in in = {};
	u32 raw = 0;

	in.pmgr_ok = g->pmgr && !regmap_read(g->pmgr, 0x80340, &raw);
	in.pmgr_raw = raw;
	g5p_ds_feed(g, G5P_DSE_POWER, &in);
}

/* Before the first wrapper wake.  Mode 3 only, one attempt per boot, through the
 * r33 helper (its own refusal rules: min/reset/auto or not on; every exit leaves
 * the gate requested on).  Returns the cycle's error; the wake then fails closed.
 */
static int g5p_ds_hook_reset(struct g5p_diag *g)
{
	u32 ms = clamp_t(u32, (10), 1, G5P_R33_GFX_OFF_MAX_MS);
	struct g5p_ds_in in = {};
	unsigned long flags;
	bool want;

	spin_lock_irqsave(&g5p_ds_lock, flags);
	want = g5p_ds_reset_wanted(&g5p_ds_state, g5p_ds_mode());
	spin_unlock_irqrestore(&g5p_ds_lock, flags);
	if (!want)
		return 0;
	g5p_mmio_gate(g);
	in.ret = g5p_r33_gfx_off(g->dev, g->pmgr, ms);
	g5p_ds_feed(g, G5P_DSE_RESET, &in);
	boot_gfx_off_done = true; /* one PMGR cycle per boot, whoever runs it */
	return in.ret;
}

/* The legacy boot_gfx_off_ms cycle (R33) is the same mechanism: record it as the reset. */

/* An in-boot recovery's gate cycle (recover.inc): a new generation, or a failed machine. */
static int g5p_ds_hook_recover(struct g5p_diag *g, int ret)
{
	struct g5p_ds_in in = { .ret = ret };
	unsigned long flags;
	bool failed;

	g5p_ds_feed(g, G5P_DSE_RECOVER, &in);
	spin_lock_irqsave(&g5p_ds_lock, flags);
	failed = g5p_ds_state.failed;
	spin_unlock_irqrestore(&g5p_ds_lock, flags);
	return failed ? -EIO : 0;
}

static void g5p_ds_hook_wake(struct g5p_diag *g, int ret)
{
	struct g5p_ds_in in = { .ret = ret };

	g5p_ds_feed(g, G5P_DSE_WAKE, &in);
}

/* Fresh shared state: every ring control word in range (RAM, the teardown ring read). */
static void g5p_ds_hook_maps(struct g5p_diag *g, int ret)
{
	struct g5p_ds_in in = { .ret = ret };
	u32 i, cons, prod;

	for (i = 0; !ret && i < G5P_TD_NRING; i++) {
		if (g5p_td_op_ring_read(g, i, &cons, &prod)) {
			in.rings_bad++;
			continue;
		}
		if (cons > 255 || prod > 255)
			in.rings_bad++;
		else if (cons != prod)
			in.rings_busy++;
	}
	g5p_ds_feed(g, G5P_DSE_MAPS, &in);
}

static void g5p_ds_hook_load(struct g5p_diag *g, int ret)
{
	struct g5p_ds_in in = { .ret = ret };

	g5p_ds_feed(g, G5P_DSE_LOAD, &in);
}

/* Asked before the release touches anything.  -EIO: the fail-closed latch; -EAGAIN: strict origin. */
static int g5p_ds_hook_release_gate(struct g5p_diag *g)
{
	enum g5p_ds_gate gate;
	unsigned long flags;
	u32 mode = g5p_ds_mode(), gen;

	spin_lock_irqsave(&g5p_ds_lock, flags);
	gate = g5p_ds_release_gate(&g5p_ds_state, mode);
	g5p_ds_state.last_gate = gate;
	if (gate != G5P_DSG_OPEN)
		g5p_ds_state.refused++;
	gen = g5p_ds_state.gen;
	spin_unlock_irqrestore(&g5p_ds_lock, flags);
	dev_emerg(g->dev, "DS108 gen=%u gate=release mode=%u answer=%s\n", gen,
		  mode, g5p_ds_gate_name[gate]);
	if (gate == G5P_DSG_OPEN)
		return 0;
	return gate == G5P_DSG_FAILED ? -EIO : -EAGAIN;
}

static void g5p_ds_hook_release(struct g5p_diag *g, int ret)
{
	struct g5p_r73b *r = g->boot ? g->boot->r73b : NULL;
	struct g5p_ds_in in = { .ret = ret };

	if (r) {
		in.power_ret = r->early_power_ret;
		in.power_raw = r->early_power_raw;
		in.power_abort = READ_ONCE(r->early_power_abort);
	}
	g5p_ds_feed(g, G5P_DSE_RELEASE, &in);
}

static void g5p_ds_hook_alive(struct g5p_diag *g, int ret)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b ? b->r73b : NULL;
	struct g5p_ds_in in = { .ret = ret };

	if (b) {
		in.alive = READ_ONCE(b->alive);
		in.released = READ_ONCE(b->released);
		in.failed = READ_ONCE(b->failed);
	}
	if (r) {
		in.power_abort = READ_ONCE(r->early_power_abort);
		/* r73e reads R1+0x1000 at the first ALIVE; 0 means not read. */
		in.gpc = READ_ONCE(r->early_state);
		in.gpc_valid = 1 && in.gpc;
	}
	g5p_ds_feed(g, G5P_DSE_ALIVE, &in);
}

/* The DRM claim: 0 expose, -EAGAIN not proven / strict origin, -EIO the fail-closed latch. */
static int __maybe_unused g5p_ds_hook_claim(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	struct g5p_ds_in in = {};

	if (b) {
		in.alive = READ_ONCE(b->alive);
		in.released = READ_ONCE(b->released);
		in.failed = READ_ONCE(b->failed) || READ_ONCE(b->event_failed);
		in.control_complete = READ_ONCE(b->control_complete);
		in.power_abort = b->r73b &&
				 READ_ONCE(b->r73b->early_power_abort);
	}
	return g5p_ds_feed(g, G5P_DSE_CLAIM, &in);
}

/* RAM only: the retire layer and the dirty-start machine, for capture without a log. */
static ssize_t recovery_status_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	const struct g5p_r8 *s = &g5p_r8_state;
	unsigned long flags;
	struct g5p_ds d;

	spin_lock_irqsave(&g5p_ds_lock, flags);
	d = g5p_ds_state;
	spin_unlock_irqrestore(&g5p_ds_lock, flags);
	return sysfs_emit(
		buf,
		"r8 gen=%u retired=%d mode=%s class=%s verdict=%s flags=%02x admit_closed=%u "
		"inside=%u detach_queued=%u lines=%u\n"
		"ds gen=%u mode=%u origin=%s failed=%u why=%s seen=%03x ok=%03x flags=%02x "
		"pmgr=%08x alive_gen=%u resets=%u reset_ok_gen=%u events=%u claims=%u refused=%u "
		"gate=%s\n",
		READ_ONCE(s->gen), READ_ONCE(s->retired),
		g5p_td_mode_name[s->mode], g5p_r8_class_name[s->cls],
		g5p_td_verdict_name[s->verdict], s->flags,
		g5p_r8_admit_closed(&s->admit), g5p_r8_admit_inside(&s->admit),
		READ_ONCE(s->detach_queued), s->lines, d.gen, g5p_ds_mode(),
		g5p_ds_origin_name[d.origin], d.failed, g5p_ds_why_name[d.why],
		d.seen, d.ok, d.flags, d.pmgr_raw, d.alive_gen, d.resets,
		d.reset_ok_gen, d.events, d.claims, d.refused,
		g5p_ds_gate_name[d.last_gate]);
}
static DEVICE_ATTR_RO(recovery_status);

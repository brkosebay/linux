/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* R74g. Included by boot.inc after boot-r74f.inc; r74f case 1 only. It is a
 * diagnostic bypass on the forced-ready epoch, not power policy.
 *
 * The firmware DC3 handler (0x1000218c..0x10002218) writes e140/e148 for
 * slot N, copies host_generation[N] to cached_generation[N], then issues
 * the e138 (N << 3) | 4 catalogue invalidation and waits for it. The DC
 * dispatcher advances the consumer only after the handler returns, so a
 * consumed DC3 pair is the completion receipt. r74g checks the RAM-visible
 * results and issues no host MMU MMIO. The natural TA is then published
 * without a doorbell. A RAM-only grace (four type-0 ALIVEs, 20 ms cap)
 * lets the firmware drain it on its own. If it stays pending, the r74f
 * ALIVE owner gets one fenced fallback doorbell. Receipts print afterwards.
 */

/* perf60: with the policy held in state 0 the firmware is quiet (no ALIVE stream, no self-drain), so
 * job 1's TA bell, owned by "the first ALIVE after the grace", would never ring.
 */
/* release8 default */

static const char *const g5p_r74g_graces[G5P_R74G_GRACES] = {
	"none", "consumed", "alives", "wall", "fault",
};

static bool g5p_r74g_on(struct g5p_boot *b)
{
	return g5p_r74f_on(b) && 1;
}

static u32 g5p_r74g_root(struct g5p_boot *b, u32 off)
{
	return g5p_le32(b->root[off / G5P_GPU_PAGE].cpu + off % G5P_GPU_PAGE);
}

static u32 g5p_r74g_shared(struct g5p_boot *b, u32 at)
{
	return g5p_le32(b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE);
}

/* RAM only: the DC3 ring, both records in attach order (3D, then TA), the
 * PA[N] entries, host and cached tags, both failure latches and the
 * root+3980 companion word, whose value 2 is firmware recovery/escape.
 */
static u32 g5p_r74g_check(struct g5p_diag *g, u32 phase, const u32 *slot,
			  u64 pa, u32 tag, u32 start)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r74g_dc3 *d = &b->r73b->r74g_dc3[phase];
	u32 *ctl = g5p_r74f_ctl(b, G5P_R74F_DC3);
	u32 bad = 0;

	dma_rmb();
	d->start = start;
	d->target = (start + 2) & 255;
	d->tag = tag;
	d->expected_pa = pa;
	d->consumer = READ_ONCE(ctl[0]);
	d->producer = READ_ONCE(ctl[4]);
	if (d->consumer != d->target || d->producer != d->target)
		bad |= G5P_R74G_BAD_RING;
	for (u32 i = 0; i < 2; i++) {
		u32 at = b->layout.offset[0x6d] + ((start + i) & 255) * 20;

		for (u32 j = 0; j < 5; j++) {
			d->rec[i][j] = g5p_r74g_shared(b, at + j * 4);
			if (d->rec[i][j] != (j == 0 ? 3 :
					     j == 1 ? slot[1 - i] :
						      0))
				bad |= G5P_R74G_BAD_RECORD;
		}
	}
	for (u32 e = 0; e < 2; e++) {
		u8 *p = g5p_boot_shared(b, 0x86) + slot[e] * 8;

		d->slot[e] = slot[e];
		d->pa[e] = (u64)g5p_le32(p + 4) << 32 | g5p_le32(p);
		d->host[e] = g5p_le32(g5p_boot_shared(b, 0x84) + slot[e] * 4);
		d->cached[e] = g5p_le32(g5p_boot_shared(b, 0x85) + slot[e] * 4);
		if (d->pa[e] != pa)
			bad |= G5P_R74G_BAD_PA;
		if (d->host[e] != tag)
			bad |= G5P_R74G_BAD_HOST;
		if (d->cached[e] != tag)
			bad |= G5P_R74G_BAD_CACHED;
	}
	d->failed = READ_ONCE(b->failed);
	d->event_failed = READ_ONCE(b->event_failed);
	if (d->failed || d->event_failed)
		bad |= G5P_R74G_BAD_FAILED;
	d->companion = g5p_r74g_root(b, G5P_R74G_COMPANION);
	d->recoveries = g5p_r74g_root(b, G5P_R74G_RECOVERIES);
	d->classification = g5p_r74g_root(b, G5P_R74G_CLASS);
	if (d->companion == 2)
		bad |= G5P_R74G_BAD_COMPANION;
	d->bad = bad;
	d->t_ns = ktime_get_mono_fast_ns();
	return bad;
}

/* A refused check is retained, never b->failed: companion 2 is firmware
 * recovery, a cached-tag-only mismatch is the firmware tag copy, and
 * anything else (a host tag included) is a refused RAM validation.
 */
static int g5p_r74g_refuse(struct g5p_diag *g, u32 bad)
{
	if (bad & G5P_R74G_BAD_COMPANION)
		return g5p_r74f_stop(g, G5P_R74G_STOP_COMPANION, bad);
	if (bad == G5P_R74G_BAD_CACHED)
		return g5p_r74f_stop(g, G5P_R74G_STOP_DC3_TAGS, bad);
	return g5p_r74f_stop(g, G5P_R74G_STOP_DC3_RAM, bad);
}

/* Two bounded lines: every field at full width still ends in end=r74g. */
static void g5p_r74g_dc3_line(struct g5p_diag *g, u32 phase, bool blob)
{
	const struct g5p_r74g_dc3 *d = &g->boot->r73b->r74g_dc3[phase];
	const char *name = phase ? "ta-preflight" : "dc3-consume";
	char line[2][512];

	scnprintf(
		line[0], sizeof(line[0]),
		"R74G dc3 phase=%s bad=%02x start=%u target=%u consumer=%u producer=%u tag=%08x "
		"slot=%u/%u host=%08x/%08x cached=%08x/%08x failed=%u event_failed=%u "
		"companion=%u recoveries=%u class=%u host_mmu=skipped t_ns=%llu end=r74g\n",
		name, d->bad, d->start, d->target, d->consumer, d->producer,
		d->tag, d->slot[0], d->slot[1], d->host[0], d->host[1],
		d->cached[0], d->cached[1], d->failed, d->event_failed,
		d->companion, d->recoveries, d->classification, d->t_ns);
	scnprintf(
		line[1], sizeof(line[1]),
		"R74G dc3-ram phase=%s pa=%llx/%llx expected_pa=%llx rec0=%x,%x,%x,%x,%x "
		"rec1=%x,%x,%x,%x,%x end=r74g\n",
		name, d->pa[0], d->pa[1], d->expected_pa, d->rec[0][0],
		d->rec[0][1], d->rec[0][2], d->rec[0][3], d->rec[0][4],
		d->rec[1][0], d->rec[1][1], d->rec[1][2], d->rec[1][3],
		d->rec[1][4]);
	for (u32 i = 0; i < ARRAY_SIZE(line); i++) {
		if (blob)
			g5p_blob_printf(g, "%s", line[i]);
		else
			dev_emerg(g->dev, "%s", line[i]);
	}
}

static void g5p_r74g_ta_line(struct g5p_diag *g, bool blob)
{
	struct g5p_r73b *r = g->boot->r73b;
	const struct g5p_r74f_bell *f = &r->r74f_bell[G5P_R74F_TA];
	u32 end = r->r74g_grace_end;
	char line[256];

	scnprintf(
		line, sizeof(line),
		"R74G ta grace_end=%s arm_alive=%u grace_alive=%u grace_ns=%llu opened=%u "
		"open_alive=%u ack_skipped=%u end=r74g\n",
		g5p_r74g_graces[end < G5P_R74G_GRACES ? end : 0], f->arm_alive,
		r->r74g_grace_alive, r->r74g_grace_ns - f->arm_ns,
		r->r74g_opened, f->open_alive, r->r74g_ack_skipped);
	if (blob)
		g5p_blob_printf(g, "%s", line);
	else
		dev_emerg(g->dev, "%s", line);
}

/* Process context, command mutex held, TA producer already published once.
 * Grace: RAM reads and sleeps only, no MMIO and no doorbell, while the IRQ
 * thread keeps decoding. It ends at the consume, four type-0 ALIVEs after
 * the arm, or the 20 ms wall cap. If the TA is still pending, the
 * generation opens, and the first ALIVE decoded after that owns the single
 * r74f fallback attempt. Returns 0 after one doorbell, 1 if consumed without
 * one, -EINPROGRESS for a retained boundary, else a fatal error.
 */
static int g5p_r74g_ta(struct g5p_diag *g, u32 start, u32 target)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	struct g5p_r74f_bell *f = &r->r74f_bell[G5P_R74F_TA];
	u32 *ctl = g5p_r74f_ctl(b, G5P_R74F_TA);
	unsigned long flags, deadline;
	u32 end = G5P_R74G_GRACE_NONE, now;
	int ret = 0;

	f->kind = G5P_R74F_TA;
	f->ring = 0x60;
	f->start = start;
	f->target = target;
	f->generation = ++r->r74f_generation;
	f->consumer_before = READ_ONCE(ctl[0]);
	f->arm_ns = ktime_get_mono_fast_ns();
	spin_lock_irqsave(&b->event_lock, flags);
	f->arm_alive = r->r74f_alive;
	WRITE_ONCE(r->r74f_active, G5P_R74F_TA + 1);
	atomic_set_release(&f->state, G5P_R74F_PUBLISHED);
	spin_unlock_irqrestore(&b->event_lock, flags);
	while (end == G5P_R74G_GRACE_NONE) {
		;
		dma_rmb();
		if (READ_ONCE(b->failed) || READ_ONCE(b->event_failed))
			end = G5P_R74G_GRACE_FAULT;
		else if (READ_ONCE(ctl[0]) == target)
			end = G5P_R74G_GRACE_CONSUMED;
		else if (READ_ONCE(r->r74f_alive) - f->arm_alive >=
			 G5P_R74G_GRACE_ALIVES)
			end = G5P_R74G_GRACE_ALIVES_END;
		else if (ktime_get_mono_fast_ns() - f->arm_ns >=
			 (u64)G5P_R74G_GRACE_US * 1000)
			end = G5P_R74G_GRACE_WALL;
		else
			usleep_range(100, 200);
	}
	spin_lock_irqsave(&b->event_lock, flags);
	r->r74g_grace_end = end;
	r->r74g_grace_alive = r->r74f_alive;
	r->r74g_grace_ns = ktime_get_mono_fast_ns();
	dma_rmb();
	if (end == G5P_R74G_GRACE_FAULT) {
		f->cancel = G5P_R74F_CANCEL_PROTOCOL;
		atomic_set_release(&f->state, G5P_R74F_CANCELLED);
	} else if (READ_ONCE(ctl[0]) == target) {
		/* Drained by the firmware policy loop: never ring. */
		f->cancel = G5P_R74F_CANCEL_CONSUMED;
		atomic_set_release(&f->state, G5P_R74F_CANCELLED);
	} else {
		r->r74g_opened = 1;
		f->open_alive = r->r74f_alive;
		f->open_ns = r->r74g_grace_ns;
		/* The next decoded type-0 ALIVE may now take ownership. */
		atomic_set_release(&f->state, G5P_R74F_OPEN);
		/* ta1_host_bell: the host owns it instead, in the same lock section a decoder would. */
		if ((1) && atomic_cmpxchg(&f->state, G5P_R74F_OPEN,
					  G5P_R74F_FIRING) == G5P_R74F_OPEN) {
			u32 fence = U32_MAX, state = U32_MAX;
			int rung;

			f->bell_alive = r->r74f_alive;
			f->attempted = 1;
			f->bell_start_ns = ktime_get_mono_fast_ns();
			rung = g5p_r74f_ring(g, f, &fence, &state);
			g5p_r74f_rang(r, f, rung, fence, state,
				      G5P_R74F_PATH_IMMEDIATE);
		}
	}
	spin_unlock_irqrestore(&b->event_lock, flags);
	if (end == G5P_R74G_GRACE_FAULT)
		ret = -EIO;
	deadline = jiffies + msecs_to_jiffies(G5P_R74F_BELL_MS);
	while (atomic_read_acquire(&f->state) == G5P_R74F_OPEN &&
	       time_before(jiffies, deadline)) {
		if (READ_ONCE(b->failed) || READ_ONCE(b->event_failed)) {
			ret = -EIO;
			break;
		}
		usleep_range(100, 200);
	}
	/* FIRING exists only inside the decoder's event_lock section. */
	spin_lock_irqsave(&b->event_lock, flags);
	if (atomic_read(&f->state) == G5P_R74F_OPEN) {
		f->cancel = ret ? G5P_R74F_CANCEL_PROTOCOL :
				  G5P_R74F_CANCEL_NO_ALIVE;
		atomic_set_release(&f->state, G5P_R74F_CANCELLED);
	}
	spin_unlock_irqrestore(&b->event_lock, flags);
	now = atomic_read_acquire(&f->state);
	g5p_r74_mark(g, G5P_R74G_TA_GRACE, end);
	if (r->r74g_opened)
		g5p_r74_mark(g, G5P_R74G_TA_FALLBACK,
			     now == G5P_R74F_RANG ? f->bell_ret : -ECANCELED);
	g5p_r74f_line(g, f, false);
	g5p_r74g_ta_line(g, false);
	if (ret)
		return ret;
	if (now == G5P_R74F_CANCELLED) {
		if (f->cancel == G5P_R74F_CANCEL_CONSUMED)
			return 1;
		return g5p_r74f_stop(g, G5P_R74G_STOP_TA_NOT_CONSUMED,
				     -ETIMEDOUT);
	}
	if (f->kicked)
		return 0;
	if (f->bell_ret == -EALREADY)
		return 1;
	if (f->bell_ret == -EHOSTDOWN)
		return g5p_r74f_stop(g, G5P_R74G_STOP_TA_WINDOW_MISSED,
				     -EHOSTDOWN);
	return f->bell_ret;
}

/* r73b-export, process context, RAM only. */

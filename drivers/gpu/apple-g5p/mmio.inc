/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * The one GPU register accessor (docs/gpu/experiments/mmio-access.md §4), Apple's licence from
 * AGXMetaFirmware_isGPURegisterAccessible (mmio-licence.h) under g->mmio_lock:
 *
 *   if (!g5p_mmio_hold(g, &t, 0)) {
 *           v = g5p_mmio_r0_read(g, &t, off);	// <= G5P_MMIO_MAX_OPS, nothing else in between
 *           g5p_mmio_release(g, &t);		// false: the licence changed under the sample
 *   }
 *
 * hold takes mmio_lock with IRQs off and returns with it held on success.  It refuses instead of
 * waiting and never wakes the GPU (no R1+0x14 write): -ENODEV when the power epoch is not
 * readable (RAM only, no MMIO done), -EAGAIN when parked, bit 29, in transition, above the
 * pstate table, or (G5P_MMIO_TRY) when the lock is taken; -EPERM for an R0 ticket in an epoch
 * the firmware may park (boot_gpu_perf < 100, unless mmio_r0_park).  Callable from hard IRQ,
 * threaded IRQ and process context, inside event_lock and g->lock.  Between hold and release: no
 * printk, no other MMIO, no sleep, no allocation.
 *
 * mmio_selftest (0600, default 0, live) drives the proof rungs of §6:
 *   bit 0  rung 1/1b: 1 kHz hrtimer census, R1+0 and R1+0x1000 only (no R0)
 *   bit 1  rung 3/4:  every 100 ms hold -> R0+0x20, R0+0x18 -> release (DFU-risk with parks: rung 4)
 *   bit 2  rung 5:    the same at every 3D completion decision (drm side; DFU-risk, park tuple)
 *   bit 3  idle check: after 5, 30 and 120 s with no job, the PMGR gfx word, then (gfx on only)
 *          R1+0x1000 and R1+0; never R0
 * Lines: GPU-MMIO sample (per sample, at most one per source per second unless the outcome
 * changes or the sample is stale/bad), GPU-MMIO idle, GPU-MMIO stats (every 10 s while on and
 * once when it goes back to 0).
 */
#define G5P_MMIO_TRY \
	BIT(0) /* trylock: refuse busy instead of spinning (the IRQ census) */
/* An R1-only ticket, counted in the census class: held whenever the RAM terms pass (R1 is
 * readable while PMGR gfx is on, mmio-access.md §1.3), R1+0 and R1+0x1000 both read even when the
 * licence refuses; R0 only if the licence passed (t->r0_ok).
 */
#define G5P_MMIO_CENSUS BIT(1)

struct g5p_mmio_ticket {
	u32 r1_0, gpc, r1_0_end, gpc_end, flags, ops;
	u64 t_ns, t_end_ns;
	unsigned long irq;
	enum g5p_mmio_why why;
	bool held, stale, r0_ok;
};

enum {
	G5P_MMS_CENSUS,
	G5P_MMS_IDLE,
	G5P_MMS_3D,
	G5P_MMS_FAULT,
	G5P_MMS_PROBE,
	G5P_MMS_NR
};
static const char *const g5p_mms_names[G5P_MMS_NR] = {
	"census", "idle-100ms", "3d-done", "fault-read", "dc-probe",
};

#define G5P_MMIO_RING 32
struct g5p_mmio_rec {
	u64 t_ns;
	u32 r1_0, gpc, us;
	u8 why, stale;
};

struct g5p_mmio_self {
	struct g5p_diag *g;
	struct hrtimer timer;
	struct delayed_work work;
	bool census_on, was_on;
	u64 stats_ns;
	/* counters: [0] R1-only census tickets, [1] R0 tickets */
	atomic_t n[2][G5P_MMIO_NR];
	atomic_t stale[2], r0_ops, bad;
	/* census transitions, written by the hrtimer, drained by the work */
	spinlock_t ring_lock;
	struct g5p_mmio_rec ring[G5P_MMIO_RING];
	u32 ring_head, ring_tail, ring_drop;
	struct g5p_mmio_rec census_last;
	u8 census_prev_why, census_prev_stale;
	/* per-source print limiter */
	u64 printed_ns[G5P_MMS_NR];
	u8 printed_why[G5P_MMS_NR];
	/* idle check */
	u64 job_ns;
	u32 idle_done;
	/* bit 4, the GPU clock: per Fender pstate nibble, the highest rate seen and the sample counts */
	u32 clk_max_khz[16], clk_cr_khz[16], clk_n[16], clk_busy[16], clk_reset;
	u64 clk_printed_ns;
};

/* mmio_selftest bit 4: the gap between the two tickets of one clock sample */

/* R0 tickets in a park epoch (boot_gpu_perf < 100): refused unless 1 (g5p_mmio_r0_regime_ok). */

static u32 g5p_mmio_rd_r1(void *ctx, u32 off)
{
	return readl((void __iomem *)ctx + off);
}

static void g5p_mmio_count(struct g5p_diag *g, const struct g5p_mmio_ticket *t)
{
	struct g5p_mmio_self *s = READ_ONCE(g->mmio_self);
	u32 cls = t->flags & G5P_MMIO_CENSUS ? 0 : 1;

	if (!s)
		return;
	atomic_inc(&s->n[cls][t->why]);
	if (t->stale)
		atomic_inc(&s->stale[cls]);
	if (t->ops && cls)
		atomic_add(min_t(u32, t->ops, G5P_MMIO_MAX_OPS), &s->r0_ops);
}

static int g5p_mmio_hold(struct g5p_diag *g, struct g5p_mmio_ticket *t,
			 unsigned int flags)
{
	struct g5p_mmio_ram ram = { 0 };
	struct g5p_boot *b;
	unsigned long irq;

	memset(t, 0, sizeof(*t));
	t->flags = flags;
	t->r1_0 = t->gpc = t->r1_0_end = t->gpc_end = U32_MAX;
	if (flags & G5P_MMIO_TRY) {
		if (!spin_trylock_irqsave(&g->mmio_lock, irq)) {
			t->why = G5P_MMIO_BUSY;
			goto refused;
		}
	} else {
		spin_lock_irqsave(&g->mmio_lock, irq);
	}
	t->t_ns = ktime_get_mono_fast_ns();
	ram.mmio_off = g->mmio_off;
	ram.powered = READ_ONCE(g->powered);
	ram.fender_ready = READ_ONCE(g->fender_ready);
	/* mmio_off first: a boot struct behind a gate is never dereferenced */
	b = ram.mmio_off ? NULL : READ_ONCE(g->boot);
	if (b) {
		ram.have_boot = true;
		ram.released = READ_ONCE(b->released);
		ram.stopped = READ_ONCE(b->stopped);
	}
	/* before any bus access: an R0 ticket in a park epoch is refused outright */
	if (!(flags & G5P_MMIO_CENSUS) && b &&
	    !g5p_mmio_r0_regime_ok(b->gpu_perf, (0))) {
		t->why = G5P_MMIO_PARKEPOCH;
		t->t_end_ns = ktime_get_mono_fast_ns();
		spin_unlock_irqrestore(&g->mmio_lock, irq);
		goto refused;
	}
	t->why = g5p_mmio_licence(&ram, READ_ONCE(g->mmio_max_pstate),
				  g5p_mmio_rd_r1, g->window[1], &t->r1_0,
				  &t->gpc);
	if (t->why && (!(flags & G5P_MMIO_CENSUS) || t->why == G5P_MMIO_OFF)) {
		t->t_end_ns = ktime_get_mono_fast_ns();
		spin_unlock_irqrestore(&g->mmio_lock, irq);
		goto refused;
	}
	if (t->gpc == U32_MAX)
		t->gpc = readl(
			g->window[1] +
			G5P_MMIO_R1_GPC); /* census: after an R1+0 refusal */
	t->r0_ok = !t->why;
	t->irq = irq;
	t->held = true;
	return 0;
refused:
	g5p_mmio_count(g, t);
	/* a park epoch does not change by waiting: -EPERM, never retried (g5p_mmio_hold_wait) */
	return t->why == G5P_MMIO_OFF	    ? -ENODEV :
	       t->why == G5P_MMIO_PARKEPOCH ? -EPERM :
					      -EAGAIN;
}

/* Only between a successful hold and its release; past G5P_MMIO_MAX_OPS no bus access. */
static u32 g5p_mmio_r0_read(struct g5p_diag *g, struct g5p_mmio_ticket *t,
			    u32 off)
{
	if (!t->held || !t->r0_ok || !g5p_mmio_op_take(&t->ops))
		return U32_MAX;
	return readl(g->rgx + off);
}

/* R0 writes count as ops too (the slave-port request words of the pin-200 probe). */
static bool g5p_mmio_r0_write(struct g5p_diag *g, struct g5p_mmio_ticket *t,
			      u32 off, u32 v)
{
	if (!t->held || !t->r0_ok || !g5p_mmio_op_take(&t->ops))
		return false;
	writel(v, g->rgx + off);
	return true;
}

/* Any R1 word under a held ticket (census or licensed); counts as an op. */
static u32 g5p_mmio_r1_read(struct g5p_diag *g, struct g5p_mmio_ticket *t,
			    u32 off)
{
	if (!t->held || !g5p_mmio_op_take(&t->ops))
		return U32_MAX;
	return readl(g->window[1] + off);
}

/* Process context: refusals (-EAGAIN) retried outside the lock every 20-40 us, <= budget_us. */
static int g5p_mmio_hold_wait(struct g5p_diag *g, struct g5p_mmio_ticket *t,
			      unsigned int flags, u32 budget_us, u32 *tries)
{
	u64 t0 = ktime_get_mono_fast_ns();
	int ret;

	might_sleep();
	for (;;) {
		(*tries)++;
		ret = g5p_mmio_hold(g, t, flags);
		if (ret != -EAGAIN || ktime_get_mono_fast_ns() - t0 >=
					      (u64)budget_us * NSEC_PER_USEC)
			return ret;
		usleep_range(20, 40);
	}
}

static bool g5p_mmio_release(struct g5p_diag *g, struct g5p_mmio_ticket *t)
{
	if (!t->held)
		return false;
	t->gpc_end = readl(g->window[1] + G5P_MMIO_R1_GPC);
	t->r1_0_end = readl(g->window[1] + G5P_MMIO_R1_CTRL);
	t->stale = t->r0_ok &&
		   g5p_mmio_stale(t->r1_0, t->gpc, t->r1_0_end, t->gpc_end,
				  READ_ONCE(g->mmio_max_pstate));
	t->t_end_ns = ktime_get_mono_fast_ns();
	t->held = false;
	spin_unlock_irqrestore(&g->mmio_lock, t->irq);
	g5p_mmio_count(g, t);
	return !t->stale;
}

static const char *g5p_mmio_did(const struct g5p_mmio_ticket *t)
{
	return t->stale ? "stale" : g5p_mmio_why_names[t->why];
}

static u32 g5p_mmio_us(const struct g5p_mmio_ticket *t)
{
	return t->t_end_ns > t->t_ns ?
		       (u32)div_u64(t->t_end_ns - t->t_ns, NSEC_PER_USEC) :
		       0;
}

/* The per-sample line, after release.  force: print regardless of the per-source limiter. */
static void g5p_mmio_sample_line(struct g5p_diag *g, u32 src,
				 const struct g5p_mmio_ticket *t, u32 *v,
				 u32 nv, bool force)
{
	struct g5p_mmio_self *s = g->mmio_self;
	u64 now = ktime_get_mono_fast_ns();
	char vs[9 * G5P_MMIO_MAX_OPS + 1] = "-";
	u32 len = 0;

	if (s && !force && !t->stale && s->printed_why[src] == t->why &&
	    now - s->printed_ns[src] < NSEC_PER_SEC)
		return;
	if (s) {
		s->printed_ns[src] = now;
		s->printed_why[src] = t->why;
	}
	for (u32 i = 0; i < nv && i < G5P_MMIO_MAX_OPS; i++)
		len += scnprintf(vs + len, sizeof(vs) - len, "%s%08x",
				 i ? "," : "", v[i]);
	dev_info(
		g->dev,
		"GPU-MMIO sample why=%s did=%s r1_0=%08x gpc=%08x r1_0_end=%08x gpc_end=%08x "
		"max=%u v=%s us=%u end=mmio\n",
		g5p_mms_names[src], g5p_mmio_did(t), t->r1_0, t->gpc,
		t->r1_0_end, t->gpc_end, READ_ONCE(g->mmio_max_pstate), vs,
		g5p_mmio_us(t));
}

/* Rung 3/5: one licensed ticket, CORE_REVISION and CORE_ID. */

/* drm side: rung 5 at the 3D completion decision, and the idle clock for bit 3. */

/* Bit 4: the GPU core clock.  CR_TIMER (R0+0x160) ticks once per 256 core clocks whether or not a
 * phase is open (r3clk: 2823 kHz at pstate 4 = 722.7 MHz, 3703 kHz at 948 MHz), so CR_TIMER over
 * host time x 256 is the clock at any moment.  The Rogue PERF_TA_CYCLE / PERF_3D_CYCLE /
 * PERF_TA_OR_3D_CYCLE counters (R0+0x6020/0x6028/0x6038, 32 bit) advance at the core clock while a
 * TA or 3D phase is open (340.0 MHz through open TA phases, docs/gpu/experiments/46-device-tests.md):
 * over host time, the highest rate per pstate (a window busy all through) is the clock again.  One
 * sample = two licensed tickets, mmio_clock_gap_us apart, each reading the three counters and
 * CR_TIMER's low word back to back.  A cycle rate above 2 GHz is a firmware counter clear
 * (R0+0x6000 = 1, 0x1001ab00) inside the window and is dropped.
 */

/* Bit 3: does the GPC power down at full idle in state 0?  The PMGR word is always readable;
 * R1 only while the gate reads on (actual == target == 0xf), through a census ticket so the RAM
 * terms and the lock still apply.  A refused licence still returns the R1 words it read.
 */

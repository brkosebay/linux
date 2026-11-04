/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* R74f deferred doorbell. Included by boot.inc before the event decoder.
 * Diagnostic bypass on the r74 forced-ready epoch only; not power policy.
 *
 * A DC3 pair or a natural TA is published once, append-only, before this
 * code runs; its producer is never rewound, republished or torn down. If
 * the immediate functional gate finds GPC inactive, that generation stays
 * pending (never b->failed), the host repeats the r74e release transition
 * once, and the first type-0 ALIVE decoded afterwards owns one doorbell
 * attempt. The owner runs under event_lock: producer confirm, R1+14 fence,
 * active check, dma_wmb(), doorbell 0x10. Receipts stay in RAM; console
 * lines are printed afterwards from process context.
 */

static const char *const g5p_r74f_kinds[G5P_R74F_KINDS] = { "dc3", "ta", "3d" };

static const char *const g5p_r74f_states[G5P_R74F_STATES] = {
	"idle", "published", "open", "firing", "rang", "cancelled",
};

static const char *const g5p_r74f_cancels[G5P_R74F_CANCELS] = {
	"none",	      "reactivation-failed",
	"no-alive",   "consumed-before-bell",
	"protocol",   "alive-during-reactivation",
	"self-drain",
};

static const char *const g5p_r74f_stops[G5P_R74F_STOPS] = {
	"none",
	"attach-reactivation-failed",
	"attach-published-not-kicked",
	"attach-kicked-not-consumed",
	"first-mmu-window-closed",
	"ta-reactivation-failed",
	"ta-published-not-kicked",
	"firmware-dc3-tag-mismatch",
	"dc3-ram-validation-refused",
	"firmware-companion-recovery",
	"ta-published-not-consumed",
	"ta-window-missed",
};

static const char *const g5p_r74f_paths[G5P_R74F_PATHS] = {
	"none",
	"immediate",
	"deferred",
};

static bool g5p_r74f_on(struct g5p_boot *b)
{
	return b && b->r73b && 1;
}

static u32 *g5p_r74f_ctl(struct g5p_boot *b, u32 kind)
{
	return (u32 *)g5p_boot_shared(b, kind == G5P_R74F_TA ? 0x60 :
					 kind == G5P_R74F_3D ? 0x64 :
							       0x6c);
}

/* Owner only, event_lock held with interrupts off. Between the active check
 * and the doorbell there is only dma_wmb(): no console, no other MMIO.
 */
static int g5p_r74f_ring(struct g5p_diag *g, const struct g5p_r74f_bell *f,
			 u32 *fence, u32 *state)
{
	struct g5p_boot *b = g->boot;
	u32 *ctl = g5p_r74f_ctl(b, f->kind);
	u32 consumer = READ_ONCE(ctl[0]);
	u32 producer = smp_load_acquire(&ctl[4]);
	u32 queued = (f->target - f->start) & 255;
	u32 progressed = (consumer - f->start) & 255;
	u32 expected = f->kind == G5P_R74F_DC3 ? 2 : 1;

	if (READ_ONCE(b->failed))
		return -EIO;
	/* Do not turn corrupted modulo-256 ring state into a doorbell. */
	if (consumer > 255 || producer > 255 || queued != expected ||
	    progressed > queued)
		return -EPROTO;
	/* Acquire: pairs with the producer store published before the state. */
	if (producer != f->target)
		return -EPROTO;
	if (consumer == f->target)
		return -EALREADY;
	writel(7, g->window[1] + 0x14);
	*fence = readl(g->window[1] + 0x14);
	if (*fence & ~7U)
		return -EPROTO;
	*state = readl(g->window[1] + 0x1000);
	/* perf60 (gpc_any_pstate): a GPC request in flight or a not-yet-awake GPC after the wake
	 * above; re-read for up to 2 ms, as Apple's setGPCPerformanceState waits for bit 4.  Reads only.
	 */
	{
		u64 t0 = ktime_get_mono_fast_ns();

		while (!g5p_gpc_ok(*state) && ktime_get_mono_fast_ns() - t0 <
						      2000ULL * NSEC_PER_USEC) {
			udelay(10);
			*state = readl(g->window[1] + 0x1000);
		}
	}
	if (!g5p_gpc_ok(*state))
		return -EHOSTDOWN;
	dma_wmb();
	writel(f->kind == G5P_R74F_3D ? 0x11 : 0x10, g->rgx + 0xb00);
	return 0;
}

/* Owner only: record the attempt, then publish it with the state release. */
static void g5p_r74f_rang(struct g5p_r73b *r, struct g5p_r74f_bell *f, int ret,
			  u32 fence, u32 state, u32 path)
{
	f->bell_ns = ktime_get_mono_fast_ns();
	f->path = path;
	f->fence = fence;
	f->bell_state = state;
	f->bell_ret = ret;
	if (!ret) {
		f->kicked = 1;
		atomic_inc(&r->r74f_bells);
	} else if (ret == -EALREADY) {
		f->cancel = G5P_R74F_CANCEL_CONSUMED;
	} else if (ret != -EHOSTDOWN) {
		f->cancel = G5P_R74F_CANCEL_PROTOCOL;
	}
	atomic_set_release(&f->state, G5P_R74F_RANG);
}

/* g5p_boot_drain(), event_lock held: one type-0 ALIVE was just decoded. */
static void g5p_r74f_alive(struct g5p_diag *g)
{
	struct g5p_r73b *r = g->boot->r73b;
	struct g5p_r74f_bell *f;
	u32 kind, fence = U32_MAX, state = U32_MAX;
	int ret;

	WRITE_ONCE(r->r74f_alive, r->r74f_alive + 1);
	kind = READ_ONCE(r->r74f_active);
	if (!kind)
		return;
	f = &r->r74f_bell[kind - 1];
	if (atomic_read_acquire(&f->state) != G5P_R74F_OPEN ||
	    r->r74f_alive <= f->open_alive)
		return;
	/* Sole owner: exactly one decoder moves OPEN to FIRING. */
	if (atomic_cmpxchg(&f->state, G5P_R74F_OPEN, G5P_R74F_FIRING) !=
	    G5P_R74F_OPEN)
		return;
	f->bell_alive = r->r74f_alive;
	f->attempted = 1;
	f->bell_start_ns = ktime_get_mono_fast_ns();
	ret = g5p_r74f_ring(g, f, &fence, &state);
	g5p_r74f_rang(r, f, ret, fence, state, G5P_R74F_PATH_DEFERRED);
}

static int g5p_r74f_poll(struct g5p_diag *g, u32 alive, u32 expected,
			 u32 timeout_us, u64 *elapsed_ns, u32 *raw)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	unsigned long flags;
	u64 start = ktime_get_mono_fast_ns();
	int ret;

	*raw = U32_MAX;
	for (;;) {
		if (READ_ONCE(g->boot->failed)) {
			ret = -EIO;
			break;
		}
		if (ktime_get_mono_fast_ns() - start >=
		    (u64)timeout_us * 1000) {
			ret = -ETIMEDOUT;
			break;
		}
		/* Serialize the ordinal check and exactly one R1 read with the
		 * decoder. Once ALIVE advances, no further R1 MMIO is licensed.
		 */
		spin_lock_irqsave(&b->event_lock, flags);
		if (READ_ONCE(r->r74f_alive) != alive) {
			spin_unlock_irqrestore(&b->event_lock, flags);
			ret = -EHOSTDOWN;
			break;
		}
		*raw = readl(g->window[1] + 0x1000);
		spin_unlock_irqrestore(&b->event_lock, flags);
		if ((*raw & 0x1f) == expected) {
			ret = 0;
			break;
		}
		udelay(10);
	}
	*elapsed_ns = ktime_get_mono_fast_ns() - start;
	return ret;
}

/* Absolute freshness budget from the DC3 doorbell. The first MMU pass may
 * touch R0 only while the same active window is still demonstrably fresh.
 */

/* The r74e release transition, once per generation, on r74f receipts and
 * without console output: core request clear, GPC 0 within 1.0 ms, GPC 1
 * within 1.8 ms. Every R1 access is serialized against ALIVE decoding;
 * ordinal advance cancels the generation before any further R1 MMIO.
 */
static int g5p_r74f_reactivate(struct g5p_diag *g, struct g5p_r74f_bell *f)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	unsigned long flags;
	u32 alive = f->arm_alive;
	int ret;

	f->reactivated = 1;
	f->power_start_ns = ktime_get_mono_fast_ns();
	spin_lock_irqsave(&b->event_lock, flags);
	if (READ_ONCE(r->r74f_alive) != alive) {
		spin_unlock_irqrestore(&b->event_lock, flags);
		f->power_ret = -EHOSTDOWN;
		return -EHOSTDOWN;
	}
	writel(0, g->window[1] + 0x20000);
	f->core_fence = readl(g->window[1] + 0x20000);
	spin_unlock_irqrestore(&b->event_lock, flags);
	ret = g5p_r74f_poll(g, alive, 0, G5P_R74F_OFF_US, &f->off_ns,
			    &f->off_raw);
	if (!ret) {
		spin_lock_irqsave(&b->event_lock, flags);
		if (READ_ONCE(r->r74f_alive) != alive) {
			spin_unlock_irqrestore(&b->event_lock, flags);
			ret = -EHOSTDOWN;
			goto out;
		}
		/* boot_gpu_perf_seed: the firmware believes pstate q; never drop the GPC under it */
		writel(0x10 | (b->perf_poweron	    ?:
				       b->perf_seed ?:
						      1),
		       g->window[1] + 0x1000);
		f->gpc_fence = readl(g->window[1] + 0x1000);
		spin_unlock_irqrestore(&b->event_lock, flags);
		ret = g5p_r74f_poll(g, alive,
				    b->perf_poweron	 ?:
					    b->perf_seed ?:
							   1,
				    G5P_R74F_ON_US, &f->on_ns, &f->on_raw);
	}
out:
	f->power_ret = ret;
	return ret;
}

/* Process context, before any publication: no overlap with a pending ring. */
static int g5p_r74f_admit(struct g5p_diag *g, u32 kind)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	u32 *dc = g5p_r74f_ctl(b, G5P_R74F_DC3);

	if (!g5p_r74b_ram_only(g) || READ_ONCE(b->failed) ||
	    READ_ONCE(r->r74f_stop) ||
	    atomic_read(&r->r74f_bell[kind].state) != G5P_R74F_IDLE)
		return -EPERM;
	if (kind == G5P_R74F_DC3)
		return READ_ONCE(r->r74f_active) ? -EBUSY : 0;
	/* M7: the TA wakeup never shares a doorbell with a pending DC3 pair. */
	dma_rmb();
	if (READ_ONCE(r->r74f_active) != G5P_R74F_DC3 + 1 ||
	    !r->r74f_bell[G5P_R74F_DC3].complete ||
	    READ_ONCE(dc[0]) != READ_ONCE(dc[4]))
		return -EBUSY;
	return 0;
}

static void g5p_r74f_line(struct g5p_diag *g, const struct g5p_r74f_bell *f,
			  bool blob)
{
	struct g5p_boot *b = g->boot;
	u32 *ctl = g5p_r74f_ctl(b, f->kind);
	u32 state = atomic_read_acquire(&f->state);
	char line[3][512];

	dma_rmb();
	scnprintf(
		line[0], sizeof(line[0]),
		"R74F bell kind=%s generation=%u ring=%02x start=%u target=%u state=%s path=%s "
		"first_ret=%d first_fence=%08x first_state=%08x attempted=%u fence=%08x "
		"bell_state=%08x bell_ret=%d kicked=%u bells=%d cancel=%s complete=%u end=r74f\n",
		g5p_r74f_kinds[f->kind], f->generation, f->ring, f->start,
		f->target, g5p_r74f_states[state < G5P_R74F_STATES ? state : 0],
		g5p_r74f_paths[f->path], f->first_ret, f->first_fence,
		f->first_state, f->attempted, f->fence, f->bell_state,
		f->bell_ret, f->kicked, atomic_read(&b->r73b->r74f_bells),
		g5p_r74f_cancels[f->cancel], f->complete);
	scnprintf(
		line[1], sizeof(line[1]),
		"R74F ring kind=%s generation=%u arm_alive=%u open_alive=%u bell_alive=%u "
		"alive_now=%u consumer_before=%u consumer_after=%u producer_after=%u "
		"consumer_now=%u producer_now=%u failed=%u event_failed=%u end=r74f\n",
		g5p_r74f_kinds[f->kind], f->generation, f->arm_alive,
		f->open_alive, f->bell_alive, READ_ONCE(b->r73b->r74f_alive),
		f->consumer_before, f->consumer_after, f->producer_after,
		READ_ONCE(ctl[0]), READ_ONCE(ctl[4]), READ_ONCE(b->failed),
		READ_ONCE(b->event_failed));
	scnprintf(
		line[2], sizeof(line[2]),
		"R74F power kind=%s generation=%u reactivated=%u core_fence=%08x gpc_fence=%08x "
		"off_ns=%llu on_ns=%llu off_raw=%08x on_raw=%08x power_ret=%d arm_ns=%llu "
		"power_start_ns=%llu open_ns=%llu bell_start_ns=%llu bell_ns=%llu done_ns=%llu "
		"end=r74f\n",
		g5p_r74f_kinds[f->kind], f->generation, f->reactivated,
		f->core_fence, f->gpc_fence, f->off_ns, f->on_ns, f->off_raw,
		f->on_raw, f->power_ret, f->arm_ns, f->power_start_ns,
		f->open_ns, f->bell_start_ns, f->bell_ns, f->done_ns);
	for (u32 i = 0; i < ARRAY_SIZE(line); i++) {
		if (blob)
			g5p_blob_printf(g, "%s", line[i]);
		else
			dev_emerg(g->dev, "%s", line[i]);
	}
}

/* r87b receipt: the DC3 attach policy and what the host did after the publish.
 * RAM reads only; one line per DC3 generation (console at the consume/timeout
 * point, blob at export).
 */
static void g5p_r87b_line(struct g5p_diag *g, const struct g5p_r74f_bell *f,
			  bool blob)
{
	struct g5p_boot *b = g->boot;
	u32 policy = READ_ONCE(b->r73b->r87b_attach);
	char line[640];

	dma_rmb();
	scnprintf(
		line, sizeof(line),
		"R87B attach policy=%u name=%s generation=%u gate_first_ret=%d "
		"gate_first_state=%08x gate_closed=%u path=%s cancel=%s bell_rung=%u bells=%d "
		"reactivated=%u cons_before=%u prod_before=%u prod_pub=%u cons_after=%u "
		"prod_after=%u consumed=%u arm_alive=%u alive_done=%u alive_delta=%u "
		"pub_to_done_ns=%llu irqs=%d events=%u end=r87b\n",
		policy, policy ? "self-drain" : "original", f->generation,
		f->first_ret, f->first_state, f->first_ret == -EHOSTDOWN,
		g5p_r74f_paths[f->path], g5p_r74f_cancels[f->cancel], f->kicked,
		atomic_read(&b->r73b->r74f_bells), f->reactivated,
		f->consumer_before, f->start, f->target, f->consumer_after,
		f->producer_after, f->complete, f->arm_alive, f->alive_done,
		f->alive_done - f->arm_alive,
		f->done_ns >= f->arm_ns ? f->done_ns - f->arm_ns : 0,
		atomic_read(&b->irqs), READ_ONCE(b->events));
	if (blob)
		g5p_blob_printf(g, "%s", line);
	else
		dev_emerg(g->dev, "%s", line);
}

/* A retained boundary: the publication stays, nothing is failed. */
static int g5p_r74f_stop(struct g5p_diag *g, u32 stop, int value)
{
	struct g5p_r73b *r = g->boot->r73b;

	r->r74f_stop_value = value;
	WRITE_ONCE(r->r74f_stop, stop);
	g5p_r74_mark(g, G5P_R74F_STOP, stop);
	dev_emerg(g->dev, "R74F stop reason=%s value=%d failed=%u end=r74f\n",
		  g5p_r74f_stops[stop], value, READ_ONCE(g->boot->failed));
	return -EINPROGRESS;
}

/* After the reactivation: open the generation, wait (bounded) for the
 * ALIVE owner while the event drain stays alive, then cancel if unrung.
 */
static int g5p_r74f_open(struct g5p_diag *g, struct g5p_r74f_bell *f)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	unsigned long flags, deadline;
	int ret;

	ret = g5p_r74f_reactivate(g, f);
	spin_lock_irqsave(&b->event_lock, flags);
	if (ret) {
		f->cancel = ret == -EHOSTDOWN ? G5P_R74F_CANCEL_ALIVE_ADVANCED :
						G5P_R74F_CANCEL_REACTIVATION;
		atomic_set_release(&f->state, G5P_R74F_CANCELLED);
	} else {
		f->open_alive = r->r74f_alive;
		f->open_ns = ktime_get_mono_fast_ns();
		/* The next decoded type-0 ALIVE may now take ownership. */
		atomic_set_release(&f->state, G5P_R74F_OPEN);
	}
	spin_unlock_irqrestore(&b->event_lock, flags);
	if (ret)
		return 0;
	deadline = jiffies + msecs_to_jiffies(G5P_R74F_BELL_MS);
	while (atomic_read_acquire(&f->state) == G5P_R74F_OPEN &&
	       time_before(jiffies, deadline)) {
		/* The IRQ path is the sole event consumer and bell owner. */
		if (READ_ONCE(b->failed)) {
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
	return ret;
}

/* Process context, command mutex held, producer already published. Returns
 * 0 after one doorbell, 1 if consumed without one, -EINPROGRESS when the
 * publication is retained unrung, else a fatal protocol error.
 */
static int g5p_r74f_bell(struct g5p_diag *g, u32 kind, u32 start, u32 target)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	struct g5p_r74f_bell *f = &r->r74f_bell[kind];
	u32 *ctl = g5p_r74f_ctl(b, kind);
	u32 fence = U32_MAX, state = U32_MAX, now;
	unsigned long flags;
	int ret, drained = 0;
	bool ta = kind == G5P_R74F_TA;
	bool self_drain;

	if (kind == G5P_R74F_3D)
		return -EINVAL; /* the 3D bell is armed by g5p_r96_3d_arm(): never the DC3 / self-drain path */
	f->kind = kind;
	f->ring = ta ? 0x60 : 0x6c;
	f->start = start;
	f->target = target;
	f->generation = ++r->r74f_generation;
	f->consumer_before = READ_ONCE(ctl[0]);
	f->arm_ns = ktime_get_mono_fast_ns();
	/* The immediate functional gate r74e used, without console output. */
	spin_lock_irqsave(&b->event_lock, flags);
	f->arm_alive = r->r74f_alive;
	WRITE_ONCE(r->r74f_active, kind + 1);
	atomic_set(&f->state, G5P_R74F_FIRING);
	f->attempted = 1;
	f->bell_start_ns = ktime_get_mono_fast_ns();
	ret = g5p_r74f_ring(g, f, &fence, &state);
	f->first_ret = ret;
	f->first_fence = fence;
	f->first_state = state;
	/* r87b self-drain: a closed DC3 gate is only a phase sample of the
	 * firmware's GPC window. The host never repairs it: no R1+20000 clear, no
	 * GPC off/on, no bell. The caller waits RAM-only for the firmware to
	 * consume the pair. Only the DC3 kind; an active gate rang above.
	 */
	self_drain = ret == -EHOSTDOWN && kind == G5P_R74F_DC3 &&
		     READ_ONCE(r->r87b_attach) == G5P_R87B_ATTACH_SELF_DRAIN;
	if (self_drain) {
		f->cancel = G5P_R74F_CANCEL_SELF_DRAIN;
		atomic_set_release(&f->state, G5P_R74F_CANCELLED);
	} else if (ret == -EHOSTDOWN) {
		atomic_set_release(&f->state, G5P_R74F_PUBLISHED);
	} else {
		g5p_r74f_rang(r, f, ret, fence, state, G5P_R74F_PATH_IMMEDIATE);
	}
	spin_unlock_irqrestore(&b->event_lock, flags);
	if (self_drain) {
		drained = 1;
		g5p_r74_mark(g, G5P_R74F_ATTACH_ARMED, state);
	} else if (ret == -EHOSTDOWN) {
		drained = g5p_r74f_open(g, f);
		/* Functional reactivation precedes all console/milestone output. */
		g5p_r74_mark(g, ta ? G5P_R74F_TA_ARMED : G5P_R74F_ATTACH_ARMED,
			     state);
		g5p_r74_mark(g,
			     ta ? G5P_R74F_TA_REACTIVATED :
				  G5P_R74F_ATTACH_REACTIVATED,
			     f->power_ret);
	}
	now = atomic_read_acquire(&f->state);
	g5p_r74_mark(g, ta ? G5P_R74F_TA_BELL : G5P_R74F_ATTACH_BELL,
		     now == G5P_R74F_RANG ? f->bell_ret : -ECANCELED);
	g5p_r74f_line(g, f, false);
	if (drained)
		return drained;
	if (now == G5P_R74F_CANCELLED) {
		if (f->cancel == G5P_R74F_CANCEL_REACTIVATION ||
		    f->cancel == G5P_R74F_CANCEL_ALIVE_ADVANCED)
			return g5p_r74f_stop(
				g,
				ta ? G5P_R74F_STOP_TA_REACTIVATION :
				     G5P_R74F_STOP_ATTACH_REACTIVATION,
				f->power_ret);
		return g5p_r74f_stop(g,
				     ta ? G5P_R74F_STOP_TA_NOT_KICKED :
					  G5P_R74F_STOP_ATTACH_NOT_KICKED,
				     -ETIMEDOUT);
	}
	if (f->kicked)
		return 0;
	if (f->bell_ret == -EALREADY)
		return 1;
	if (f->bell_ret == -EHOSTDOWN)
		return g5p_r74f_stop(g,
				     ta ? G5P_R74F_STOP_TA_NOT_KICKED :
					  G5P_R74F_STOP_ATTACH_NOT_KICKED,
				     -EHOSTDOWN);
	return f->bell_ret;
}

/* RAM only: the consumer this generation reached, at consume or timeout. */
static void g5p_r74f_consumed(struct g5p_diag *g, u32 kind)
{
	struct g5p_r74f_bell *f = &g->boot->r73b->r74f_bell[kind];
	u32 *ctl = g5p_r74f_ctl(g->boot, kind);

	dma_rmb();
	f->consumer_after = READ_ONCE(ctl[0]);
	f->producer_after = READ_ONCE(ctl[4]);
	f->complete = f->consumer_after == f->target;
	f->done_ns = ktime_get_mono_fast_ns();
	f->alive_done = READ_ONCE(g->boot->r73b->r74f_alive);
	if (kind == G5P_R74F_DC3)
		g5p_r87b_line(g, f, false);
}

/* g5p_r22_mmu(): attach's first pass only; records the gate it already read. */

/* g5p_r22_command(): an r74f boundary keeps the epoch, drain and backing. */

/* r73b-export, process context, RAM only. */

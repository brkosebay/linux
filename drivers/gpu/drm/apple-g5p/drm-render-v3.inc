/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * TYPED_V3 serial executor: the control flow that completed the natural TA and the
 * natural 3D in r96 (docs/gpu/experiments/production-typed-render-kernel.md), on typed
 * state.  Included from drm-render.inc after g5p_render_execute().
 *
 * What it does, in order, for each job a V3 context runs (strictly serial across every context,
 * render-user-serial.h; only the epoch's job 1 attaches and has hardware evidence):
 *   0. bind, the first job of every later context (multi-client-design.md §4.2): its own DC3
 *      pair, published in RAM only, no doorbell, a RAM-only wait for the firmware to consume
 *      it.  With the TEMPORARY render_user_spare_pairs instead: a pair job 1 pre-bound, taken
 *      over through the tag path.
 *   1. attach (job 1): the typed preparation (PM, channels, catalogue entries), then the
 *      DC3 pair is published and its doorbell is owned by the r74f machinery (r87b
 *      self-drain: a closed gate is never repaired by the host).  The firmware installs
 *      the page-table root and the generation tags itself (r74g); the host does NO MMU pass
 *      and checks the result in RAM only.
 *   2. TA: record, WORK_CTL and producer, no doorbell; the r74g grace (four ALIVEs or
 *      20 ms) lets the firmware drain it, and only if it is still pending afterwards the
 *      first ALIVE decoded owns one fenced fallback doorbell.
 *   3. TA wait: RAM only (drain, sample, sleep), 20 s per phase by default, 30 s for the
 *      whole job (render-user-wait.h decides the deadline and closes the race with a
 *      completion).  tahang1: complete on a coherent target with the PM saved and either
 *      the idle words clear, the TA completion event, or a 500 ms coherent hold
 *      (render-user-retire.h); a phase pending 1 s prints one RAM-only `slow` line.  From
 *      the TA complete on, and after any timeout or fault, the host touches no register.
 *   4. 3D: record, WORK_CTL and producer, NO doorbell (r96 stage 1: the firmware consumed
 *      it 2 ms later), RAM-only wait as above.
 *   RC4a (render_user_touch_ms, default 0 = off): at publish + T of each phase, odd jobs read
 *   the receipt pages once and even jobs only record the instant (render-user-touch.h); RAM only.
 *   repeat109: the 3D phase is also complete when its target state is coherent and its completion
 *   event arrived, even if the engine idle words never clear (render-user-retire.h; the latched
 *   render_user_idle_settle_ms, default 0, adds a minimum coherent run); one `RENDER-V3 retire`
 *   receipt line splits the firmware's retirement from the host's decision.  RAM only.
 *   wake110 (render_user_wake, default 0 = off): right after the producer store of every phase but
 *   job 1's TA, the A/B's factor phases run Apple's submit kick (R1+0x14 = 7, GPC active, R0+0xb00 =
 *   0x10 / 0x11) and its control phases the same guards without a register (render-user-wake.h);
 *   one `RENDER-V3 wake` line per job.  The only register access after the TA complete, licensed
 *   while no timeout or fault was noted.
 * Jobs >= 2 (a compositor's next frame, or another context's): no attach, the PM pool is
 * rebuilt, and both rings are published with no doorbell at all.
 * Not in this path on purpose: the r74h / r80 / r85-r96 root-cause instrumentation (stream
 * markers, PM canary, plateau and end banks, region/target dumps), host MMU passes, cache
 * maintenance MMIO, selector reads, snapshots (they read R0), and any recovery.  A timeout
 * or fault retains everything; the context is frozen and kept until reboot.
 *
 * Terminal state (render-user-terminal.h): the TA complete, a timeout and a fault go through one
 * first-writer-wins latch, g5p_rv3_terminal().  From then on g5p_ru_mmio_ok() is false, the admission
 * refuses every job that would start behind a failure (-EIO), and the legacy typed paths that read or
 * write registers (snapshot, invalidate, run, destroy) refuse.  g5p_rv3_td_hook() is the optional
 * recovery bridge (terminal note and the observation-only CAPTURE entry); it is nothing without it.
 */

static const char *const g5p_rv3_phase_names[G5P_RV3_PHASES] = {
	"idle",	   "attach",	 "bind",    "ta-publish",
	"ta-wait", "3d-publish", "3d-wait", "done",
};
static const char *const g5p_rv3_results[] = { "pending", "complete", "timeout",
					       "fault" };
static const char *const g5p_rv3_how[] = { "none", "idle-clear", "idle-stuck",
					   "hold" };

#include "drm-latency123.inc"

/* perf60 stage 4 (render-user-pmset.h): per-context PM records, the iOS model (one ParamBuffer
 * per work queue, 6cb9afc/6cb2378).  Latched at the epoch's first attach, with boot_pm_sets:
 *   1  every context bound after the epoch's first gets its own PM set (control + S, list, F in
 *      the firmware GiB), its own pool pages and its own aux pages at G5P_RU_PRIVATE_VA in its
 *      VM; the epoch's first context keeps the device PM.  PM ID 0 everywhere.
 *   2  as 1, and set s carries PM ID s + 1 (TA W+0x10, TA start, 3D start, tag6).
 * RAM only, by the executor between jobs.
 */
/* release8 default */

/* The PM a context's jobs use: its set, or the device PM. */
static u8 *g5p_rv3_pm_cpu(struct g5p_boot *b, const struct g5p_render *r)
{
	if (r->pm_set >= 0)
		return b->pm_set[r->pm_set * G5P_PMSET_PAGES + G5P_PMSET_CONTROL]
			.cpu;
	return g5p_ru_pm_cpu(b);
}

static u32 g5p_rv3_pm_meta(const struct g5p_render *r)
{
	return g5p_meta_address(
		r->pm_set >= 0 ? g5p_pmset_va(r->pm_set, G5P_PMSET_CONTROL) :
				 G5P_SHARED_VA + G5P_R22_PM);
}

static u32 g5p_rv3_pm_id(const struct g5p_rdev *rd, const struct g5p_render *r)
{
	return rd->pm_ctx >= 2 && r->pm_set >= 0 ? g5p_pmset_id(r->pm_set) : 0;
}

static struct g5p_dma_page *g5p_rv3_pm_aux(struct g5p_rdev *rd,
					   struct g5p_render *r)
{
	return r->pm_aux_mapped ? r->pm_aux : rd->aux;
}

/* The boot tuple the V3 path is proven under (r96 params.txt), minus the instrumentation:
 * r73b/r73c/r73e/r73f, r74 with the raised IRQ cap, r74f, r74g, r87b self-drain; no r74h,
 * no r80..r96 (their IRQ banks read registers), no r75/r79 firmware control, and no fixed
 * render engine (it is exclusive with the DRM one).
 */
static bool g5p_rv3_tuple(struct g5p_diag *g)
{
	struct g5p_boot *b = g ? g->boot : NULL;
	struct g5p_r73b *x = b ? b->r73b : NULL;

	return x && g5p_r74g_on(b) && g5p_r74b_ram_only(g) &&
	       x->r87b_attach == G5P_R87B_ATTACH_SELF_DRAIN;
}

/* The admission state of a context and the device, as plain values for g5p_ru_v3_admit(). */
static struct g5p_ru_v3_state g5p_rv3_state(struct g5p_diag *g,
					    struct g5p_render *r)
{
	const struct g5p_rdev *rd = r->ctx->gpu->rdev;

	return (struct g5p_ru_v3_state){
		.tuple = g5p_rv3_tuple(g),
		.v3_accepted = r->v3_accepted,
		.bound = r->bound,
		.jobs = r->jobs,
		.device_up = READ_ONCE(rd->up),
		.device_v3 = READ_ONCE(rd->v3),
		.sequence = READ_ONCE(rd->sequence),
		.slot_ta = r->slot[0],
		.slot_3d = r->slot[1],
		.variant = 0 || r->after_ta || 0 || r->v.pm || r->v.memory ||
			   r->v.rt || r->v.slots,
		.wait_ms = r->wait_ms,
		.quiet = !g5p_ru_mmio_ok(&g5p_rv3_term),
		.terminal_failed = g5p_ru_term_failed(&g5p_rv3_term),
	};
}

/* A terminal epoch is a device state (-EIO), the others are about this request. */
static int g5p_rv3_refusal_errno(int why)
{
	if (why == G5P_RU_V3_TERMINAL)
		return -EIO;
	return why == G5P_RU_V3_OVERFLOW ? -EOVERFLOW : -EOPNOTSUPP;
}

/* Submit-time admission: every V3 job refused here has touched nothing. */
static int g5p_rv3_admit_submit(struct g5p_drm *d, struct g5p_render *r)
{
	struct g5p_diag *g = d->core;
	struct g5p_ru_v3_state st;
	int why;

	/* The epoch is being replaced: the executor admits the job again under the lock. */
	if (READ_ONCE(d->rdev->recovering))
		return 0;
	st = g5p_rv3_state(g, r);
	why = g5p_ru_v3_admit(&st, false);

	if (!why)
		return 0;
	/* per submit while admission is closed and a client retries: ratelimited */
	dev_info_ratelimited(
		g->dev, "RENDER-V3 refused: %s; no hardware touched end=v3\n",
		g5p_ru_v3_why[why]);
	return g5p_rv3_refusal_errno(why);
}

/* Recovery hooks (patches/gpu-recovery), compiled only when that series is applied: core.inc then includes
 * teardown.inc before this file, and teardown.h defines G5P_TEARDOWN_H.  The CAPTURE entry is observation only
 * (RAM reads, no register access) and runs only with the recovery module parameter set, once per epoch, on a
 * timeout or fault.  Without the series both are nothing, and the prod98 kernel does not depend on it.
 */
#ifdef G5P_TEARDOWN_H
static void g5p_rv3_td_hook(struct g5p_diag *g, int kind, const char *why)
{
	g5p_td_note_terminal(&g5p_td_state, g5p_ru_td_kind(kind));
	if ((0) && g5p_ru_term_capture_due(&g5p_rv3_term, kind))
		g5p_td_capture(g, why);
}
#else
static inline void g5p_rv3_td_hook(struct g5p_diag *g, int kind,
				   const char *why)
{
}
#endif

/* The terminal state of the epoch: the TA complete, and any timeout or fault.  First writer wins; from the first
 * note on g5p_ru_mmio_ok() is false and the host touches no register (R0, R1, AIC, doorbell) and no bank.  The r80
 * flag is set when an r80 block exists, so that a bank that might be latched cannot run behind this path (the V3
 * tuple forbids r80, so there is none today); the latch above does not depend on it.
 */
static void g5p_rv3_terminal(struct g5p_diag *g, int kind, const char *why)
{
	bool failed = g5p_ru_term_failed(&g5p_rv3_term);

	if (g5p_ru_term_note(&g5p_rv3_term, kind))
		dev_emerg(
			g->dev,
			"RENDER-V3 terminal kind=%s why=%s mmio=off r0=off export=ram-only end=v3\n",
			g5p_ru_term_names[g5p_ru_term_kind(&g5p_rv3_term)],
			why);
	else if (!failed && g5p_ru_term_failed(&g5p_rv3_term))
		/* The epoch's TA complete took the latch long ago (job 1): a later timeout or fault
		 * only makes it sticky-failed, which closes admission for every client.  Say so once.
		 */
		dev_emerg(
			g->dev,
			"RENDER-V3 terminal kind=%s why=%s latched=%s failed=1 admission=closed end=v3\n",
			g5p_ru_term_names
				[kind > G5P_RU_TERM_NONE &&
						 kind < G5P_RU_TERM_KINDS ?
					 kind :
					 G5P_RU_TERM_FAULT],
			why,
			g5p_ru_term_names[g5p_ru_term_kind(&g5p_rv3_term)]);
	g5p_rv3_td_hook(g, kind, why);
}

/* "Retained boundary" of the r74f machinery -> the errno a job reports.  -EINPROGRESS is
 * never a job error (the UAPI uses it for "pending").
 */
static int g5p_rv3_errno(struct g5p_diag *g, int ret)
{
	if (ret != -EINPROGRESS)
		return ret;
	switch (READ_ONCE(g->boot->r73b->r74f_stop)) {
	case G5P_R74F_STOP_ATTACH_NOT_KICKED:
	case G5P_R74F_STOP_ATTACH_NOT_CONSUMED:
	case G5P_R74F_STOP_TA_NOT_KICKED:
	case G5P_R74G_STOP_TA_NOT_CONSUMED:
	case G5P_R74G_STOP_TA_WINDOW_MISSED:
		return -ETIMEDOUT;
	default:
		return -EIO;
	}
}

/*
 * Interim: on iOS the firmware writes back its own lines with the cache maintenance it runs on
 * every tick (meta+0x6d4); that needs init words still being recovered (latency123.md), and
 * this read stands in for it until then.
 *
 * While a phase is in flight the firmware does not see its own progress in the root and
 * shared pages until a coherent read of those lines reaches the fabric: without one, the TA
 * and 3D stages wait for the firmware tick, for seconds (latency123: key to frame 7.9 s
 * median; with a Normal-NC read of both regions every 1-2 ms, 0.27 s).  One 32-bit load per
 * 64-byte line, before every RAM sample of an in-flight phase.  RAM only, and an idle GPU
 * has no sampler, so it costs nothing then.
 */
static void g5p_rv3_fw_touch(struct g5p_boot *b)
{
	struct g5p_dma_page *const regions[] = { b->root, b->shared };
	u32 counts[] = { ARRAY_SIZE(b->root), ARRAY_SIZE(b->shared) };

	for (u32 r = 0; r < ARRAY_SIZE(regions); r++)
		for (u32 i = 0; i < counts[r]; i++)
			for (u32 at = 0; at < G5P_GPU_PAGE; at += 64)
				(void)READ_ONCE(
					*(const u32 *)(regions[r][i].cpu + at));
	/* perf60 stage 4: the per-context PM sets are firmware pages like the shared ones */
	for (u32 i = 0; i < b->pm_sets * G5P_PMSET_PAGES; i++)
		for (u32 at = 0; at < G5P_GPU_PAGE; at += 64)
			(void)READ_ONCE(*(const u32 *)(b->pm_set[i].cpu + at));
}

/* perf60 bell gates, A/B knobs for the multi-context 3D fault on the bell path (render_user_wake=2 only;
 * 0600, read at every bell, all 0 = no gate).  They run between a phase's producer store and its bell;
 * in policy state 0 the firmware does not drain a ring on its own, so a gate holds the phase back.
 * RAM reads, sleeps and barriers only; no register access.
 *   bell_gap_us      (a) the TA bell waits until this long after the previous job (any context) ended
 *   bell_idle_ms     (a) the TA bell first waits, up to this, for the 3D engine idle words to read zero
 *   bell_3d_delay_us (c) mb() and this delay between the 3D producer store and the 3D bell
 *   bell_ctx_gap_us  (d) mb() and this delay before the TA bell of a job of another context than the last
 *   bell_touch       (c) the Normal-NC read of the firmware pages (g5p_rv3_fw_touch) right before each bell
 */

/* perf60 job-start gate: the same waits before the job's own host writes (the PM pool rebuild and
 * the shared TA/3D work records, g5p_rv3_pm_rebuild() and g5p_ru_job()), which happen before any
 * publish.  The bell gates above run after them, so they cannot separate "the host rewrites shared
 * firmware pages while the firmware still finishes the previous job" from a doorbell race.
 *   job_gap_us   the job's first host write no sooner than this after the previous job ended
 *   job_idle_ms  then wait up to this (RAM) for the TA and 3D engine idle words to read zero
 */
/* perf60 layout (latched at the epoch's first attach, default 0 = the r22 layout):
 *   render_user_job_slots 2: job seq writes its command, W record and microsequence into slot
 *       (seq - 1) % 2 of the channel page, so a job never rewrites the record the firmware used for
 *       the job before it; the pointer ring carries each job's addresses (render-user-layout.h).
 *   render_user_pm_once 1: the PM pool, list and F are built by the epoch's first attach only and
 *       never rewritten by the host between jobs; the TA's tag6 record has the firmware rebuild F,
 *       as on iOS (PM created per queue once; docs/gpu/experiments/freelist-lifetime-production-fix.md).
 */
/* release8 default */
/* release8 default */

/* The record slot of job seq on this epoch's layout. */
static u32 g5p_rv3_js(const struct g5p_rdev *rd, u32 seq)
{
	return rd->job_slots == 2 ? (seq - 1) & 1 : 0;
}









/* RAM only. */
static void g5p_rv3_sample(struct g5p_diag *g, struct g5p_render *r, u32 e,
			   u32 seq, struct g5p_rv3_ev *s)
{
	struct g5p_boot *b = g->boot;
	u8 *p = g5p_ru_page(b, e), *global = g5p_ru_global(b, e),
	   *pm = g5p_rv3_pm_cpu(b, r);
	u32 *ring = (u32 *)g5p_boot_shared(b, e ? 0x64 : 0x60);

	g5p_rv3_fw_touch(b);
	memset(s, 0, sizeof(*s));
	dma_rmb();
	s->ns = ktime_get_ns();
	s->prod = READ_ONCE(ring[4]);
	s->cons = READ_ONCE(ring[0]);
	for (u32 i = 0; i < ARRAY_SIZE(s->stamp); i++)
		s->stamp[i] = g5p_le32(p + G5P_R22_STAMP + 4 * i);
	for (u32 i = 0; i < ARRAY_SIZE(s->ctl); i++)
		s->ctl[i] = g5p_le32(p + G5P_WORK_CTL + i * 0x10);
	for (u32 i = 0; i < ARRAY_SIZE(s->idle); i++)
		s->idle[i] = g5p_le32(global + 0x724 + i * 4);
	s->pm80 = g5p_le32(pm + 0x80);
	s->failed = READ_ONCE(b->failed) || READ_ONCE(b->event_failed);
	s->proto = s->prod != r->target[e] || s->cons > 255 ||
		   !g5p_ru_stamp_sane(p, seq);
	s->consumed = s->cons == r->target[e];
	/* The predicates of the flown r74b (TA) and r96 (3D) waits; the 3D's PM serial is a receipt. */
	s->idle_ok = !s->idle[0] && !s->idle[1] && !s->idle[2] &&
		     (!e || (!s->idle[4] && !s->idle[5]));
	s->retired = g5p_ru_retired_at(p, g5p_ru_meta(e), e, seq,
				       g5p_rv3_js(r->ctx->gpu->rdev, seq));
	s->pm_ok = e ? s->pm80 == r->ctx->gpu->rdev->pm_serial + 1 :
		       g5p_r22_pm_saved(pm + 0x80);
	s->event = !!(e ? READ_ONCE(r->ctx->gpu->td_hit_ns) :
			  READ_ONCE(r->ctx->gpu->ta_hit_ns));
}

/* TA: one step of render-user-retire.h's TA rules when ta is given, else the flown r74b
 * predicate.  3D: one step of its 3D rules when rt is given, else the r96 predicate.
 */
static bool g5p_rv3_done(u32 e, const struct g5p_rv3_ev *s,
			 struct g5p_ru_retire *rt, struct g5p_ru_ta *ta)
{
	if (e && rt)
		return g5p_ru_rt_step(rt, s->ns, s->consumed, s->retired,
				      s->idle_ok,
				      s->event) != G5P_RU_RT_PENDING;
	if (!e && ta)
		return g5p_ru_ta_step(ta, s->ns, s->consumed, s->retired,
				      s->pm_ok, s->idle_ok,
				      s->event) != G5P_RU_TA_PENDING;
	return s->consumed && s->idle_ok && s->retired && (e || s->pm_ok);
}

/* The TA clauses in the bits of render-user-retire.h, for the receipt. */
static u32 g5p_rv3_ta_unsat(const struct g5p_rv3_ev *s)
{
	return (s->consumed ? 0 : G5P_RU_RT_CONSUMED) |
	       (s->retired ? 0 : G5P_RU_RT_RETIRED) |
	       (s->idle_ok ? 0 : G5P_RU_RT_IDLE) |
	       (s->pm_ok ? 0 : G5P_RU_RT_PM);
}

/* Arm engine e's completion-event latch of g5p_drm_event() (3D: for this job's endpoint), before
 * the phase's producer store, so an event decoded right after the publish is not missed.  The
 * latch is set under status_lock by whichever CPU decodes the event, and the wait samples it
 * level-triggered every 1-2 ms, so there is no wakeup to lose.  RAM only.
 */
static void g5p_rv3_event_arm(struct g5p_drm *d, u32 e, u32 endpoint)
{
	unsigned long flags;

	BUILD_BUG_ON(G5P_RU_RT_GRID != G5P_WORK_SLOTS);
	spin_lock_irqsave(&d->status_lock, flags);
	if (e) {
		d->td_want = G5P_RV3_TD_WANT | endpoint;
		WRITE_ONCE(d->td_hit_ns, 0);
	} else {
		d->ta_want = true;
		WRITE_ONCE(d->ta_hit_ns, 0);
	}
	spin_unlock_irqrestore(&d->status_lock, flags);
}

/* Disarm engine e's latch and return the decode time of the matching event (0 = none). */
static u64 g5p_rv3_event_disarm(struct g5p_drm *d, u32 e)
{
	unsigned long flags;
	u64 hit;

	spin_lock_irqsave(&d->status_lock, flags);
	if (e) {
		d->td_want = 0;
		hit = d->td_hit_ns;
	} else {
		d->ta_want = false;
		hit = d->ta_hit_ns;
	}
	spin_unlock_irqrestore(&d->status_lock, flags);
	return hit;
}

/* RC4a: the receipt's read set (g5p_rv3_receipt()'s blobs: PM-control, then W, channels and the
 * stamp window of the TA and of the 3D page), read once with 32-bit loads, the access size of the
 * receipt's hex dump.  The words are folded, not copied: no allocation, no store to shared memory.
 * RAM only: no register, no doorbell, no print, no latch.
 */
static u32 g5p_rv3_touch_sweep(struct g5p_boot *b, u32 *bytes)
{
	u8 *ta = g5p_ru_page(b, 0), *td = g5p_ru_page(b, 1);
	const u8 *const src[] = {
		g5p_ru_pm_cpu(b), ta + G5P_R22_W, ta,
		ta + G5P_R22_TQ,  td + G5P_R22_W, td,
		td + G5P_R22_TQ,
	};
	static const u32 len[] = {
		0x140, G5P_RU_TA_W_BYTES, 0x340, 0x80, G5P_RU_3D_W_BYTES, 0x340,
		0x80,
	};
	u32 sum = 0, n = 0;

	dma_rmb();
	for (u32 i = 0; i < ARRAY_SIZE(src); i++) {
		for (u32 at = 0; at < len[i]; at += 4)
			sum = (sum << 5 | sum >> 27) ^
			      READ_ONCE(*(const u32 *)(src[i] + at));
		n += len[i];
	}
	*bytes = n;
	return sum;
}

/* RC4a: the instant of one phase, taken at the sample at now_ns.  The touch arm reads, the control
 * arm only records; both record the same instant.
 */
static void g5p_rv3_touch(struct g5p_diag *g, struct g5p_ru_touch *t,
			  u64 now_ns)
{
	u32 bytes = 0, sum = 0;

	if (t->sweep)
		sum = g5p_rv3_touch_sweep(g->boot, &bytes);
	g5p_ru_touch_mark(t, now_ns, t->sweep ? ktime_get_ns() : now_ns, bytes,
			  sum);
}

/* tahang1: a phase still pending this long after its publish prints its RAM state once. */
#define G5P_RV3_SLOW_MS 1000U

/* The one `slow` line of a phase: the deciding sample's RAM words and the waiter's own state, so
 * a stall that later times out is described from the first second on.  RAM only.
 */
static void g5p_rv3_slow(struct g5p_diag *g, u32 e, u32 seq,
			 const struct g5p_ru_wait *w,
			 const struct g5p_rv3_ev *s, u32 unsat, u64 hit_ns)
{
	dev_warn(
		g->dev,
		"RENDER-V3 slow seq=%u engine=%u ms=%lld ring=%u/%u stamp=%08x/%08x "
		"ctl=%08x,%08x,%08x,%08x,%08x idle=%08x,%08x,%08x,%08x,%08x,%08x pm80=%08x "
		"consumed=%u retired=%u pm=%u event_ms=%lld unsat=%#x busy_ms=%lld events=%u "
		"end=v3\n",
		seq, e, g5p_ru_rt_ms(w->start_ns, s->ns), s->prod, s->cons,
		s->stamp[0], s->stamp[1], s->ctl[0], s->ctl[1], s->ctl[2],
		s->ctl[3], s->ctl[4], s->idle[0], s->idle[1], s->idle[2],
		s->idle[3], s->idle[4], s->idle[5], s->pm80, s->consumed,
		s->retired, s->pm_ok, g5p_ru_rt_ms(w->start_ns, hit_ns), unsat,
		g5p_ru_rt_ms(w->start_ns, w->busy_ns),
		READ_ONCE(g->boot->events));
	g5p_np_policy(g, "slow", seq, e);
}

/* nopatch: the policy record at a few phase ends (log-spaced) and at every timeout.  RAM only. */
static void g5p_rv3_np_end(struct g5p_diag *g, u32 e, u32 seq, u32 result)
{
	if (seq == 1 || seq == 10 || seq == 100 || seq == 1000 ||
	    seq == 10000 || result == G5P_RU_WAIT_TIMEOUT)
		g5p_np_policy(g,
			      result == G5P_RU_WAIT_TIMEOUT ? "phase-timeout" :
							      "phase-end",
			      seq, e);
}

/* One phase's RAM-only wait.  g5p_ru_wait_step()/close() decide; this loop only samples.  RC4a: the
 * touch instant is taken after a step that left the phase undecided, before the sleep (off: never).
 * The predicate is level-triggered: every sample, including the one right after each sleep and the
 * closing one at the deadline, re-reads the whole RAM state (rt: the 3D, ta: the TA).
 */
static u32 g5p_rv3_wait(struct g5p_diag *g, struct g5p_render *r, u32 e,
			u32 seq, struct g5p_ru_wait *w, struct g5p_ru_touch *t,
			struct g5p_ru_retire *rt, struct g5p_ru_ta *ta,
			struct g5p_rv3_ev *s, u64 *consumed_ns, int *err)
{
	struct g5p_drm *d = r->ctx->gpu;
	bool closing = false, slow = false;

	for (;;) {
		int drain = g5p_boot_drain(g);
		u64 seen = READ_ONCE(
			d->event_serial); /* after the drain, before the sample */
		bool fault, silent, done;

		g5p_rv3_sample(g, r, e, seq, s);
		/* latency101: a silent event stream during the phase is a dead firmware (never off). */
		silent =
			g5p_ru_wait_grace(w, s->ns, READ_ONCE(g->boot->events));
		g5p_ru_wait_busy(w, s->ns, !s->idle_ok);
		fault = drain || s->failed || s->proto || silent;
		if (drain)
			*err = drain;
		else if (s->proto)
			*err = -EPROTO;
		else if (s->failed)
			*err = -EIO;
		else if (silent)
			*err = -EIO;
		if (s->consumed && !*consumed_ns)
			*consumed_ns = s->ns;
		done = g5p_rv3_done(e, s, rt, ta);
		if (closing)
			g5p_ru_wait_close(w, s->ns, done, fault);
		else
			g5p_ru_wait_step(w, s->ns, done, fault);
		if (rt)
			g5p_ru_rt_note(rt, w->result);
		if (ta)
			g5p_ru_ta_note(ta, w->result);
		if (w->result) {
			g5p_rv3_np_end(g, e, seq, w->result);
			return w->result;
		}
		if (!slow && s->ns - w->start_ns >=
				     (u64)G5P_RV3_SLOW_MS * NSEC_PER_MSEC) {
			slow = true;
			g5p_rv3_slow(g, e, seq, w, s,
				     rt ? rt->unsat :
				     ta ? ta->unsat :
					  0,
				     e ? READ_ONCE(d->td_hit_ns) :
					 READ_ONCE(d->ta_hit_ns));
		}
		if (w->expired) {
			closing = true; /* the closing sample, at once */
			continue;
		}
		if (g5p_ru_touch_due(t, s->ns, w->result, w->expired))
			g5p_rv3_touch(g, t, s->ns);
		g5p_l123_pause(d, g, e, s, seen);
	}
}

/* One phase's deadline: the legacy single bound, or (latency101) the soft/hard pair. */
static void g5p_rv3_wait_start(struct g5p_diag *g, struct g5p_render *r,
			       struct g5p_ru_wait *w, u64 pub_ns,
			       u64 job_deadline_ns)
{
	if (r->hard_ms)
		g5p_ru_wait_start_lat(w, pub_ns, r->soft_ms, r->hard_ms,
				      job_deadline_ns,
				      READ_ONCE(g->boot->events));
	else
		g5p_ru_wait_start(w, pub_ns, r->wait_ms, job_deadline_ns);
}

/* After a timeout (or fault) was decided: the one RAM look of the legacy path, or (latency101,
 * timeout only) a RAM-only observation window that keeps draining and sampling at the wait cadence
 * and prints nothing, so a completion that lands milliseconds after the decision is recorded
 * (late_after/late_ns) before the receipt line and the terminal latch.  It never changes the
 * result and never touches a register.  The 3D look steps a copy of the predicate's record, so the
 * receipt keeps the deciding sample's.
 */
static void g5p_rv3_observe(struct g5p_diag *g, struct g5p_render *r, u32 e,
			    u32 seq, struct g5p_ru_wait *w,
			    const struct g5p_ru_retire *rt,
			    const struct g5p_ru_ta *ta, struct g5p_rv3_ev *late)
{
	struct g5p_ru_retire copy, *c = NULL;
	struct g5p_ru_ta tcopy, *tc = NULL;

	if (rt) {
		copy = *rt;
		c = &copy;
	}
	if (ta) {
		tcopy = *ta;
		tc = &tcopy;
	}
	if (!r->obs_ms || w->result != G5P_RU_WAIT_TIMEOUT) {
		g5p_rv3_sample(g, r, e, seq, late);
		g5p_ru_wait_after(w, g5p_rv3_done(e, late, c, tc));
		return;
	}
	do {
		g5p_boot_drain(
			g); /* a fault latches b->failed, the result stays */
		g5p_rv3_sample(g, r, e, seq, late);
		if (!g5p_ru_wait_observe(w, late->ns,
					 g5p_rv3_done(e, late, c, tc),
					 r->obs_ms))
			break;
		usleep_range(1000, 2000);
	} while (1);
}

/* wake110: render-user-wake.h's register access, on the Fender and RGX windows. */
static void g5p_rv3_wk_w1(void *ctx, u32 reg, u32 value)
{
	writel(value, ((struct g5p_diag *)ctx)->window[1] + reg);
}

/* The GPC state read of g5p_wk_ring() comes right after its R1+0x14 = 7 wake: in bell mode
 * (render_user_wake=2) it waits up to G5P_RV3_WK_SETTLE_US, re-reading every 10 us, for a settled
 * powered pstate (bit 4 clear, nibble nonzero), as Apple's setGPCPerformanceState waits for bit 4.
 * Reads only; the ring's own check of the final value is unchanged.
 */
#define G5P_RV3_WK_SETTLE_US 2000U

static u32 g5p_rv3_wk_r1(void *ctx, u32 reg)
{
	void __iomem *r1 = ((struct g5p_diag *)ctx)->window[1];
	u32 raw = readl(r1 + reg);
	u64 t0;

	if (reg != G5P_WK_R1_GPC || (2) != G5P_WK_ALWAYS ||
	    (!(raw & 0x10) && (raw & 0xf)))
		return raw;
	t0 = ktime_get_mono_fast_ns();
	while (((raw & 0x10) || !(raw & 0xf)) &&
	       ktime_get_mono_fast_ns() - t0 <
		       (u64)G5P_RV3_WK_SETTLE_US * NSEC_PER_USEC) {
		udelay(10);
		raw = readl(r1 + reg);
	}
	return raw;
}

static void g5p_rv3_wk_wmb(void *ctx)
{
	dma_wmb();
}

static void g5p_rv3_wk_w0(void *ctx, u32 reg, u32 value)
{
	writel(value, ((struct g5p_diag *)ctx)->rgx + reg);
}

/* wake110: an r74f bell that still owns its ring (the ALIVE decoder may fire it). */
static bool g5p_rv3_wk_r74f_busy(struct g5p_r73b *x)
{
	u32 k = READ_ONCE(x->r74f_active);
	int s;

	if (!k || k > G5P_R74F_KINDS)
		return false;
	s = atomic_read_acquire(&x->r74f_bell[k - 1].state);
	return s == G5P_R74F_PUBLISHED || s == G5P_R74F_OPEN ||
	       s == G5P_R74F_FIRING;
}

/* wake111: DS108's claim proof re-read at the publish: ALIVE proven in the current reset
 * generation, the claim admitted in it, nothing latched failed since, no R74E power abort.
 * Called under event_lock (interrupts off); g5p_ds_lock is a leaf (g5p_ds_event() is pure).
 */
static bool g5p_rv3_wk_ds108(struct g5p_boot *b)
{
	bool ok;

	spin_lock(&g5p_ds_lock);
	ok = g5p_ds_proven(&g5p_ds_state) &&
	     g5p_ds_ok(&g5p_ds_state, G5P_DSE_CLAIM);
	spin_unlock(&g5p_ds_lock);
	return ok && !(b->r73b && READ_ONCE(b->r73b->early_power_abort));
}

/* wake110: before the phase's producer store, the engine's type-1 count and a cleared first-decode
 * time, so an event decoded right after the publish is counted.  RAM only.
 */
static void g5p_rv3_wake_mark(struct g5p_drm *d, const struct g5p_render *r,
			      struct g5p_rv3 *v, u32 e)
{
	unsigned long flags;

	if (!r->wake)
		return;
	spin_lock_irqsave(&d->status_lock, flags);
	v->wk_t1_pub[e] = d->t1_count[e];
	WRITE_ONCE(d->t1_first_ns[e], 0);
	spin_unlock_irqrestore(&d->status_lock, flags);
}

/* wake110: at the job's end, the type-1 counts and the first decode of each published phase. */
static void g5p_rv3_wake_end(struct g5p_drm *d, const struct g5p_render *r,
			     struct g5p_rv3 *v)
{
	unsigned long flags;

	if (!r->wake)
		return;
	spin_lock_irqsave(&d->status_lock, flags);
	for (u32 e = 0; e < 2; e++) {
		v->wk_t1_end[e] = d->t1_count[e];
		v->wk_first_ns[e] = d->t1_first_ns[e];
	}
	spin_unlock_irqrestore(&d->status_lock, flags);
	if (v->phase < G5P_RV3_TA_PUBLISH)
		v->wk_first_ns[0] = 0;
	if (v->phase < G5P_RV3_TD_PUBLISH)
		v->wk_first_ns[1] = 0;
}

/* wake110: microseconds / milliseconds from a phase's publish to a later instant; -1 = none. */
static s64 g5p_rv3_wk_us(u64 from_ns, u64 to_ns)
{
	return from_ns && to_ns >= from_ns ?
		       (s64)(to_ns - from_ns) / NSEC_PER_USEC :
		       -1;
}

static s64 g5p_rv3_wk_ms(u64 from_ns, u64 to_ns)
{
	return from_ns && to_ns >= from_ns ?
		       (s64)(to_ns - from_ns) / NSEC_PER_MSEC :
		       -1;
}

/* wake110: one phase's publish-time decision, right after its producer store and barrier.  Both
 * arms take the event lock (the r74f owner's serialization against the decoder) and sample the
 * same RAM; only a factor that passed every guard runs g5p_wk_ring().  Nothing is printed under
 * the lock.  Off: returns at once.
 */
static void g5p_rv3_wake(struct g5p_diag *g, struct g5p_render *r,
			 struct g5p_rv3 *v, u32 e, u32 start, u32 seq)
{
	const struct g5p_wk_io io = { g, g5p_rv3_wk_w1, g5p_rv3_wk_r1,
				      g5p_rv3_wk_wmb, g5p_rv3_wk_w0 };
	struct g5p_boot *b = g->boot;
	struct g5p_drm *d = r->ctx->gpu;
	u32 *ring = (u32 *)g5p_boot_shared(b, e ? 0x64 : 0x60);
	u8 *p = g5p_ru_page(b, e);
	struct g5p_wk_in in;
	unsigned long flags;

	if (!r->wake)
		return;
	spin_lock_irqsave(&b->event_lock, flags);
	v->wk_start_ns[e] = ktime_get_ns();
	dma_rmb();
	in = (struct g5p_wk_in){
		.mode = r->wake,
		.seed = 0,
		.seq = seq,
		.e = e,
		.attempted = g5p_wk_attempted(v->wk_did[e]),
		.term_failed = g5p_ru_term_failed(&g5p_rv3_term),
		.boot_failed = READ_ONCE(b->failed) ||
			       READ_ONCE(b->event_failed) ||
			       READ_ONCE(b->stopped),
		.power = g5p_wk_power_mask(
			READ_ONCE(g->powered), READ_ONCE(g->fender_ready),
			READ_ONCE(b->released), READ_ONCE(b->alive),
			READ_ONCE(b->control_complete), g5p_rv3_wk_ds108(b),
			READ_ONCE(g->reset_transition)),
		.r74f_pending = g5p_rv3_wk_r74f_busy(b->r73b),
		.start = start,
		.target = r->target[e],
		.prod = READ_ONCE(ring[4]),
		.cons = READ_ONCE(ring[0]),
		.stamp = { g5p_le32(p + G5P_R22_STAMP),
			   g5p_le32(p + G5P_R22_STAMP + 4) },
		.type1 = READ_ONCE(d->t1_count[e]) - v->wk_t1_pub[e],
	};
	v->wk_cons[e] = in.cons;
	v->wk_power[e] = in.power;
	v->wk_did[e] = g5p_wk_decide(&in, &io, &v->wk_fence[e], &v->wk_gpc[e]);
	v->wk_done_ns[e] = ktime_get_ns();
	spin_unlock_irqrestore(&b->event_lock, flags);
	if (v->wk_did[e] == G5P_WK_DID_RANG)
		r->wake_rang++;
}

/* The epoch's first job: the device (PM, channels) and this context's DC3 records, shared with
 * g5p_render_attach(), then the r74f/r74g protocol of the fixed r74f_attach() on typed state.
 */
static int g5p_rv3_attach(struct g5p_drm *d, struct g5p_render *r,
			  struct g5p_rv3 *v)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_vm *vm = r->ctx->vm;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	unsigned long deadline;
	u32 start, tag, bad;
	int ret;

	ret = g5p_r74f_admit(g, G5P_R74F_DC3);
	if (ret)
		return ret;
	ret = g5p_render_attach_prepare(d, r, &start, &tag);
	if (ret)
		return ret;
	r->dc_target = (start + 2) & 255;
	r->bound = true;
	rd->up = true;
	rd->v3 = true;
	rd->job_slots = (2) == 2 ? 2 : 1;
	rd->pm_once = (1) == 1;
	rd->pm_ctx = b->pm_sets ? min((3), 3U) : 0;
	r->pm_first = r->pm_set < 0;
	dev_info(
		g->dev,
		"RENDER-V3 layout job_slots=%u pm_once=%u slot1=%#x..%#x pm_ctx=%u pm_sets=%u "
		"end=v3\n",
		rd->job_slots, rd->pm_once, G5P_RU_SLOT1, G5P_RU_SLOT1_END,
		rd->pm_ctx, b->pm_sets);
	g5p_render_retain(r);
	v->touched = true;
	WRITE_ONCE(r->ctx->live, true);
	dma_wmb();
	smp_store_release(&ctl[4], r->dc_target);
	dma_wmb();
	v->attach_ns = ktime_get_ns();
	g5p_r74_mark(g, G5P_R74_ATTACH_PUBLISHED, r->dc_target);
	/* The producer is final from here; only the doorbell may be pending. */
	ret = g5p_r74f_bell(g, G5P_R74F_DC3, start, r->dc_target);
	if (ret < 0)
		return g5p_rv3_errno(g, ret);
	deadline = jiffies + 3 * HZ;
	do {
		ret = g5p_boot_drain(g);
		if (ret || READ_ONCE(b->failed))
			return ret ?: -EIO;
		dma_rmb();
		if (READ_ONCE(ctl[4]) != r->dc_target ||
		    READ_ONCE(ctl[0]) > 255)
			return -EPROTO;
		if (READ_ONCE(ctl[0]) == r->dc_target) {
			g5p_r74f_consumed(g, G5P_R74F_DC3);
			bad = g5p_r74g_check(g, 0, r->slot, vm->root.dma, tag,
					     start);
			g5p_r74_mark(g, G5P_R74_ATTACH_CONSUMED, r->dc_target);
			g5p_r74g_dc3_line(g, 0, false);
			if (bad)
				return g5p_rv3_errno(g,
						     g5p_r74g_refuse(g, bad));
			for (u32 e = 0; e < 2; e++)
				g5p_mmu_record(
					g,
					"RENDER tags slot=%u host=%08x cached=%08x expected=%08x\n",
					r->slot[e],
					g5p_le32(g5p_boot_shared(b, 0x84) +
						 r->slot[e] * 4),
					g5p_le32(g5p_boot_shared(b, 0x85) +
						 r->slot[e] * 4),
					tag);
			g5p_mmu_record(
				g,
				"RENDER-USER attached TA=%u 3D=%u PC=%pad tag=%08x blocks=%u; RAM-only, firmware "
				"DC3 installed CBASE, host MMU pass skipped\n",
				r->slot[0], r->slot[1], &vm->root.dma, tag,
				rd->blocks);
			return 0;
		}
		usleep_range(100, 200);
	} while (time_before(jiffies, deadline));
	g5p_r74f_consumed(g, G5P_R74F_DC3);
	return g5p_rv3_errno(g,
			     g5p_r74f_stop(g, G5P_R74F_STOP_ATTACH_NOT_CONSUMED,
					   READ_ONCE(ctl[0])));
}

/* perf60 bell mode (render_user_wake=2): a DC publication (bind, spare pairs, unbind) gets Apple's
 * submit kick too.  With the policy held in state 0 the firmware no longer drains the DC ring on
 * its own (perf60clk-9ec288b868: a second client's RAM-only DC3 bind was never consumed).  The
 * wake path's guards (event lock, power proof, no fault or terminal, no r74f bell owning a ring,
 * the ring holding exactly this publication), then Apple's order: R1+0x14 = 7 and its read-back,
 * R1+0x1000 re-read every 10 us until a settled powered pstate (g5p_gpc_ok(): bit 4 clear,
 * nibble nonzero; Apple's setGPCPerformanceState waits for bit 4 the same way), at most
 * G5P_RV3_DC_SETTLE_US, and R0+0xb00 = 0x10 only then.  A GPC that never reads on is not written:
 * no R0 access, the publication waits as before.  Off (render_user_wake != 2): returns at once.
 */
#define G5P_RV3_DC_SETTLE_US 2000U

static void g5p_rv3_dc_kick(struct g5p_diag *g, u32 start, u32 target,
			    const char *why)
{
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	u32 fence = U32_MAX, gpc = U32_MAX, power, cons, n = 0;
	const char *did = "skip-guard";
	unsigned long flags;
	u64 t0;

	;
	;
	spin_lock_irqsave(&b->event_lock, flags);
	dma_rmb();
	cons = READ_ONCE(ctl[0]);
	power = g5p_wk_power_mask(READ_ONCE(g->powered),
				  READ_ONCE(g->fender_ready),
				  READ_ONCE(b->released), READ_ONCE(b->alive),
				  READ_ONCE(b->control_complete),
				  g5p_rv3_wk_ds108(b),
				  READ_ONCE(g->reset_transition));
	if (READ_ONCE(b->failed) || READ_ONCE(b->event_failed) ||
	    READ_ONCE(b->stopped) || g5p_ru_term_failed(&g5p_rv3_term) ||
	    !g5p_wk_power_ready(power) || g5p_rv3_wk_r74f_busy(b->r73b) ||
	    cons != start || READ_ONCE(ctl[4]) != target)
		goto out;
	writel(7, g->window[1] + 0x14);
	fence = readl(g->window[1] + 0x14);
	if (fence & ~7U) {
		did = "skip-fence";
		goto out;
	}
	t0 = ktime_get_mono_fast_ns();
	gpc = readl(g->window[1] + 0x1000);
	while (!g5p_gpc_ok(gpc) &&
	       ktime_get_mono_fast_ns() - t0 <
		       (u64)G5P_RV3_DC_SETTLE_US * NSEC_PER_USEC) {
		udelay(10);
		gpc = readl(g->window[1] + 0x1000);
		n++;
	}
	if (!g5p_gpc_ok(gpc)) {
		did = "skip-gpc";
		goto out;
	}
	/* the publication's RAM stores before the MTS write (Apple: dmb ish before vt+0x80) */
	wmb();
	writel(0x10, g->rgx + 0xb00);
	did = "rang";
out:
	spin_unlock_irqrestore(&b->event_lock, flags);
	dev_info(
		g->dev,
		"RENDER-V3 dc-kick why=%s did=%s dc=%u->%u power=%02x fence=%08x gpc=%08x "
		"settle=%u end=v3\n",
		why, did, cons, target, power, fence, gpc, n);
}

/* RAM only: drain the event stream, read the firmware pages (interim, see g5p_rv3_fw_touch) and
 * sample the DC ring until its consumer reaches `target`
 * or G5P_RU_BIND_MS pass.  No doorbell and no register, whether or not a job has retired.
 */
static int g5p_rv3_dc_wait(struct g5p_diag *g, u32 target, u64 *consumed_ns)
{
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	unsigned long deadline = jiffies + msecs_to_jiffies(G5P_RU_BIND_MS);

	for (;;) {
		bool expired = time_after_eq(jiffies, deadline);
		int ret = g5p_boot_drain(g);

		if (ret)
			return ret;
		if (READ_ONCE(b->failed))
			return -EIO;
		g5p_rv3_fw_touch(b);
		dma_rmb();
		switch (g5p_ru_bind_step(READ_ONCE(ctl[0]), READ_ONCE(ctl[4]),
					 target, expired)) {
		case G5P_RU_BIND_CONSUMED:
			*consumed_ns = ktime_get_ns();
			return 0;
		}
		usleep_range(100, 200);
	}
}

static s64 g5p_rv3_us(u64 from_ns, u64 to_ns)
{
	return from_ns && to_ns >= from_ns ?
		       (s64)(to_ns - from_ns) / NSEC_PER_USEC :
		       -1;
}

/*
 * The first job of a context on a device that is already up binds the context's slot pair
 * with a DC3 pair published in RAM only (multi-client-design.md §4.2).  No doorbell and no
 * register access of any kind, so it is safe after a job retired and while the GPU may be
 * idle-gated: the firmware services the DC ring in its scheduler pass, before TA and 3D, and
 * the host waits on RAM.  Core lock held, between jobs.
 *
 * Refused before the producer store, it touched nothing.  From the store on, the firmware may
 * hold the pair: the context is retained, and on failure it is lost (v->bind_failed) but the
 * device and the contexts already bound are not.  A pair not consumed in time keeps every later
 * bind refused until the firmware catches up.
 */
/*
 * render_user_close_park (release8 quiet2, docs/gpu/experiments/release8-quiet2.md).  The PM writes
 * a closed context's free list back through that context's TA slot after the context is gone, and
 * not only while it is the last or the previous distinct context (render-user-pmset.h): two bench
 * faults wrote the free list of a context two switches back, through its slot, on the 2nd job of a
 * third context, after the pm-prev rule had unbound it (page-invalid level 2, its PC level empty).
 * iOS never leaves the PM's target unresolvable: its PM is the device's (shared by the work queues,
 * AGXWorkQueue_initAndAttachPM), in the kernel's GPU address space, and a GART keeps its slots until
 * the address space is destroyed (AGXGart_detachSlotsAndDestroyAddressSpace).  Here, a closed
 * context's pair is bound again, right after its unbind (between jobs, core lock held), to the park
 * catalogue: a kernel page that maps only the firmware window, where the PM sets live.  A late
 * write-back then lands in the closed context's own set page (released, reused last) instead of
 * faulting.  A later bind of the pair unbinds it first.  Both steps are the proven DC3
 * transitions (PA to zero, zero to PA); a park that fails leaves the pair unbound, as before.
 */
/* release8 default */

static int g5p_rv3_unbind_slots(struct g5p_drm *d, const u32 *slots, u32 n,
				const char *why);

/* Before a bind, core lock held: a pair a close parked (RAM: park PA and tag on both slots) is
 * unbound first, by its own DC3.  0 when the pair was not parked or the unbind verified.
 */
static int g5p_rv3_unpark(struct g5p_drm *d, const u32 *slot)
{
	struct g5p_boot *b = d->core->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	struct g5p_ru_bind_sample s = {};
	struct g5p_ru_bind_mem m;

	if (!d->rdev->park.cpu)
		return 0;
	dma_rmb();
	g5p_render_bind_mem(b, slot, READ_ONCE(ctl[4]), &m);
	g5p_ru_bind_read(&m, READ_ONCE(ctl[0]), READ_ONCE(ctl[4]), &s);
	if (!g5p_ru_bind_parked(&s, d->rdev->park.dma))
		return 0;
	return g5p_rv3_unbind_slots(d, slot, 2, "unpark");
}

static int g5p_rv3_bind(struct g5p_drm *d, struct g5p_render *r,
			struct g5p_rv3 *v)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_vm *vm = r->ctx->vm;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	struct g5p_ru_bind_sample s = {};
	struct g5p_ru_bind_mem m;
	u64 pub_ns, consumed_ns = 0;
	u32 start, tag, bad = 0;
	int ret;

	dma_rmb();
	if (rd->dc_pending && READ_ONCE(ctl[0]) != rd->dc_wait) {
		dev_emerg(
			g->dev,
			"RENDER-V3 bind refused slots=%u/%u dc=%u/%u wait=%u: an earlier bind is "
			"pending; no hardware touched end=v3\n",
			r->slot[0], r->slot[1], READ_ONCE(ctl[0]),
			READ_ONCE(ctl[4]), rd->dc_wait);
		return -EIO;
	}
	rd->dc_pending = false;
	ret = g5p_rv3_unpark(d, r->slot);
	if (ret)
		return ret;
	ret = g5p_render_vm_prepare(d, r, &start, &tag);
	if (ret) {
		dev_emerg(
			g->dev,
			"RENDER-V3 bind refused slots=%u/%u ret=%d dc=%u/%u: ring busy or slot in use; "
			"no hardware touched end=v3\n",
			r->slot[0], r->slot[1], ret, READ_ONCE(ctl[0]),
			READ_ONCE(ctl[4]));
		return ret;
	}
	r->dc_target = (start + 2) & 255;
	g5p_render_retain(r);
	v->bind_failed = true;
	dma_wmb();
	pub_ns = ktime_get_ns();
	smp_store_release(&ctl[4], r->dc_target);
	dma_wmb();
	g5p_rv3_dc_kick(g, start, r->dc_target, "bind");
	ret = g5p_rv3_dc_wait(g, r->dc_target, &consumed_ns);
	if (ret == -ETIMEDOUT) {
		rd->dc_pending = true;
		rd->dc_wait = r->dc_target;
	}
	dma_rmb();
	g5p_render_bind_mem(b, r->slot, start, &m);
	g5p_ru_bind_read(&m, READ_ONCE(ctl[0]), READ_ONCE(ctl[4]), &s);
	if (!ret) {
		bad = g5p_ru_bind_check(&s, r->slot, vm->root.dma, tag,
					r->dc_target);
		ret = bad ? -EUCLEAN : 0;
	}
	dev_emerg(
		g->dev,
		"RENDER-V3 bind slots=%u/%u ret=%d consumed_us=%lld tag=%08x host=%08x/%08x "
		"cached=%08x/%08x dc=%u->%u/%u bad=%#x pc=%pad mode=ram-dc3 end=v3\n",
		r->slot[0], r->slot[1], ret, g5p_rv3_us(pub_ns, consumed_ns),
		tag, s.host[0], s.host[1], s.cached[0], s.cached[1], start,
		s.cons, s.prod, bad, &vm->root.dma);
	if (ret)
		return ret;
	v->bind_failed = false;
	r->bound = true;
	WRITE_ONCE(r->ctx->live, true);
	return 0;
}

/*
 * TEMPORARY fallback (render_user_spare_pairs, read at the epoch's first job), for a firmware
 * that never drains a DC3 published after a job retired.  Before job 1's attach, bind that
 * many extra slot pairs, each to a zeroed kernel-owned catalogue, in one RAM-only batch with no
 * doorbell.  The firmware drains it on its own like the self-drained attach of this boot tuple
 * (r87b); job 1's own attach, doorbell and r74g checks then run unchanged behind it
 * (g5p_r74f_ring() refuses a ring that holds more than that attach's pair).  Later contexts
 * take a pair over with g5p_rv3_adopt().  A pair that does not check out is never used.  Core
 * lock held, device not up, so register access is still allowed (none is made here).
 */
static int g5p_rv3_spares(struct g5p_drm *d, struct g5p_rv3 *v)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	u32 want = min((0), G5P_RU_MAX_SPARES);
	u32 start = READ_ONCE(ctl[4]), target, n, ok = 0;
	struct g5p_ru_bind_sample s;
	struct g5p_ru_bind_mem m;
	u64 pub_ns, consumed_ns = 0;
	unsigned long flags;
	int ret;

	if (!want || rd->spares)
		return 0;
	rd->spares = kcalloc(want, sizeof(*rd->spares), GFP_KERNEL);
	if (!rd->spares)
		return -ENOMEM;
	for (n = 0; n < want; n++) {
		struct g5p_rspare *sp = &rd->spares[n];
		bool pair;

		spin_lock_irqsave(&g->slot_lock, flags);
		pair = g5p_ru_slot_pair(g->user_slots, false, sp->slot);
		if (pair) {
			__set_bit(sp->slot[0], g->user_slots);
			__set_bit(sp->slot[1], g->user_slots);
		}
		spin_unlock_irqrestore(&g->slot_lock, flags);
		if (!pair)
			break;
		g5p_render_bind_mem(b, sp->slot, start + 2 * n, &m);
		g5p_ru_bind_read(&m, READ_ONCE(ctl[0]), start, &s);
		if (!g5p_ru_bind_busy(&s) &&
		    !g5p_vm_alloc_page(d->drm.dev, &sp->root)) {
			g5p_table_clean_page(g, &sp->root);
			continue;
		}
		spin_lock_irqsave(&g->slot_lock, flags);
		__clear_bit(sp->slot[0], g->user_slots);
		__clear_bit(sp->slot[1], g->user_slots);
		spin_unlock_irqrestore(&g->slot_lock, flags);
		break;
	}
	if (!n)
		return 0;
	dsb(sy);
	for (u32 i = 0; i < n; i++) {
		g5p_render_bind_mem(b, rd->spares[i].slot, start + 2 * i, &m);
		g5p_ru_bind_publish(&m, rd->spares[i].slot,
				    rd->spares[i].root.dma, G5P_RU_SPARE_TAG);
	}
	rd->spare_count = n;
	target = (start + 2 * n) & 255;
	v->touched = true;
	dma_wmb();
	pub_ns = ktime_get_ns();
	smp_store_release(&ctl[4], target);
	dma_wmb();
	g5p_rv3_dc_kick(g, start, target, "spares");
	ret = g5p_rv3_dc_wait(g, target, &consumed_ns);
	for (u32 i = 0; !ret && i < n; i++) {
		struct g5p_rspare *sp = &rd->spares[i];

		g5p_render_bind_mem(b, sp->slot, start + 2 * i, &m);
		g5p_ru_bind_read(&m, READ_ONCE(ctl[0]), READ_ONCE(ctl[4]), &s);
		sp->ok = !g5p_ru_bind_check(&s, sp->slot, sp->root.dma,
					    G5P_RU_SPARE_TAG, target);
		ok += sp->ok;
	}
	dev_emerg(
		g->dev,
		"RENDER-V3 spares pairs=%u ok=%u ret=%d consumed_us=%lld dc=%u->%u first=%u/%u "
		"last=%u/%u end=v3\n",
		n, ok, ret, g5p_rv3_us(pub_ns, consumed_ns), start,
		READ_ONCE(ctl[0]), rd->spares[0].slot[0], rd->spares[0].slot[1],
		rd->spares[n - 1].slot[0], rd->spares[n - 1].slot[1]);
	return ret;
}

/*
 * TEMPORARY fallback: the first job of a context on a device whose job 1 bound spare pairs.
 * Takes the next pair that checked out, makes its catalogue page this VM's root (a copy of the
 * current root) and publishes the VM's tag through the proven RAM-only tag path: CBASE already
 * names the page, and the changed tag makes the firmware invalidate the slots' PC level at this
 * context's first job.  No DC3, no register.  Core lock held; the context has a job queued, so
 * no VM edit of it runs concurrently.
 */
static int g5p_rv3_adopt(struct g5p_drm *d, struct g5p_render *r,
			 struct g5p_rv3 *v)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_vm *vm = r->ctx->vm;
	struct g5p_rspare *sp = NULL;
	struct g5p_ru_bind_sample s;
	struct g5p_ru_bind_mem m;
	struct g5p_dma_page old;
	unsigned long flags;
	struct g5p_rt *rt;
	u32 entry;
	int ret;

	for (u32 i = 0; i < rd->spare_count && !sp; i++)
		if (rd->spares[i].ok && !rd->spares[i].used)
			sp = &rd->spares[i];
	if (!sp) {
		dev_emerg(
			g->dev,
			"RENDER-V3 bind refused slots=%u/%u mode=spare: none left of %u; no hardware "
			"touched end=v3\n",
			r->slot[0], r->slot[1], rd->spare_count);
		return -ENOSPC;
	}
	/* RAM: the pair still names its page, with the tag job 1 checked. */
	dma_rmb();
	g5p_render_bind_mem(b, sp->slot, 0, &m);
	g5p_ru_bind_read(&m, 0, 0, &s);
	for (u32 e = 0; e < 2; e++) {
		if (s.pa[e] == sp->root.dma && s.host[e] == G5P_RU_SPARE_TAG &&
		    s.cached[e] == G5P_RU_SPARE_TAG)
			continue;
		sp->ok = false;
		dev_emerg(
			g->dev,
			"RENDER-V3 bind refused slots=%u/%u mode=spare pair=%u/%u host=%08x cached=%08x: "
			"pair changed; no hardware touched end=v3\n",
			r->slot[0], r->slot[1], sp->slot[0], sp->slot[1],
			s.host[e], s.cached[e]);
		return -EIO;
	}
	ret = g5p_render_fw_entry(d, r, &entry);
	if (ret)
		return ret;
	g5p_vm_entry(&vm->mutation, vm->root.cpu + G5P_RU_FIRMWARE_CI * 4,
		     entry, 0);
	if (b->sink_ready)
		g5p_vm_entry(&vm->mutation, vm->root.cpu,
			     g5p_le32(b->catalogue.cpu), 0);
	memcpy(sp->root.cpu, vm->root.cpu, G5P_GPU_PAGE);
	old = vm->root;
	vm->root = sp->root;
	sp->root = old; /* the VM's former root, never published */
	g5p_ru_free_page(d->drm.dev, &sp->root);
	list_for_each_entry(rt, &r->targets, link)
		for (u32 i = 0; i < rt->page_count; i++)
			g5p_table_clean_page(g, &rt->pages[i]);
	spin_lock_irqsave(&g->slot_lock, flags);
	__clear_bit(r->slot[0], g->user_slots);
	__clear_bit(r->slot[1], g->user_slots);
	spin_unlock_irqrestore(&g->slot_lock, flags);
	r->slot[0] = sp->slot[0];
	r->slot[1] = sp->slot[1];
	sp->used = true;
	g5p_render_retain(r);
	r->bound = true;
	WRITE_ONCE(r->ctx->live, true);
	/* Root and tables to PoC, then a tag unlike the cached one in both slots. */
	ret = g5p_render_invalidate(r->ctx);
	v->bind_failed = ret != 0;
	dev_emerg(
		g->dev,
		"RENDER-V3 bind slots=%u/%u ret=%d consumed_us=0 tag=%08x cached=%08x/%08x "
		"pc=%pad mode=spare end=v3\n",
		r->slot[0], r->slot[1], ret,
		g5p_le32(g5p_boot_shared(b, 0x84) + r->slot[0] * 4),
		s.cached[0], s.cached[1], &vm->root.dma);
	return ret;
}

/* r74f_publish_ta() on typed state: record, WORK_CTL, producer; then (job 1 only) the r74g grace
 * and, only if the ring is still pending, the one ALIVE-owned fallback doorbell.  Jobs >= 2
 * publish and return: no grace, no bell (the r74f/r74g machinery is one-shot).  No canary, no marker.
 */
static int g5p_rv3_publish_ta(struct g5p_diag *g, struct g5p_render *r,
			      struct g5p_rv3 *v, u32 seq)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *x = b->r73b;
	struct g5p_vm *vm = r->ctx->vm;
	struct g5p_r74g_dc3 *dc = x->r74g_dc3;
	u32 *ring = (u32 *)g5p_boot_shared(b, 0x60);
	u8 *p = g5p_ru_page(b, 0);
	u32 start = READ_ONCE(ring[4]), previous = g5p_ru_endpoint(seq - 1),
	    endpoint = g5p_ru_endpoint(seq);
	bool first = seq == 1;
	u32 bad;
	int ret;

	if (!r->bound || r->submitted[0] || start > 255 ||
	    READ_ONCE(ring[0]) != start ||
	    g5p_le32(p + G5P_WORK_CTL) != previous ||
	    g5p_le32(p + G5P_WORK_CTL + 0x40) != previous)
		return -EBUSY;
	if (first) {
		ret = g5p_r74f_admit(g, G5P_R74F_TA);
		if (ret)
			return ret;
		/* g5p_r74g_ta_preflight() on typed state: DC3 still complete with the same records,
		 * tag and backing, and no page-table mutation since the attach.
		 */
		bad = g5p_r74g_check(g, 1, r->slot, dc[0].expected_pa,
				     dc[0].tag, dc[0].start);
		if (g5p_vm_generation(&vm->mutation) != dc[0].tag ||
		    vm->root.dma != dc[0].expected_pa ||
		    r->dc_target != dc[0].target) {
			bad |= G5P_R74G_BAD_MUTATION;
			dc[1].bad = bad;
		}
		g5p_r74g_dc3_line(g, 1, false);
		if (bad)
			return g5p_rv3_errno(g, g5p_r74g_refuse(g, bad));
	}
	for (u32 j = 0; j < 5; j++) {
		u32 at = b->layout.offset[0x61] + start * 20 + j * 4;

		g5p_put32(b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE,
			  j == 0 ? 0 :
			  j == 1 ? g5p_ru_meta(0) :
			  j == 2 ? endpoint :
				   0);
	}
	r->target[0] = (start + 1) & 255;
	r->submitted[0] = true;
	dma_wmb();
	WRITE_ONCE(*(u32 *)(p + G5P_WORK_CTL + 0x40), endpoint);
	dma_wmb();
	if (first)
		g5p_r74_mark(g, G5P_R74_TA_PUBLISH, start);
	v->ta_pub_ns = ktime_get_ns();
	v->job_deadline_ns =
		v->ta_pub_ns +
		(u64)(r->hard_ms ? 2 * r->hard_ms : G5P_RU_V3_CAP_MS) *
			NSEC_PER_MSEC;
	smp_store_release(&ring[4], r->target[0]);
	dma_wmb();
	if (first) {
		ret = g5p_r74g_ta(g, start, r->target[0]);
		v->grace_end = READ_ONCE(x->r74g_grace_end);
		v->bell = ret == 0;
		if (ret < 0)
			return g5p_rv3_errno(g, ret);
	}
	/* wake110: jobs >= 2 reach this at once after the producer store (job 1: excluded). */
	if (!first)
		0;
	g5p_rv3_wake(g, r, v, 0, start, seq);
	/* The line the userspace lane parses (a9c_parse_publish): same format as the fixed path's. */
	g5p_mmu_record_job(
		g,
		"RENDER publish engine=%u slot=%u endpoint=%u kick=%x W=%08x sequence=%u "
		"identity=%u\n",
		0, r->slot[0], endpoint, first ? 0x10 : 0,
		g5p_ru_meta(0) +
			g5p_ru_w_at(g5p_rv3_js(r->ctx->gpu->rdev, seq)),
		seq, 0);
	return 0;
}

/* The RAM preconditions of the 3D publish, one bit per clause (0 = admitted).  RAM reads only; the bits are
 * what the refusal line prints, so a second blocker is visible in the run that meets it:
 *   0 TA not retired   1 TA not consumed   2 3D already submitted   3 producer > 255   4 consumer != producer
 *   5 WORK_CTL         6 WORK_CTL+0x40     7 command tag != 4       8 dependency word  9 dependency value
 *  10 3D stamp words != seq - 1 (job N leaves N; 0 before the first)
 */
static u32 g5p_rv3_td_why(const struct g5p_render *r, const u8 *p,
			  const u32 *ring, u32 start, u32 previous, u32 seq)
{
	const u8 *cmd = p + g5p_ru_cmd_at(g5p_rv3_js(r->ctx->gpu->rdev, seq));

	return (u32)!r->retired[0] | (u32)!r->consumed[0] << 1 |
	       (u32)r->submitted[1] << 2 | (u32)(start > 255) << 3 |
	       (u32)(READ_ONCE(ring[0]) != start) << 4 |
	       (u32)(g5p_le32(p + G5P_WORK_CTL) != previous) << 5 |
	       (u32)(g5p_le32(p + G5P_WORK_CTL + 0x40) != previous) << 6 |
	       (u32)(g5p_le32(cmd) != 4) << 7 |
	       (u32)(g5p_le32(cmd + 4) != g5p_ru_meta(0) + G5P_R22_STAMP + 4)
		       << 8 |
	       (u32)(g5p_le32(cmd + 8) != seq) << 9 |
	       (u32)(g5p_le32(p + G5P_R22_STAMP) != seq - 1 ||
		     g5p_le32(p + G5P_R22_STAMP + 4) != seq - 1)
		       << 10;
}

/* The 3D after the TA retired: record, WORK_CTL, producer, and NO doorbell. */
static int g5p_rv3_publish_3d(struct g5p_diag *g, struct g5p_render *r,
			      struct g5p_rv3 *v, u32 seq)
{
	struct g5p_boot *b = g->boot;
	u32 *ring = (u32 *)g5p_boot_shared(b, 0x64);
	u8 *p = g5p_ru_page(b, 1);
	u32 start = READ_ONCE(ring[4]), previous = g5p_ru_endpoint(seq - 1),
	    endpoint = g5p_ru_endpoint(seq);
	u32 why;

	/* RAM preconditions of r96's admit (g5p_rv3_td_why()).  Nothing has been written when one
	 * fails, and the refusal names the clauses and the raw values it read.
	 */
	why = g5p_rv3_td_why(r, p, ring, start, previous, seq);
	if (why) {
		dev_emerg(
			g->dev,
			"RENDER-V3 3d-refused seq=%u why=%#x ring=%u/%u ctl=%08x/%08x cmd=%08x/%08x/%08x "
			"stamp=%08x/%08x end=v3\n",
			seq, why, start, READ_ONCE(ring[0]),
			g5p_le32(p + G5P_WORK_CTL),
			g5p_le32(p + G5P_WORK_CTL + 0x40),
			g5p_le32(p + g5p_ru_cmd_at(g5p_rv3_js(r->ctx->gpu->rdev,
							      seq))),
			g5p_le32(p +
				 g5p_ru_cmd_at(
					 g5p_rv3_js(r->ctx->gpu->rdev, seq)) +
				 4),
			g5p_le32(p +
				 g5p_ru_cmd_at(
					 g5p_rv3_js(r->ctx->gpu->rdev, seq)) +
				 8),
			g5p_le32(p + G5P_R22_STAMP),
			g5p_le32(p + G5P_R22_STAMP + 4));
		return -EPROTO;
	}
	for (u32 j = 0; j < 5; j++) {
		u32 at = b->layout.offset[0x65] + start * 20 + j * 4;

		g5p_put32(b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE,
			  j == 0 ? 1 :
			  j == 1 ? g5p_ru_meta(1) :
			  j == 2 ? endpoint :
				   0);
	}
	r->target[1] = (start + 1) & 255;
	r->submitted[1] = true;
	dma_wmb();
	WRITE_ONCE(*(u32 *)(p + G5P_WORK_CTL + 0x40), endpoint);
	dma_wmb();
	v->td_pub_ns = ktime_get_ns();
	smp_store_release(&ring[4], r->target[1]);
	dma_wmb();
	0;
	g5p_rv3_wake(g, r, v, 1, start, seq);
	/* The 3D twin of the TA's publish line: engine 1, kick 0 (no doorbell). */
	g5p_mmu_record_job(
		g,
		"RENDER publish engine=%u slot=%u endpoint=%u kick=%x W=%08x sequence=%u "
		"identity=%u\n",
		1, r->slot[1], endpoint, 0,
		g5p_ru_meta(1) +
			g5p_ru_w_at(g5p_rv3_js(r->ctx->gpu->rdev, seq)),
		seq, 1);
	return 0;
}

static const char *const g5p_rv3_ta_how[G5P_RU_TA_HOWS] = { "none",
							    "idle-clear",
							    "event", "hold" };

/* tahang1: the TA decisions since load, by rule (core lock held).  A TA that completed with an
 * idle word still set prints one line with the running totals: rate-limited for rule (b), which
 * is routine, always for rule (c), which means the event was not seen.  RAM only.
 */
static u32 g5p_rv3_ta_rules[G5P_RU_TA_HOWS];
static DEFINE_RATELIMIT_STATE(g5p_rv3_ta_rs, 5 * HZ, 4);

static void g5p_rv3_ta_count(struct g5p_diag *g, struct g5p_drm *d,
			     const struct g5p_rv3 *v)
{
	const struct g5p_rv3_ev *s = &v->last[0];
	u32 how = min_t(u32, v->tr.decided, G5P_RU_TA_HOWS - 1);
	u64 hit = READ_ONCE(d->ta_hit_ns);

	g5p_rv3_ta_rules[how]++;
	if (how == G5P_RU_TA_IDLE_CLEAR)
		return;
	if (how == G5P_RU_TA_EVENT && !__ratelimit(&g5p_rv3_ta_rs))
		return;
	dev_warn(
		g->dev,
		"RENDER-V3 ta-done seq=%u how=%s idle=%08x,%08x,%08x pm80=%08x coherent_ms=%lld "
		"event_ms=%lld decided_ms=%lld busy_ms=%lld rules=%u/%u/%u end=v3\n",
		v->seq, g5p_rv3_ta_how[how], s->idle[0], s->idle[1], s->idle[2],
		s->pm80, g5p_ru_rt_ms(v->ta_pub_ns, v->tr.coherent_ns),
		g5p_ru_rt_ms(v->ta_pub_ns, hit),
		g5p_ru_rt_ms(v->ta_pub_ns, v->ta.end_ns),
		g5p_ru_rt_ms(v->ta_pub_ns, v->ta.busy_ns),
		g5p_rv3_ta_rules[G5P_RU_TA_IDLE_CLEAR],
		g5p_rv3_ta_rules[G5P_RU_TA_EVENT],
		g5p_rv3_ta_rules[G5P_RU_TA_HOLD]);
}

/* A receipt line: a failed job's at KERN_EMERG as before (it survives a freeze), a completed job's at
 * job_log (wake every job, end/lat/retire the first 8 of a context: routine at desktop rates).
 */
#define g5p_rv3_receipt_printk(g, ret, fmt, ...)                      \
	do {                                                          \
		if (ret)                                              \
			dev_emerg((g)->dev, fmt, ##__VA_ARGS__);      \
		else                                                  \
			g5p_job_printk((g)->dev, fmt, ##__VA_ARGS__); \
	} while (0)

/* The end of a job, whatever it was: one receipt line on the console (it survives a freeze) and the
 * RAM-only raw state in the blob.  Reads RAM only.
 */
static void g5p_rv3_receipt(struct g5p_diag *g, struct g5p_render *r,
			    struct g5p_rjob *job, int ret)
{
	struct g5p_rv3 *v = &job->rv3;
	struct g5p_r73b *x = g->boot->r73b;
	const struct g5p_rv3_ev *t = &v->last[0], *d = &v->last[1];
	const struct g5p_ru_touch *k = v->touch;

	/* RC4a: every job while the A/B is on, so both arms are complete past the verbose jobs. */
	if (r->touch_ms)
		dev_emerg(
			g->dev,
			"RENDER-V3 touch seq=%u ret=%d arm=%s touch_ms=%u ta=%s/%lld td=%s/%lld "
			"seen_us=%lld/%lld read_us=%lld/%lld after_us=%lld/%lld bytes=%u/%u "
			"sum=%08x/%08x end=v3\n",
			v->seq, ret,
			g5p_ru_touch_arm(v->seq) ? "touch" : "control",
			r->touch_ms, g5p_rv3_results[v->ta.result],
			v->ta.end_ns ? (s64)(v->ta.end_ns - v->ta_pub_ns) /
					       NSEC_PER_MSEC :
				       -1,
			g5p_rv3_results[v->td.result],
			v->td.end_ns ? (s64)(v->td.end_ns - v->td_pub_ns) /
					       NSEC_PER_MSEC :
				       -1,
			g5p_ru_touch_us(v->ta_pub_ns, k[0].seen_ns),
			g5p_ru_touch_us(v->td_pub_ns, k[1].seen_ns),
			g5p_ru_touch_us(k[0].seen_ns, k[0].done_ns),
			g5p_ru_touch_us(k[1].seen_ns, k[1].done_ns),
			g5p_ru_touch_after_us(&k[0], v->ta.result,
					      v->ta.end_ns),
			g5p_ru_touch_after_us(&k[1], v->td.result,
					      v->td.end_ns),
			k[0].bytes, k[1].bytes, k[0].sum, k[1].sum);
	/* wake110: every job while the wake is on (the end/lat/retire lines stop at seq 8). */
	if (r->wake)
		g5p_rv3_receipt_printk(
			g, ret,
			"RENDER-V3 wake seq=%u ret=%d mode=%u seed=%08x arm=%s/%s did=%s/%s "
			"power=%02x/%02x fence=%08x/%08x gpc=%08x/%08x cons_at=%u/%u bell_us=%lld/%lld "
			"lock_us=%lld/%lld ta=%s/%lld td=%s/%lld ev_ms=%lld/%lld consumed_ms=%lld/%lld "
			"busy_ms=%lld/%lld t1_pub=%u/%u t1_end=%u/%u endpoint=%u ta_stamp=%08x/%08x "
			"td_stamp=%08x/%08x ta_ring=%u/%u td_ring=%u/%u rang=%u end=v3\n",
			v->seq, ret, r->wake, r->wake_seed,
			g5p_wk_arm_names[g5p_wk_arm(r->wake, r->wake_seed,
						    v->seq, 0)],
			g5p_wk_arm_names[g5p_wk_arm(r->wake, r->wake_seed,
						    v->seq, 1)],
			g5p_wk_did_names[min_t(u32, v->wk_did[0],
					       G5P_WK_DIDS - 1)],
			g5p_wk_did_names[min_t(u32, v->wk_did[1],
					       G5P_WK_DIDS - 1)],
			v->wk_power[0], v->wk_power[1], v->wk_fence[0],
			v->wk_fence[1], v->wk_gpc[0], v->wk_gpc[1],
			v->wk_cons[0], v->wk_cons[1],
			g5p_rv3_wk_us(v->ta_pub_ns, v->wk_done_ns[0]),
			g5p_rv3_wk_us(v->td_pub_ns, v->wk_done_ns[1]),
			g5p_rv3_wk_us(v->wk_start_ns[0], v->wk_done_ns[0]),
			g5p_rv3_wk_us(v->wk_start_ns[1], v->wk_done_ns[1]),
			g5p_rv3_results[v->ta.result],
			g5p_rv3_wk_ms(v->ta_pub_ns, v->ta.end_ns),
			g5p_rv3_results[v->td.result],
			g5p_rv3_wk_ms(v->td_pub_ns, v->td.end_ns),
			g5p_rv3_wk_ms(v->ta_pub_ns, v->wk_first_ns[0]),
			g5p_rv3_wk_ms(v->td_pub_ns, v->wk_first_ns[1]),
			g5p_rv3_wk_ms(v->ta_pub_ns, v->ta_consumed_ns),
			g5p_rv3_wk_ms(v->td_pub_ns, v->td_consumed_ns),
			g5p_rv3_wk_ms(v->ta_pub_ns, v->ta.busy_ns),
			g5p_rv3_wk_ms(v->td_pub_ns, v->td.busy_ns),
			v->wk_t1_pub[0], v->wk_t1_pub[1], v->wk_t1_end[0],
			v->wk_t1_end[1], g5p_ru_endpoint(v->seq), t->stamp[0],
			t->stamp[1], d->stamp[0], d->stamp[1], t->prod, t->cons,
			d->prod, d->cons, r->wake_rang);
	if (!g5p_ru_v3_verbose(r->jobs, ret))
		return;
	g5p_rv3_receipt_printk(
		g, ret,
		"RENDER-V3 end seq=%u ret=%d phase=%s ta=%s/%lld td=%s/%lld bound_ms=%u "
		"cap_ms=%u ta_consumed_ms=%lld td_consumed_ms=%lld late_gap=%u/%u "
		"late_after=%u/%u grace=%u bells=%d ta_ring=%u/%u td_ring=%u/%u "
		"ta_stamp=%08x/%08x td_stamp=%08x/%08x pm80=%08x r0_after_ta_publish=bell-only "
		"end=v3\n",
		v->seq, ret,
		g5p_rv3_phase_names[min_t(u32, v->phase, G5P_RV3_PHASES - 1)],
		g5p_rv3_results[v->ta.result],
		v->ta.end_ns ?
			(s64)(v->ta.end_ns - v->ta_pub_ns) / NSEC_PER_MSEC :
			-1,
		g5p_rv3_results[v->td.result],
		v->td.end_ns ?
			(s64)(v->td.end_ns - v->td_pub_ns) / NSEC_PER_MSEC :
			-1,
		r->wait_ms, G5P_RU_V3_CAP_MS,
		v->ta_consumed_ns ? (s64)(v->ta_consumed_ns - v->ta_pub_ns) /
					    NSEC_PER_MSEC :
				    -1,
		v->td_consumed_ns ? (s64)(v->td_consumed_ns - v->td_pub_ns) /
					    NSEC_PER_MSEC :
				    -1,
		v->ta.late_gap, v->td.late_gap, v->ta.late_after,
		v->td.late_after, v->grace_end, atomic_read(&x->r74f_bells),
		t->prod, t->cons, d->prod, d->cons, t->stamp[0], t->stamp[1],
		d->stamp[0], d->stamp[1], t->pm80 ? t->pm80 : d->pm80);
	if (r->hard_ms)
		g5p_rv3_receipt_printk(
			g, ret,
			"RENDER-V3 lat seq=%u soft_ms=%u hard_ms=%u obs_ms=%u slow=%u/%u late_soft=%u/%u "
			"silent=%u/%u soft_seen_ms=%lld/%lld busy_ms=%lld/%lld late_after_ms=%lld/%lld "
			"events=%u end=v3\n",
			v->seq, r->soft_ms, r->hard_ms, r->obs_ms, v->ta.slow,
			v->td.slow, v->ta.late_soft, v->td.late_soft,
			v->ta.silent, v->td.silent,
			v->ta.soft_seen_ns ?
				(s64)(v->ta.soft_seen_ns - v->ta_pub_ns) /
					NSEC_PER_MSEC :
				-1,
			v->td.soft_seen_ns ?
				(s64)(v->td.soft_seen_ns - v->td_pub_ns) /
					NSEC_PER_MSEC :
				-1,
			v->ta.busy_ns ? (s64)(v->ta.busy_ns - v->ta_pub_ns) /
						NSEC_PER_MSEC :
					-1,
			v->td.busy_ns ? (s64)(v->td.busy_ns - v->td_pub_ns) /
						NSEC_PER_MSEC :
					-1,
			v->ta.late_ns ? (s64)(v->ta.late_ns - v->ta_pub_ns) /
						NSEC_PER_MSEC :
					-1,
			v->td.late_ns ? (s64)(v->td.late_ns - v->td_pub_ns) /
						NSEC_PER_MSEC :
					-1,
			READ_ONCE(g->boot->events));
	/* repeat109: the 3D's firmware retirement (coherent_ms, event_ms) apart from the host's decision
	 * (decided_ms; held_ms = how long a coherent target waited for it), the clauses still open at the
	 * deciding or closing sample (unsat=TA/3D, render-user-retire.h bits) and that sample's 3D idle words.
	 */
	g5p_rv3_receipt_printk(
		g, ret,
		"RENDER-V3 retire seq=%u ret=%d td=%s how=%s settle_ms=%u coherent_ms=%lld "
		"event_ms=%lld decided_ms=%lld held_ms=%lld unsat=%#x/%#x "
		"idle=%08x,%08x,%08x,%08x,%08x,%08x run=%u breaks=%u busy_ms=%lld clear_ms=%lld "
		"ta_how=%s ta_unsat=%#x ta_coherent_ms=%lld ta_event_ms=%lld "
		"ta_idle=%08x,%08x,%08x end=v3\n",
		v->seq, ret, g5p_rv3_results[v->td.result],
		g5p_rv3_how[min_t(u32, v->rt.decided, 3)], r->settle_ms,
		g5p_ru_rt_ms(v->td_pub_ns, v->rt.coherent_ns),
		g5p_ru_rt_ms(v->td_pub_ns, v->td_event_ns),
		g5p_ru_rt_ms(v->td_pub_ns, v->td.end_ns),
		g5p_ru_rt_ms(v->rt.coherent_ns, v->td.end_ns),
		g5p_rv3_ta_unsat(t), v->rt.unsat, d->idle[0], d->idle[1],
		d->idle[2], d->idle[3], d->idle[4], d->idle[5], v->rt.run,
		v->rt.breaks, g5p_ru_rt_ms(v->td_pub_ns, v->rt.busy_ns),
		g5p_ru_rt_ms(v->td_pub_ns, v->rt.clear_ns),
		g5p_rv3_ta_how[min_t(u32, v->tr.decided, G5P_RU_TA_HOWS - 1)],
		v->tr.unsat, g5p_ru_rt_ms(v->ta_pub_ns, v->tr.coherent_ns),
		g5p_ru_rt_ms(v->ta_pub_ns, v->ta_event_ns), t->idle[0],
		t->idle[1], t->idle[2]);
	/* The raw rows once per context (its first job) and for every failure: the blob is not a
	 * frame log.
	 */
	if (!v->touched || (r->jobs > 1 && !ret))
		return;
	g5p_blob_hex(g, "RENDER-V3 PM-control", g5p_rv3_pm_cpu(g->boot, r),
		     0x140);
	g5p_blob_hex(g, "RENDER-V3 TA-W",
		     g5p_ru_page(g->boot, 0) +
			     g5p_ru_w_at(g5p_rv3_js(r->ctx->gpu->rdev, v->seq)),
		     G5P_RU_TA_W_BYTES);
	g5p_blob_hex(g, "RENDER-V3 TA-CH", g5p_ru_page(g->boot, 0), 0x340);
	/* The stamp words (+0x1840/+0x1844) live in this 0x80-byte window; no earlier blob held one. */
	g5p_blob_hex(g, "RENDER-V3 TA-STAMP",
		     g5p_ru_page(g->boot, 0) + G5P_R22_TQ, 0x80);
	if (v->phase >= G5P_RV3_TD_PUBLISH) {
		g5p_blob_hex(g, "RENDER-V3 3D-W",
			     g5p_ru_page(g->boot, 1) +
				     g5p_ru_w_at(g5p_rv3_js(r->ctx->gpu->rdev,
							    v->seq)),
			     G5P_RU_3D_W_BYTES);
		g5p_blob_hex(g, "RENDER-V3 3D-CH", g5p_ru_page(g->boot, 1),
			     0x340);
		g5p_blob_hex(g, "RENDER-V3 3D-STAMP",
			     g5p_ru_page(g->boot, 1) + G5P_R22_TQ, 0x80);
	}
}

/* Jobs >= 2: the legacy job>=2 PM step with policy 0 (pool, list and F rebuilt from the device
 * pool, state S kept), whichever context the job belongs to.
 */
static int g5p_rv3_pm_rebuild(struct g5p_drm *d)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_boot *b = d->core->boot;
	u64 *physical;
	int ret;

	mutex_lock(&rd->lock);
	physical = g5p_rdev_addresses(rd);
	ret = physical ? 0 : -ENOMEM;
	if (!ret)
		ret = g5p_ru_pm_pool(
			g5p_ru_pm_cpu(b), b->render_meta[0].cpu,
			b->render_meta[1].cpu, G5P_SHARED_VA + G5P_SHARED_SIZE,
			G5P_SHARED_VA + G5P_SHARED_SIZE + G5P_GPU_PAGE,
			physical, rd->blocks);
	mutex_unlock(&rd->lock);
	kvfree(physical);
	return ret;
}

/* render-user-loss.h for a phase that did not complete. */
static int g5p_rv3_loss(struct g5p_drm *d, struct g5p_render *r, bool first,
			u32 res, const struct g5p_ru_wait *w)
{
	struct g5p_boot *b = d->core->boot;
	u32 drain_ms = (45000);
	struct g5p_ru_loss_in in = {
		.result = res,
		.first = first,
		.isolate = drain_ms &&
			   g5p_ru_drain_valid(drain_ms, r->hard_ms, r->obs_ms),
		.silent = w->silent,
		.fault = READ_ONCE(b->failed) || READ_ONCE(b->event_failed),
	};

	return g5p_ru_loss(&in);
}

/*
 * The phase on engine `e` of this job timed out with the firmware alive: lose the job and its
 * context, not the device.  No terminal note and no device failure: the job ends -ETIMEDOUT
 * (g5p_render_run() marks the context bad, so its client gets -EIO from now on and its pair and
 * memory stay retained), and the channel is blocked behind the stuck phase until the scheduler's
 * drain (g5p_rv3_drain()) clears it.  RAM only.  Fails (-ENOMEM) only if the record cannot be
 * allocated; the caller then ends the epoch as before.
 */
static int g5p_rv3_lose(struct g5p_drm *d, struct g5p_render *r,
			struct g5p_rv3 *v, u32 seq, u32 e)
{
	struct g5p_rstuck *s = kzalloc(sizeof(*s), GFP_KERNEL);
	const struct g5p_ru_wait *w = e ? &v->td : &v->ta;

	if (!s)
		return -ENOMEM;
	v->lost = true;
	s->r = r;
	s->v = *v;
	s->seq = seq;
	s->engine = e;
	s->drain_ms = (45000);
	s->lost_ns = ktime_get_ns();
	kref_get(&r->ctx->ref);
	d->rdev->stuck = s;
	dev_emerg(
		d->core->dev,
		"RENDER-V3 lost seq=%u engine=%u slots=%u/%u phase_ms=%lld drain_ms=%u: context "
		"lost, device kept, channel blocked end=v3\n",
		seq, e, r->slot[0], r->slot[1],
		g5p_rv3_us(w->start_ns, w->end_ns) / 1000, s->drain_ms);
	return 0;
}

/* A job that stopped the firmware may be recovered from in this boot (core recover.inc,
 * render-user-recover.h).  A refusal is one line: why the epoch ends.
 */
static bool g5p_rv3_recoverable(struct g5p_drm *d, bool first)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_ru_recover_in in = {
		.first = first,
		.retired = g5p_r8_is_retired(),
		.closed = g5p_r8_admit_closed(&g5p_r8_state.admit),
		.recoveries = rd->recoveries,
		.streak = rd->recover_streak,
		.budget = (1),
		.streak_max = (3),
	};

	if (g5p_ru_recoverable(&in))
		return true;
	dev_emerg(
		d->core->dev,
		"RENDER-V3 no-recover why=%s recoveries=%u streak=%u recover=%u streak_max=%u "
		"boot_max=%u first=%u end=v3\n",
		g5p_ru_recover_refusal(&in), in.recoveries, in.streak,
		in.budget, in.streak_max, G5P_RU_RECOVER_BOOT_MAX, first);
	return false;
}

/* The job stopped the firmware (silent stream, a fault): lose it and its context, not the
 * device; the worker recovers the GPU before it picks anything (g5p_rv3_recover()).  RAM only.
 */
static void g5p_rv3_lose_recover(struct g5p_drm *d, struct g5p_render *r,
				 struct g5p_rv3 *v, u32 seq, u32 e,
				 const char *why)
{
	struct g5p_boot *b = d->core->boot;
	struct g5p_fw_recovery rec = b->fw_recovery;
	u32 recs = READ_ONCE(b->fw_recoveries);

	v->lost = true;
	WRITE_ONCE(d->rdev->recovering, true);
	dev_emerg(
		d->core->dev,
		"RENDER-V3 lost seq=%u engine=%u slots=%u/%u why=%s: context lost, device "
		"recovers end=v3\n",
		seq, e, r->slot[0], r->slot[1], why);
	/* The guilty job is the one waited on (one job in flight, core lock); the packet's slot
	 * names the phase the firmware stopped in (render-user-recover.h).  RAM only.
	 */
	if (recs)
		dev_emerg(
			d->core->dev,
			"RENDER-V3 fault-attrib seq=%u engine=%u packet_slot=%u match=%u code=%08x "
			"w20=%08x w24=%08x packets=%u end=v3\n",
			seq, e, rec.slot, rec.slot == e, rec.code, rec.w20,
			rec.w24, recs);
}

/* render_user_test_fault_job: this job's TA reads its control stream (W+0x98, g5p_ru_fill_ta())
 * from the page the context reserved in its VM and never maps, so the GPU faults in that VM.  A
 * RAM edit of the job's own W copy before it is staged; the firmware sees an ordinary job.
 */
static void g5p_rv3_test_fault(struct g5p_diag *g, struct g5p_render *r,
			       struct g5p_rjob *job, u32 seq)
{
	g5p_put64(job->ta_w + 0x98, r->fault_node.start);
	dev_warn(
		g->dev,
		"RENDER-V3 test-fault seq=%u job=%u slots=%u/%u va=%#llx: TA control stream "
		"unmapped end=v3\n",
		seq, r->jobs + 1, r->slot[0], r->slot[1], r->fault_node.start);
}

/* One V3 job: [attach, the epoch's job 1 only | bind, a later context's first job,] TA, 3D.
 * Core lock held.  Returns 0, -ECANCELED (a latched stage ended it), -ETIMEDOUT (a phase ran out
 * its deadline, or the bind's DC3 was not consumed), or a fault.
 */
/* perf60 MMU arm, bits 1/3 (render_user_mmu_inval): before a bound context's job publishes
 * anything, move its slots' host tag to a fresh PC field, so the firmware's tag compare at this
 * job's TA and 3D kicks takes its PC-level invalidate (e138 = (S << 3) | 4) for each slot.  RAM
 * only.  vm->mutation is left alone: an edit on another thread (VM_BIND of a render VM takes no
 * core lock) recomputes its own tag and checks it only against the cached tags, as before.
 */
static void g5p_rv3_mmu_retag(struct g5p_diag *g, struct g5p_render *r, u32 seq,
			      bool switched)
{
	struct g5p_boot *b = g->boot;
	u32 host[2], cached[2], tag;

	dma_rmb();
	for (u32 e = 0; e < 2; e++) {
		host[e] = g5p_le32(g5p_boot_shared(b, 0x84) + r->slot[e] * 4);
		cached[e] = g5p_le32(g5p_boot_shared(b, 0x85) + r->slot[e] * 4);
	}
	tag = g5p_ru_mmu_pc_tag(host[0] ?: cached[0], cached[0], cached[1]);
	if (tag) {
		for (u32 e = 0; e < 2; e++)
			g5p_put32(g5p_boot_shared(b, 0x84) + r->slot[e] * 4,
				  tag);
		dma_wmb();
	}
	g5p_mmu_record(
		g,
		"RENDER-V3 mmu-retag seq=%u slots=%u/%u why=%s host=%08x/%08x cached=%08x/%08x "
		"tag=%08x end=v3\n",
		seq, r->slot[0], r->slot[1], switched ? "switch" : "every",
		host[0], host[1], cached[0], cached[1], tag);
}

/* perf60 A/B: the park that the never-park envelope removed, back around a context switch only.
 * Before a bound context's job that follows another context's job, ctrl+0x1040 = 50 makes the
 * firmware's next pass compute x < 100 (policy state 2: it parks the GPC when idle, as release7);
 * after the window the pin goes back and the job runs in state 0 with its bells as usual.  RAM
 * only: the firmware reloads policy+0x6f4 from ctrl+0x1040 on every pass (perf-envelope.h).
 */



/* perf60: the incoming context's render targets on a switch (lat123_log), RAM. */
static void g5p_rv3_switch_line(struct g5p_diag *g, struct g5p_render *r,
				u32 seq)
{
	char buf[160];
	struct g5p_rt *rt;
	u32 n = 0, len = 0;

	buf[0] = 0;
	list_for_each_entry(rt, &r->targets, link)
		if (n++ < 4)
			len += scnprintf(buf + len, sizeof(buf) - len,
					 " %ux%u/s%u/p%u%s", rt->width,
					 rt->height, rt->samples ?: 1,
					 rt->page_count,
					 rt->destroyed ? "/d" : "");
	dev_info(g->dev,
		 "RENDER-V3 switch seq=%u to=%u/%u jobs=%u rts=%u%s end=v3\n",
		 seq, r->slot[0], r->slot[1], r->jobs, n, buf);
}

/* perf60 per-context PM state (render_user_pm_switch, default 0).  The PM record's selected state
 * S (control+0x80, 0xc0 bytes: the backing pointers a0/a8/b0 the fresh TA loads the PM context
 * from, the lanes TA finish saves and the 3D start restores, the bc accumulator) and the five
 * aux pages it points into (the PM context store) are built once at attach and then shared by
 * every context; the host never rewrites them.  iOS builds one PM record per work queue
 * (6cb2378: S explicit zeros plus the four backing pointers) and keeps it per queue.
 *   1: a context's job after another context's job starts from attach's fresh state (S rebuilt,
 *      aux zeroed);
 *   2: each context keeps its own S + aux: saved after each of its completed jobs, restored when
 *      it follows another context's job, fresh the first time.
 * RAM only, between jobs (the previous job retired, this one has published nothing).
 */

/* perf60 stage 4: (re)build context r's PM set for this epoch: grow its pool to the device's
 * block count, zero its aux pages, build its control + S, list and F (g5p_ru_pm(), as attach
 * builds the device PM) and clean them to PoC.  rd->lock (the pool's size) held.
 */
static int g5p_rv3_pmctx_build(struct g5p_drm *d, struct g5p_render *r, u32 seq,
			       const char *how)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_dma_page *page = &b->pm_set[r->pm_set * G5P_PMSET_PAGES];
	u32 blocks = rd->blocks;
	u64 *physical, list_va, free_va;
	int ret = 0;

	lockdep_assert_held(&rd->lock);
	if (blocks > r->pm_blocks) {
		struct g5p_dma_page *pool =
			kvcalloc(blocks * 8, sizeof(*pool), GFP_KERNEL);

		if (!pool)
			return -ENOMEM;
		if (r->pm_pool)
			memcpy(pool, r->pm_pool,
			       r->pm_blocks * 8 * sizeof(*pool));
		for (u32 i = r->pm_blocks * 8; !ret && i < blocks * 8; i++)
			ret = g5p_vm_alloc_page(d->drm.dev, &pool[i]);
		if (ret) {
			for (u32 i = r->pm_blocks * 8; i < blocks * 8; i++)
				g5p_ru_free_page(d->drm.dev, &pool[i]);
			kvfree(pool);
			return ret;
		}
		kvfree(r->pm_pool);
		r->pm_pool = pool;
		r->pm_blocks = blocks;
	}
	physical = kvcalloc(r->pm_blocks * 8, sizeof(*physical), GFP_KERNEL);
	if (!physical)
		return -ENOMEM;
	for (u32 i = 0; i < r->pm_blocks * 8; i++) {
		physical[i] = r->pm_pool[i].dma;
		memset(r->pm_pool[i].cpu, 0, G5P_GPU_PAGE);
		g5p_table_clean_page(g, &r->pm_pool[i]);
	}
	for (u32 i = 0; i < G5P_RU_AUX_PAGES; i++) {
		struct g5p_dma_page *aux = &g5p_rv3_pm_aux(rd, r)[i];

		memset(aux->cpu, 0, G5P_GPU_PAGE);
		g5p_table_clean_page(g, aux);
	}
	list_va = g5p_pmset_va(r->pm_set, G5P_PMSET_LIST);
	free_va = g5p_pmset_va(r->pm_set, G5P_PMSET_FREE);
	ret = g5p_ru_pm(page[G5P_PMSET_CONTROL].cpu, page[G5P_PMSET_LIST].cpu,
			page[G5P_PMSET_FREE].cpu, list_va, free_va, physical,
			r->pm_blocks, G5P_RU_PRIVATE_VA);
	kvfree(physical);
	for (u32 k = 0; k < G5P_PMSET_PAGES; k++)
		g5p_table_clean_page(g, &page[k]);
	dma_wmb();
	if (!ret)
		r->pm_epoch = rd->recoveries + 1;
	dev_info(
		g->dev,
		"RENDER-V3 pm-ctx seq=%u slots=%u/%u how=%s set=%d id=%u blocks=%u meta=%#x "
		"ret=%d end=v3\n",
		seq, r->slot[0], r->slot[1], how, r->pm_set,
		g5p_rv3_pm_id(rd, r), r->pm_blocks, g5p_rv3_pm_meta(r), ret);
	return ret;
}

/* Before a context's bind (its tables are published by the bind that follows): take a free set,
 * allocate its aux pages, point G5P_RU_PRIVATE_VA in its VM at them, and build the set.  On any
 * failure the context keeps the device PM and the shared aux pages, as before stage 4.
 */
static void g5p_rv3_pmctx_assign(struct g5p_drm *d, struct g5p_render *r,
				 u32 seq)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_boot *b = d->core->boot;
	struct g5p_vm *vm = r->ctx->vm;
	int set, ret = 0;

	if (r->pm_set >= 0)
		return;
	set = g5p_pmset_pick(rd->pm_set_used, b->pm_sets, rd->pm_set_next);
	for (u32 i = 0; set >= 0 && !ret && i < G5P_RU_AUX_PAGES; i++)
		if (!r->pm_aux[i].cpu)
			ret = g5p_vm_alloc_page(d->drm.dev, &r->pm_aux[i]);
	if (set < 0 || ret) {
		dev_warn(
			d->core->dev,
			"RENDER-V3 pm-ctx seq=%u slots=%u/%u how=none set=%d ret=%d: the device PM end=v3\n",
			seq, r->slot[0], r->slot[1], set, ret);
		return;
	}
	r->pm_set = set;
	mutex_lock(&rd->lock);
	ret = g5p_rv3_pmctx_build(d, r, seq, "new");
	mutex_unlock(&rd->lock);
	if (ret) {
		r->pm_set = -1;
		return;
	}
	for (u32 i = 0; i < G5P_RU_AUX_PAGES; i++) {
		u64 va = G5P_RU_PRIVATE_VA + i * G5P_GPU_PAGE;
		u8 *pte = g5p_vm_pte(vm, d->drm.dev, va, false);

		u64 leaf = g5p_leaf_entry(r->pm_aux[i].dma,
					  g5p_r25_attributes(&r->v, va));

		if (!IS_ERR_OR_NULL(pte))
			g5p_vm_entry(&vm->mutation, pte, leaf, 2);
	}
	dma_wmb();
	r->pm_aux_mapped = true;
	rd->pm_set_used[set] = true;
	rd->pm_set_next = (set + 1) % b->pm_sets;
}

/* Before each job of a context with a set: a new epoch or a grown device pool rebuilds the set;
 * without pm_once the pool is rebuilt per job, as g5p_rv3_pm_rebuild() does the device's.
 */
static int g5p_rv3_pmctx_job(struct g5p_drm *d, struct g5p_render *r, u32 seq)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_boot *b = d->core->boot;
	struct g5p_dma_page *page;
	u64 *physical;
	int ret = 0;

	if (r->pm_set < 0)
		return 0;
	page = &b->pm_set[r->pm_set * G5P_PMSET_PAGES];
	mutex_lock(&rd->lock);
	if (r->pm_epoch != rd->recoveries + 1 || r->pm_blocks < rd->blocks) {
		ret = g5p_rv3_pmctx_build(d, r, seq, "rebuild");
	} else if (!rd->pm_once) {
		physical = kvcalloc(r->pm_blocks * 8, sizeof(*physical),
				    GFP_KERNEL);
		ret = physical ? 0 : -ENOMEM;
		for (u32 i = 0; !ret && i < r->pm_blocks * 8; i++)
			physical[i] = r->pm_pool[i].dma;
		if (!ret)
			ret = g5p_ru_pm_pool(
				page[G5P_PMSET_CONTROL].cpu,
				page[G5P_PMSET_LIST].cpu,
				page[G5P_PMSET_FREE].cpu,
				g5p_pmset_va(r->pm_set, G5P_PMSET_LIST),
				g5p_pmset_va(r->pm_set, G5P_PMSET_FREE),
				physical, r->pm_blocks);
		kvfree(physical);
	}
	mutex_unlock(&rd->lock);
	return ret;
}

/* render_user_pm_ctx 3: the context that brought the epoch up (the device PM for its attach job)
 * takes its own set before its next job; the device aux pages its VM already maps become the
 * set's aux (no other context maps them once every later one has its own), so no PTE changes.
 */
static void g5p_rv3_pmctx_take(struct g5p_drm *d, struct g5p_render *r, u32 seq)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_boot *b = d->core->boot;
	int set = g5p_pmset_pick(rd->pm_set_used, b->pm_sets, rd->pm_set_next),
	    ret;

	if (set < 0) {
		r->pm_first = false; /* stays on the device PM; not retried */
		return;
	}
	r->pm_set = set;
	mutex_lock(&rd->lock);
	ret = g5p_rv3_pmctx_build(d, r, seq, "take");
	mutex_unlock(&rd->lock);
	if (ret) {
		r->pm_set = -1;
		r->pm_first = false;
		return;
	}
	rd->pm_set_used[set] = true;
	rd->pm_set_next = (set + 1) % b->pm_sets;
}

/* perf60 stage 4 diagnostics (render_user_fault_read): whose page each PM lane in S points at
 * after the TA saved them: the device pool or aux, a set page, a context's pool or aux.
 */

static void g5p_rv3_pm_lanes(struct g5p_drm *d, struct g5p_render *r, u32 seq)
{
	{
		return;
	}
}

/* The context is gone (g5p_render_release()): its set is free again; its pages go. */
static void g5p_rv3_pmctx_release(struct g5p_drm *d, struct g5p_render *r)
{
	struct g5p_rdev *rd = d->rdev;

	if (r->pm_set >= 0)
		rd->pm_set_used[r->pm_set] = false;
	r->pm_set = -1;
	for (u32 i = 0; r->pm_pool && i < r->pm_blocks * 8; i++)
		g5p_ru_free_page(d->drm.dev, &r->pm_pool[i]);
	kvfree(r->pm_pool);
	r->pm_pool = NULL;
	r->pm_blocks = 0;
	for (u32 i = 0; i < G5P_RU_AUX_PAGES; i++)
		g5p_ru_free_page(d->drm.dev, &r->pm_aux[i]);
}

#define G5P_RV3_PM_S 0x80U
#define G5P_RV3_PM_S_LEN 0xc0U
#define G5P_RV3_PM_SAVE (G5P_RV3_PM_S_LEN + G5P_RU_AUX_PAGES * G5P_GPU_PAGE)

/* perf60 fault address: PM state snapshots (S, PM control, the aux windows S points at, each aux
 * page's head) at three phases of every job, in a ring and per slot pair; printed at a fault read.
 * Only the executor writes them.  RAM only.
 */
enum { G5P_PMSNAP_START, G5P_PMSNAP_TA_DONE, G5P_PMSNAP_END };
struct g5p_pmsnap {
	u64 ns, auxva[3];
	u32 seq, phase, slot, valid;
	u8 s[G5P_RV3_PM_S_LEN], ctl[0x80], win[3][64],
		head[G5P_RU_AUX_PAGES][64];
};
#define G5P_PMSNAP_RING 9U
static struct g5p_pmsnap g5p_pmsnap_ring[G5P_PMSNAP_RING];
static struct g5p_pmsnap g5p_pmsnap_slot[16][3];
static u32 g5p_pmsnap_head;

static void g5p_rv3_pmsnap(struct g5p_drm *d, struct g5p_render *r, u32 seq,
			   u32 phase)
{
	{
		return;
	}
}

static void g5p_rv3_pmsnap_print(struct g5p_diag *g, u32 seq, const char *set,
				 const struct g5p_pmsnap *x)
{
	u64 now = ktime_get_ns();

	if (!x->valid)
		return;
	dev_info(
		g->dev,
		"RENDER-V3 pmsnap seq=%u set=%s job=%u slot=%u phase=%s ago_us=%llu "
		"aux_va=%#llx/%#llx/%#llx end=v3\n",
		seq, set, x->seq, x->slot,
		x->phase == G5P_PMSNAP_START   ? "start" :
		x->phase == G5P_PMSNAP_TA_DONE ? "ta-done" :
						 "end",
		(now - x->ns) / NSEC_PER_USEC, x->auxva[0], x->auxva[1],
		x->auxva[2]);
	for (u32 o = 0; o < sizeof(x->s); o += 64)
		dev_info(g->dev, "RENDER-V3 pmsnap S+%02x %*ph\n", o, 64,
			 x->s + o);
	for (u32 o = 0; o < sizeof(x->ctl); o += 64)
		dev_info(g->dev, "RENDER-V3 pmsnap C+%02x %*ph\n", o, 64,
			 x->ctl + o);
	for (u32 k = 0; k < 3; k++)
		dev_info(g->dev, "RENDER-V3 pmsnap W%u %*ph\n", k, 64,
			 x->win[k]);
	for (u32 i = 0; i < G5P_RU_AUX_PAGES; i++)
		dev_info(g->dev, "RENDER-V3 pmsnap A%u %*ph\n", i, 64,
			 x->head[i]);
}

static void g5p_rv3_pmsnap_dump(struct g5p_diag *g, struct g5p_render *r,
				u32 seq)
{
	for (u32 p = 0; p < 3; p++)
		g5p_rv3_pmsnap_print(g, seq, "slot-last",
				     &g5p_pmsnap_slot[r->slot[0] & 15][p]);
	for (u32 i = G5P_PMSNAP_RING; i; i--)
		g5p_rv3_pmsnap_print(g, seq, "ring",
				     &g5p_pmsnap_ring[(g5p_pmsnap_head - i) %
						      G5P_PMSNAP_RING]);
}

/* perf60 fault address: the VM edit log entries that cover the fault VA (any VM, newest first),
 * then the faulting job's VM's latest edits.  Copied under the lock, printed outside it.
 */
#define G5P_VMLOG_HITS 24U
#define G5P_VMLOG_TAIL 16U
static void g5p_vmlog_dump(struct g5p_diag *g, u32 seq, const void *vm, u64 va,
			   bool have_va)
{
	struct g5p_vmlog_ent *out = kmalloc_array(
		G5P_VMLOG_HITS + G5P_VMLOG_TAIL, sizeof(*out), GFP_KERNEL);
	u32 hits = 0, tail = 0, n, head;
	unsigned long flags;
	u64 now = ktime_get_ns();

	if (!out)
		return;
	spin_lock_irqsave(&g5p_vmlog.lock, flags);
	head = g5p_vmlog.head;
	n = min(head, G5P_VMLOG_N);
	for (u32 i = 1; i <= n; i++) {
		const struct g5p_vmlog_ent *x =
			&g5p_vmlog.e[(head - i) % G5P_VMLOG_N];

		if (have_va && hits < G5P_VMLOG_HITS &&
		    g5p_ru_mmu_fault_in(va, x->va, x->size))
			out[hits++] = *x;
		else if (x->vm == vm && tail < G5P_VMLOG_TAIL)
			out[G5P_VMLOG_HITS + tail++] = *x;
	}
	spin_unlock_irqrestore(&g5p_vmlog.lock, flags);
	dev_info(
		g->dev,
		"RENDER-V3 vmlog seq=%u va=%#llx edits=%u covering=%u vm_tail=%u end=v3\n",
		seq, have_va ? va : 0, head, hits, tail);
	for (u32 i = 0; i < hits + tail; i++) {
		const struct g5p_vmlog_ent *x =
			&out[i < hits ? i : G5P_VMLOG_HITS + i - hits];

		dev_info(
			g->dev,
			"RENDER-V3 vmlog seq=%u %s ago_ms=%llu op=%s pid=%d comm=%s vm=%s/%u va=%#llx "
			"size=%#llx handle=%#x off=%#llx end=v3\n",
			seq, i < hits ? "cover" : "tail",
			(now - x->ns) / NSEC_PER_MSEC,
			g5p_vmlog_ops[min_t(u32, x->op,
					    ARRAY_SIZE(g5p_vmlog_ops) - 1)],
			x->tgid, x->comm, x->vm == vm ? "this" : "other",
			x->vm_id, x->va, x->size, x->handle, x->offset);
	}
	kfree(out);
}

/* perf60 fault address (render_user_fault_read): the MMU fault banks, read once at a failed
 * phase, the way iOS's fault handler (6ca5c14) reads them; the firmware keeps no copy.  Bell mode
 * only.  Under the event lock with the wake path's power proof (the fault already latched
 * b->failed, so that flag is not a guard here), then the GPU register licence (mmio.inc,
 * mmio-access.md §4): no wake (Apple's reads never write R1+0x14, G6), a refusal retried outside
 * the locks every 20-40 us for at most G5P_RV3_DC_SETTLE_US, and the four bank reads only under a
 * held ticket.  A parked GPC records skip-parked, as Apple's report has no bif fault entry then.
 * At most G5P_RV3_FAULT_READS per boot.  It runs before the loss decision and the terminal
 * latch, while the faulted job is still the GPU's current work.
 */
#define G5P_RV3_FAULT_READS G5P_RU_RECOVER_BOOT_MAX
static atomic_t g5p_rv3_fault_reads = ATOMIC_INIT(0);

/* faultrec: a fault VA in the PM-set window (render-user-pmset.h) names a set and its part; say
 * which context owns that set now and how it stands against the two job records and the deferred
 * closes (render_user_close_defer).  RAM only; rdev lock for the context list.
 */
static void g5p_rv3_fault_pmset(struct g5p_drm *d, struct g5p_render *r,
				u32 seq, u32 bank,
				const struct g5p_ru_mmu_fault *f)
{
	static const char *const parts[G5P_PMSET_PAGES] = { "control", "list",
							    "free" };
	struct g5p_rdev *rd = d->rdev;
	struct g5p_render *o, *owner = NULL;
	u32 page, set, own[2] = { U32_MAX, U32_MAX }, df[2][2];
	bool last = false, prev = false, closing = false, bound = false;

	if (f->va < G5P_PMSET_VA || f->va >= G5P_PMSET_END)
		return;
	page = (f->va - G5P_PMSET_VA) / G5P_GPU_PAGE;
	set = page / G5P_PMSET_PAGES;
	mutex_lock(&rd->lock);
	list_for_each_entry(o, &rd->contexts, rdev_link)
		if (o->pm_set == (int)set) {
			owner = o;
			break;
		}
	if (owner) {
		own[0] = owner->slot[0];
		own[1] = owner->slot[1];
		last = rd->pm_last == owner;
		prev = rd->pm_prev == owner;
		closing = owner->closing;
		bound = owner->bound;
	}
	for (u32 k = 0; k < 2; k++) {
		o = rd->close_deferred[k];
		df[k][0] = o ? o->slot[0] : U32_MAX;
		df[k][1] = o ? o->slot[1] : U32_MAX;
	}
	mutex_unlock(&rd->lock);
	dev_emerg(
		d->core->dev,
		"RENDER-V3 fault-pmset seq=%u bank=%u mmu_context=%u set=%u part=%s owner=%d/%d "
		"bound=%u closing=%u last=%u prev=%u job=%u/%u deferred=%d/%d,%d/%d job_slots=%u "
		"end=v3\n",
		seq, bank, f->context, set, parts[page % G5P_PMSET_PAGES],
		(int)own[0], (int)own[1], bound, closing, last, prev,
		r->slot[0], r->slot[1], (int)df[0][0], (int)df[0][1],
		(int)df[1][0], (int)df[1][1], rd->job_slots);
}

static void g5p_rv3_fault_read(struct g5p_drm *d, struct g5p_render *r, u32 seq,
			       u32 e, int res, bool silent)
{
	u32 full = (0), w[4] = { 0 };
	u32 mode = full | ((1) ? 1 : 0);
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_ru_mmu_fault f[2];
	struct g5p_mmio_ticket t = { .r1_0 = U32_MAX,
				     .gpc = U32_MAX,
				     .r1_0_end = U32_MAX,
				     .gpc_end = U32_MAX };
	const char *did = "skip-guard";
	u32 power = 0, tries = 0;
	unsigned long flags;
	bool fault = res == G5P_RU_WAIT_FAULT && !silent, ticket = false;
	u64 t0;
	int ret;

	if ((2) != G5P_WK_ALWAYS || !(mode & (fault ? 1 : 2)))
		return;
	if (atomic_inc_return(&g5p_rv3_fault_reads) > G5P_RV3_FAULT_READS) {
		did = "skip-cap";
		goto print;
	}
	t0 = ktime_get_mono_fast_ns();
	for (;;) {
		spin_lock_irqsave(&b->event_lock, flags);
		power = g5p_wk_power_mask(
			READ_ONCE(g->powered), READ_ONCE(g->fender_ready),
			READ_ONCE(b->released), READ_ONCE(b->alive),
			READ_ONCE(b->control_complete), g5p_rv3_wk_ds108(b),
			READ_ONCE(g->reset_transition));
		if (READ_ONCE(b->stopped) || !g5p_wk_power_ready(power)) {
			spin_unlock_irqrestore(&b->event_lock, flags);
			break;
		}
		tries++;
		ticket = true;
		ret = g5p_mmio_hold(g, &t, 0);
		if (!ret) {
			w[0] = g5p_mmio_r0_read(g, &t, 0xe150);
			w[1] = g5p_mmio_r0_read(g, &t, 0xe154);
			w[2] = g5p_mmio_r0_read(g, &t, 0xe158);
			w[3] = g5p_mmio_r0_read(g, &t, 0xe15c);
			g5p_mmio_release(g, &t);
		}
		spin_unlock_irqrestore(&b->event_lock, flags);
		did = g5p_mmio_did(&t);
		if (ret != -EAGAIN ||
		    ktime_get_mono_fast_ns() - t0 >=
			    (u64)G5P_RV3_DC_SETTLE_US * NSEC_PER_USEC)
			break;
		usleep_range(20, 40);
	}
	if (ticket)
		g5p_mmio_sample_line(g, G5P_MMS_FAULT, &t, w, t.ops ? 4 : 0,
				     true);
print:
	f[0] = g5p_ru_mmu_fault_decode(w[0], w[1]);
	f[1] = g5p_ru_mmu_fault_decode(w[2], w[3]);
	dev_emerg(
		g->dev,
		"RENDER-V3 fault-read seq=%u engine=%u slots=%u/%u why=%s did=%s power=%02x "
		"fence=none r1_0=%08x gpc=%08x tries=%u e150=%08x:%08x e158=%08x:%08x end=v3\n",
		seq, e, r->slot[0], r->slot[1],
		fault  ? "fault" :
		silent ? "silent" :
			 "timeout",
		did, power, t.r1_0, t.gpc, tries, w[1], w[0], w[3], w[2]);
	for (u32 k = 0; k < 2; k++)
		if (f[k].valid)
			dev_emerg(
				g->dev,
				"RENDER-V3 fault-decode seq=%u bank=%u va=%#llx %s type=%s level=%u "
				"requester=%u/%s sideband=%#x context=%u job_slots=%u/%u end=v3\n",
				seq, k, f[k].va, f[k].read ? "read" : "write",
				g5p_ru_mmu_fault_types[f[k].type], f[k].level,
				f[k].requester,
				g5p_ru_mmu_requester(f[k].requester),
				f[k].sideband, f[k].context, r->slot[0],
				r->slot[1]);
	for (u32 k = 0; k < 2; k++)
		if (f[k].valid)
			g5p_rv3_fault_pmset(d, r, seq, k, &f[k]);
	g5p_rv3_switch_line(g, r, seq);
	if (!full)
		return;
	g5p_vmlog_dump(g, seq, r->ctx->vm, f[0].valid ? f[0].va : f[1].va,
		       f[0].valid || f[1].valid);
	g5p_rv3_pmsnap(d, r, seq, G5P_PMSNAP_END);
	g5p_rv3_pmsnap_dump(g, r, seq);
}

/* GPU overclock thermal stop (core boot.inc, boot_gpu_oc_mhz): every 500 ms while the epoch runs
 * state 5, the three SoC zones; at or above boot_gpu_oc_temp_mc (or unreadable) the overclock is off
 * for the rest of the boot and the render scheduler recovers the GPU (g5p_rv3_recover(): gate cycle,
 * firmware boot) between jobs, so the new epoch powers on at state 4 with record 5 cleared: iOS's
 * park -> reboot -> new pstate, never a pstate change under a live firmware.  Every 10 s one status
 * line: the temperatures, the real Fender state (an R1-only licence ticket) and the jobs run.
 */
#define G5P_OC_POLL_MS 500
static void g5p_rsched_kick(struct g5p_drm *d);

static void g5p_oc_work(struct work_struct *work)
{
	struct g5p_rdev *rd =
		container_of(to_delayed_work(work), struct g5p_rdev, oc_work);
	struct g5p_drm *d = rd->drm;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = READ_ONCE(g->boot);
	u32 lim = (90000), zones;
	const char *zone;
	int t;

	if (!b || !0 || READ_ONCE(g5p_oc_tripped)) {
		clear_bit(0, &rd->oc_armed);
		return;
	}
	zones = g5p_oc_temp(&t, &zone);
	if (zones != ARRAY_SIZE(g5p_oc_zone_names) || t >= (int)lim) {
		WRITE_ONCE(g5p_oc_tripped, true);
		dev_emerg(
			g->dev,
			"GPUPERF oc thermal-stop temp=%d zone=%s zones=%u limit=%u mhz=%u jobs=%u: the "
			"GPU recovers at state 4 (723 MHz) end=gpuperf\n",
			t, zone, zones, lim, b->gpu_oc_mhz, rd->oc_jobs);
		WRITE_ONCE(rd->recovering, true);
		g5p_rsched_kick(d);
		clear_bit(0, &rd->oc_armed);
		return;
	}
	if (!(++rd->oc_ticks % (10000 / G5P_OC_POLL_MS))) {
		struct g5p_mmio_ticket tk;

		if (!g5p_mmio_hold(g, &tk, G5P_MMIO_CENSUS))
			g5p_mmio_release(g, &tk);
		dev_info(
			g->dev,
			"GPUPERF oc status mhz=%u temp=%d zone=%s limit=%u r1=%s r1_1000=%08x jobs=%u "
			"end=gpuperf\n",
			b->gpu_oc_mhz, t, zone, lim, g5p_mmio_did(&tk), tk.gpc,
			rd->oc_jobs);
	}
	queue_delayed_work(system_wq, &rd->oc_work,
			   msecs_to_jiffies(G5P_OC_POLL_MS));
}



static int g5p_rv3_execute(struct g5p_drm *d, struct g5p_render *r,
			   struct g5p_rjob *job)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_rv3 *v = &job->rv3;
	struct g5p_ru_v3_state st = g5p_rv3_state(g, r);
	bool first = !rd->up, bound_before = r->bound;
	int err = -EIO, ret, why, loss;
	u32 seq, res;

	if (r->stage_done)
		return -ECANCELED;
	g5p_l123_job_reset(g);
	/* The submit-time admission again, now under the core lock and with the bind state: still nothing touched. */
	why = g5p_ru_v3_admit(&st, true);
	if (why)
		return g5p_rv3_refusal_errno(why);
	seq = rd->sequence + 1;
	;
	0;
	if (bound_before && d->bell_prev_ctx && d->bell_prev_ctx != r->ctx) {
		u32 ms = (0);

		;
		;
		if (ms && b->gpu_perf)
			0;
	}
	0;
	if (g5p_ru_v3_verbose(r->jobs + 1, 0))
		g5p_job_printk(
			g->dev,
			"RENDER-V3 start seq=%u bound_ms=%u cap_ms=%u slots=%u/%u attach=%s "
			"ta_doorbell=%s 3d_doorbell=none wait=ram-only after_timeout=quiet end=v3\n",
			seq, r->wait_ms, G5P_RU_V3_CAP_MS, r->slot[1],
			r->slot[0],
			first		? "firmware-dc3" :
			r->bound	? "kept" :
			rd->spare_count ? "spare" :
					  "ram-dc3",
			first ? "grace-owned" : "none");
	if (first) {
		v->phase = G5P_RV3_ATTACH;
		ret = g5p_rv3_spares(d, v);
		if (!ret)
			ret = g5p_rv3_attach(d, r, v);
		/* perf60 stage 4: a context with a set that brings a new epoch up */
		if (!ret)
			ret = g5p_rv3_pmctx_job(d, r, seq);
		if (ret)
			goto out;
		;
	} else {
		if (!r->bound) {
			/* perf60 stage 4: its own PM set and aux pages before the bind
			 * publishes its tables
			 */
			if (rd->pm_ctx)
				g5p_rv3_pmctx_assign(d, r, seq);
			/* RAM only; a failure is this context's alone (v->bind_failed). */
			v->phase = G5P_RV3_BIND;
			ret = rd->spare_count ? g5p_rv3_adopt(d, r, v) :
						g5p_rv3_bind(d, r, v);
			if (ret)
				goto out;
			;
		}
		/* The firmware owns the rings and the channels from here on: any failure quarantines. */
		v->touched = true;
		if (rd->pm_ctx == 3 && r->pm_first && r->pm_set < 0)
			g5p_rv3_pmctx_take(d, r, seq);
		if (r->pm_set >= 0)
			ret = g5p_rv3_pmctx_job(d, r, seq);
		else
			ret = rd->pm_once ? 0 : g5p_rv3_pm_rebuild(d);
		if (ret)
			goto out;
		;
		;
		g5p_rv3_pmsnap(d, r, seq, G5P_PMSNAP_START);
		if (g5p_ru_mmu_retag_due((0), bound_before,
					 d->bell_prev_ctx != r->ctx))
			g5p_rv3_mmu_retag(g, r, seq,
					  d->bell_prev_ctx != r->ctx);
	}
	rd->pm_serial = g5p_le32(g5p_rv3_pm_cpu(b, r) + 0x80);
	if (g5p_ru_fault_due(r->fault_job, r->jobs, first))
		g5p_rv3_test_fault(g, r, job, seq);
	for (u32 e = 0; e < 2; e++) {
		g5p_ru_job_at(
			g5p_ru_page(b, e), g5p_ru_meta(e),
			g5p_meta_address(g5p_object_va(&b->layout, 0x51 + e)),
			g5p_rv3_pm_meta(r), r->slot[e], e, seq,
			e ? job->td_w : job->ta_w,
			g5p_ru_meta(0) + G5P_R22_STAMP + 4, 0,
			g5p_rv3_js(rd, seq), g5p_rv3_pm_id(rd, r));
		r->submitted[e] = false;
		r->consumed[e] = false;
		r->retired[e] = false;
	}
	g5p_l123_job_prepare(d, job);
	g5p_render_clean_job(g);
	rd->sequence = seq;
	v->seq = seq;
	r->jobs++;
	v->phase = G5P_RV3_TA_PUBLISH;
	g5p_rv3_event_arm(d, 0, 0);
	g5p_rv3_wake_mark(d, r, v, 0);
	ret = g5p_rv3_publish_ta(g, r, v, seq);
	if (ret)
		goto out;
	v->phase = G5P_RV3_TA_WAIT;
	if (r->stall_job && r->jobs == r->stall_job && !first) {
		/* render_user_test_stall_job: the host gives up, the firmware runs on */
		g5p_ru_wait_start(&v->ta, v->ta_pub_ns, 1, v->job_deadline_ns);
		dev_warn(
			g->dev,
			"RENDER-V3 test-stall seq=%u job=%u slots=%u/%u: TA bound 1 ms end=v3\n",
			seq, r->jobs, r->slot[0], r->slot[1]);
	} else {
		g5p_rv3_wait_start(g, r, &v->ta, v->ta_pub_ns,
				   v->job_deadline_ns);
	}
	g5p_ru_touch_start(&v->touch[0], v->ta_pub_ns, r->touch_ms, seq);
	g5p_ru_ta_start(&v->tr);
	res = g5p_rv3_wait(g, r, 0, seq, &v->ta, &v->touch[0], NULL, &v->tr,
			   &v->last[0], &v->ta_consumed_ns, &err);
	r->consumed[0] = v->ta_consumed_ns != 0;
	if (res != G5P_RU_WAIT_COMPLETE) {
		struct g5p_rv3_ev late;

		/* Timeout or fault: (latency101: the RAM-only observation window first,) terminal
		 * latch, one RAM sample for the record (a completion that arrives now is evidence,
		 * never the result), no export, no register access.
		 */
		g5p_rv3_fault_read(d, r, seq, 0, res, v->ta.silent);
		if (res == G5P_RU_WAIT_TIMEOUT && r->obs_ms)
			g5p_rv3_observe(g, r, 0, seq, &v->ta, NULL, &v->tr,
					&late);
		loss = g5p_rv3_loss(d, r, first, res, &v->ta);
		if (loss == G5P_RU_LOSS_CONTEXT &&
		    !g5p_rv3_lose(d, r, v, seq, 0)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		if (loss == G5P_RU_LOSS_DEVICE &&
		    g5p_rv3_recoverable(d, first)) {
			g5p_rv3_lose_recover(d, r, v, seq, 0,
					     v->ta.silent ? "ta-silent" :
					     res == G5P_RU_WAIT_TIMEOUT ?
							    "ta-timeout" :
							    "ta-fault");
			ret = g5p_ru_lost_errno(res, err);
			goto out;
		}
		g5p_rv3_terminal(
			g,
			res == G5P_RU_WAIT_TIMEOUT ? G5P_RU_TERM_TIMEOUT :
						     G5P_RU_TERM_FAULT,
			res == G5P_RU_WAIT_TIMEOUT ? "ta-timeout" : "ta-fault");
		if (!(res == G5P_RU_WAIT_TIMEOUT && r->obs_ms))
			g5p_rv3_observe(g, r, 0, seq, &v->ta, NULL, &v->tr,
					&late);
		ret = res == G5P_RU_WAIT_TIMEOUT ? -ETIMEDOUT : err;
		if (first)
			g5p_r74_mark(g, G5P_R74_TA_RAM_TIMEOUT, ret);
		goto out;
	}
	g5p_rv3_ta_count(g, d, v);
	r->retired[0] = true;
	g5p_rv3_terminal(g, G5P_RU_TERM_COMPLETE, "ta-complete");
	g5p_rv3_pmsnap(d, r, seq, G5P_PMSNAP_TA_DONE);
	g5p_rv3_pm_lanes(d, r, seq);
	if (first)
		g5p_r74_mark(g, G5P_R74_TA_RAM_COMPLETE,
			     g5p_le32(g5p_ru_page(b, 0) + G5P_R22_STAMP + 4));
	;
	v->phase = G5P_RV3_TD_PUBLISH;
	g5p_rv3_event_arm(d, 1, g5p_ru_endpoint(seq));
	g5p_rv3_wake_mark(d, r, v, 1);
	ret = g5p_rv3_publish_3d(g, r, v, seq);
	if (ret) {
		g5p_rv3_sample(
			g, r, 1, seq,
			&v->last[1]); /* RAM only: the receipt prints the real 3D ring and stamps */
		goto out;
	}
	v->phase = G5P_RV3_TD_WAIT;
	g5p_rv3_wait_start(g, r, &v->td, v->td_pub_ns, v->job_deadline_ns);
	g5p_ru_touch_start(&v->touch[1], v->td_pub_ns, r->touch_ms, seq);
	g5p_ru_rt_start(&v->rt, r->settle_ms);
	res = g5p_rv3_wait(g, r, 1, seq, &v->td, &v->touch[1], &v->rt, NULL,
			   &v->last[1], &v->td_consumed_ns, &err);
	r->consumed[1] = v->td_consumed_ns != 0;
	/* mmio_selftest bit 2 (rung 5, DFU-risk on a park tuple): one licensed ticket at the 3D
	 * completion decision; always: the idle clock of bit 3.
	 */
	;
	if (res != G5P_RU_WAIT_COMPLETE) {
		struct g5p_rv3_ev late;

		g5p_rv3_fault_read(d, r, seq, 1, res, v->td.silent);
		if (res == G5P_RU_WAIT_TIMEOUT && r->obs_ms)
			g5p_rv3_observe(g, r, 1, seq, &v->td, &v->rt, NULL,
					&late);
		loss = g5p_rv3_loss(d, r, first, res, &v->td);
		if (loss == G5P_RU_LOSS_CONTEXT &&
		    !g5p_rv3_lose(d, r, v, seq, 1)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		if (loss == G5P_RU_LOSS_DEVICE &&
		    g5p_rv3_recoverable(d, first)) {
			g5p_rv3_lose_recover(d, r, v, seq, 1,
					     v->td.silent ? "3d-silent" :
					     res == G5P_RU_WAIT_TIMEOUT ?
							    "3d-timeout" :
							    "3d-fault");
			ret = g5p_ru_lost_errno(res, err);
			goto out;
		}
		g5p_rv3_terminal(
			g,
			res == G5P_RU_WAIT_TIMEOUT ? G5P_RU_TERM_TIMEOUT :
						     G5P_RU_TERM_FAULT,
			res == G5P_RU_WAIT_TIMEOUT ? "3d-timeout" : "3d-fault");
		if (!(res == G5P_RU_WAIT_TIMEOUT && r->obs_ms))
			g5p_rv3_observe(g, r, 1, seq, &v->td, &v->rt, NULL,
					&late);
		ret = res == G5P_RU_WAIT_TIMEOUT ? -ETIMEDOUT : err;
		goto out;
	}
	/* Rule (c) decided: the matching event was not seen.  Always one line (RAM only). */
	if (v->rt.decided == G5P_RU_RT_HOLD)
		dev_warn(
			g->dev,
			"RENDER-V3 3d-hold seq=%u idle=%08x,%08x,%08x,%08x,%08x,%08x coherent_ms=%lld "
			"decided_ms=%lld end=v3\n",
			seq, v->last[1].idle[0], v->last[1].idle[1],
			v->last[1].idle[2], v->last[1].idle[3],
			v->last[1].idle[4], v->last[1].idle[5],
			g5p_ru_rt_ms(v->td_pub_ns, v->rt.coherent_ns),
			g5p_ru_rt_ms(v->td_pub_ns, v->td.end_ns));
	r->retired[1] = true;
	v->phase = G5P_RV3_DONE;
	rd->recover_streak =
		0; /* a job finished: a later fault starts a new streak */
	ret = 0;
out:
	if (v->phase >= G5P_RV3_TA_PUBLISH)
		v->ta_event_ns = g5p_rv3_event_disarm(d, 0);
	if (v->phase >= G5P_RV3_TD_PUBLISH)
		v->td_event_ns = g5p_rv3_event_disarm(d, 1);
	g5p_rv3_wake_end(d, r, v);
	/* Any other failure after the hardware was touched (attach, publish, drain, protocol): the same
	 * criterion g5p_render_run() quarantines the context by.  A stage stop (-ECANCELED) is not one.
	 */
	if (ret < 0 && ret != -ECANCELED && v->touched && !v->lost &&
	    !g5p_ru_term_failed(&g5p_rv3_term))
		g5p_rv3_terminal(g, g5p_ru_term_from_ret(ret),
				 g5p_rv3_phase_names[min_t(
					 u32, v->phase, G5P_RV3_PHASES - 1)]);
	g5p_rv3_receipt(g, r, job, ret);
	g5p_l123_job_end(d, job, seq, ret);
	if (!ret) {
		g5p_rv3_pmsnap(d, r, seq, G5P_PMSNAP_END);
		;
		;
	}
	/* render-user-pmset.h: the PM's write-back targets */
	if (g5p_pmset_switched(rd->pm_last, r)) {
		rd->pm_prev = rd->pm_last;
		rd->pm_last = r;
	}
	/* perf60 bell gates: the next job's gap and context switch are measured from here. */
	d->bell_prev_end_ns = ktime_get_ns();
	d->bell_prev_ctx = r->ctx;
	return ret;
}

/*
 * The channel is blocked behind a lost job (g5p_rv3_lose()): wait for its stuck phase, RAM only,
 * with the firmware's event stream watched from the first sample (silent for 1 s = dead).  A late
 * TA gets the job's 3D published (its W is staged since the job's start; no doorbell for a job
 * other than the epoch's first), so the 3D channel's sequence and the PM state stay as the
 * firmware expects; a late 3D clears the channel.  Then the held jobs run.  A silent stream, a
 * fault, a refused 3D or the drain bound end the epoch as a timeout or fault always did, and the
 * held jobs then fail at admission.  Called by the scheduler's worker before any pick; returns
 * false, having done nothing, when the core lock is busy.
 */
static bool g5p_rv3_drain(struct g5p_drm *d)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_rstuck *s = rd->stuck;
	struct g5p_render *r = s->r;
	struct g5p_rv3 *v = &s->v;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	unsigned long deadline = jiffies + 2 * HZ;
	u32 st = s->engine ? G5P_RU_DRAIN_TD : G5P_RU_DRAIN_TA,
	    res = G5P_RU_WAIT_PENDING;
	struct g5p_ru_wait w = {};
	bool armed = false;
	int act, err = -EIO;

	while (!mutex_trylock(&g->lock)) {
		if (time_after(jiffies, deadline))
			return false;
		msleep(5);
	}
	WRITE_ONCE(rd->executing, true);
	for (;;) {
		u32 e = st == G5P_RU_DRAIN_TA ? 0 : 1;
		u64 now = ktime_get_ns();

		g5p_ru_wait_start_lat(&w, now, 0, s->drain_ms,
				      now + (u64)s->drain_ms * NSEC_PER_MSEC,
				      READ_ONCE(b->events));
		if (READ_ONCE(b->failed) || READ_ONCE(b->stopped)) {
			res = G5P_RU_WAIT_FAULT;
		} else if (!e) {
			g5p_rv3_event_arm(d, 0, 0);
			g5p_ru_ta_start(&v->tr);
			res = g5p_rv3_wait(g, r, 0, s->seq, &w, &v->touch[0],
					   NULL, &v->tr, &v->last[0],
					   &v->ta_consumed_ns, &err);
			v->ta_event_ns = g5p_rv3_event_disarm(d, 0);
		} else {
			if (!armed)
				g5p_rv3_event_arm(d, 1,
						  g5p_ru_endpoint(s->seq));
			g5p_ru_rt_start(&v->rt, r->settle_ms);
			res = g5p_rv3_wait(g, r, 1, s->seq, &w, &v->touch[1],
					   &v->rt, NULL, &v->last[1],
					   &v->td_consumed_ns, &err);
			v->td_event_ns = g5p_rv3_event_disarm(d, 1);
		}
		act = g5p_ru_drain_step(&st, res);
		if (act != G5P_RU_DRAIN_PUBLISH_TD)
			break;
		r->retired[0] = true;
		g5p_rv3_event_arm(d, 1, g5p_ru_endpoint(s->seq));
		armed = true;
		if (g5p_rv3_publish_3d(g, r, v, s->seq)) {
			g5p_rv3_event_disarm(d, 1);
			res = G5P_RU_WAIT_FAULT;
			act = g5p_ru_drain_step(&st, res);
			break;
		}
	}
	if (act == G5P_RU_DRAIN_UNBLOCK) {
		r->retired[1] = true;
		r->lost_drained = true;
		dev_emerg(
			g->dev,
			"RENDER-V3 drained seq=%u engine=%u slots=%u/%u lost_ms=%lld: channel clear, "
			"held jobs resume end=v3\n",
			s->seq, s->engine, r->slot[0], r->slot[1],
			g5p_rv3_us(s->lost_ns, ktime_get_ns()) / 1000);
	} else {
		bool fault = res == G5P_RU_WAIT_FAULT;

		dev_emerg(
			g->dev,
			"RENDER-V3 drain dead seq=%u engine=%u phase=%s why=%s err=%d lost_ms=%lld: the "
			"device recovers if it may, else the epoch ends end=v3\n",
			s->seq, s->engine,
			st == G5P_RU_DRAIN_DEAD && !v->td_pub_ns ? "ta" : "3d",
			w.silent ? "silent" :
			fault	 ? "fault" :
				   "timeout",
			err, g5p_rv3_us(s->lost_ns, ktime_get_ns()) / 1000);
		if (g5p_rv3_recoverable(d, false))
			WRITE_ONCE(rd->recovering, true);
		else
			g5p_rv3_terminal(
				g,
				fault ? G5P_RU_TERM_FAULT : G5P_RU_TERM_TIMEOUT,
				fault ? "drain-fault" : "drain-timeout");
	}
	/* after the ruling: `recovering` or the terminal latch is visible first (g5p_rdev_ruling()) */
	smp_store_release(&rd->executing, false);
	rd->stuck = NULL;
	mutex_unlock(&g->lock);
	g5p_context_put(r->ctx);
	kfree(s);
	return true;
}

/* The firmware window's leaves again from the new epoch's tables, in the same pages, so every
 * render VM's root entry stays valid (g5p_render_fw_entry()).  Core lock held.
 */
static void g5p_render_fw_rebuild(struct g5p_drm *d)
{
	struct g5p_rwin *w = &d->rdev->window;
	struct g5p_boot *b = d->core->boot;
	u8 *leaves[ARRAY_SIZE(b->leaf)];

	if (!w->built)
		return;
	for (u32 i = 0; i < b->leaves; i++)
		leaves[i] = b->leaf[i].cpu;
	w->pages = 0;
	w->missing = 0;
	for (u32 i = 0; i < G5P_RU_FW_PAGES; i++) {
		u64 va = g5p_ru_fw_window_va[i];
		u64 leaf = g5p_ru_fw_leaf(leaves, b->leaf_index, b->leaves, va);
		u32 k = g5p_ru_fw_slot(w->leaf_di, w->leaves, va);

		if (k == w->leaves)
			continue;
		g5p_put64(w->leaf[k].cpu + g5p_ru_fw_ti(va) * 8,
			  leaf & 1 ? leaf : 0);
		w->pages += leaf & 1;
		w->missing += !(leaf & 1);
	}
	for (u32 i = 0; i < w->leaves; i++)
		g5p_table_clean_page(d->core, &w->leaf[i]);
}

/*
 * A lost job stopped the firmware (g5p_rv3_lose_recover(), or a drain that died): cycle the GPU
 * and boot the firmware again (core recover.inc), then start the render device over as at boot.
 * Every context survives except the lost one: its bindings belonged to the old epoch, which the
 * cycle ended, so each is unbound in RAM, its binding's reference dropped, and its next job binds
 * again (the first job after the recovery runs the epoch's attach).  The aux pages and the PM
 * pool are zeroed as at their allocation.  The held jobs run after this.  Returns false, having
 * done nothing, when the core lock is busy.  A failed recovery leaves the device failed.
 */
static bool g5p_rv3_recover(struct g5p_drm *d)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	unsigned long deadline = jiffies + 2 * HZ, flags;
	struct g5p_render **drop = NULL;
	struct g5p_render *r;
	u32 n = 0, count = 0, dropped = 0, stage;
	int ret;

	while (!mutex_trylock(&g->lock)) {
		if (time_after(jiffies, deadline))
			return false;
		msleep(5);
	}
	/* The old epoch's bindings are void from here: no edit may publish a tag into it. */
	mutex_lock(&rd->lock);
	list_for_each_entry(r, &rd->contexts, rdev_link) {
		WRITE_ONCE(r->bound, false);
		count++;
	}
	mutex_unlock(&rd->lock);
	drop = kcalloc(max(count, 1U), sizeof(*drop), GFP_KERNEL);
	rd->recoveries++;
	rd->recover_streak++;
	ret = drop ? g5p_recover(g, "lost-job", &stage) : -ENOMEM;
	if (ret) {
		g5p_rv3_terminal(g, G5P_RU_TERM_FAULT, "recover-failed");
		WRITE_ONCE(rd->recovering, false);
		mutex_unlock(&g->lock);
		kfree(drop);
		dev_emerg(
			g->dev,
			"RENDER-V3 recover failed ret=%d n=%u: the epoch ends end=v3\n",
			ret, rd->recoveries);
		return true;
	}
	g5p_ru_term_reset(&g5p_rv3_term);
	/* the new firmware's PM has written nothing back yet: no context is its last or previous */
	rd->pm_last = NULL;
	rd->pm_prev = NULL;
	rd->up = false;
	rd->v3 = false;
	rd->sequence = 0;
	rd->pm_serial = 0;
	rd->dc_pending = false;
	rd->dc_wait = 0;
	if (rd->spares) {
		spin_lock_irqsave(&g->slot_lock, flags);
		for (u32 i = 0; i < rd->spare_count; i++)
			if (!rd->spares[i].used) {
				__clear_bit(rd->spares[i].slot[0],
					    g->user_slots);
				__clear_bit(rd->spares[i].slot[1],
					    g->user_slots);
			}
		spin_unlock_irqrestore(&g->slot_lock, flags);
		for (u32 i = 0; i < rd->spare_count; i++)
			g5p_ru_free_page(d->drm.dev, &rd->spares[i].root);
		kfree(rd->spares);
		rd->spares = NULL;
		rd->spare_count = 0;
	}
	g5p_render_fw_rebuild(d);
	mutex_lock(&rd->lock);
	for (u32 i = 0; i < G5P_RU_AUX_PAGES; i++)
		if (rd->aux[i].cpu) {
			memset(rd->aux[i].cpu, 0, G5P_GPU_PAGE);
			g5p_table_clean_page(g, &rd->aux[i]);
		}
	for (u32 i = 0; i < rd->blocks * 8; i++) {
		memset(rd->physical[i].cpu, 0, G5P_GPU_PAGE);
		g5p_table_clean_page(g, &rd->physical[i]);
	}
	list_for_each_entry(r, &rd->contexts, rdev_link) {
		r->jobs = 0;
		r->dc_target = 0;
		WRITE_ONCE(r->live_maps, false);
		memset(r->submitted, 0, sizeof(r->submitted));
		memset(r->consumed, 0, sizeof(r->consumed));
		memset(r->retired, 0, sizeof(r->retired));
		/* nothing of the old epoch holds this context's memory any more */
		r->lost_drained = true;
		WRITE_ONCE(r->ctx->live, false);
		if (r->retained && n < count) {
			r->retained = false;
			drop[n++] = r;
		}
	}
	mutex_unlock(&rd->lock);
	spin_lock_irqsave(&d->status_lock, flags);
	d->last_error = 0;
	d->job_error = 0;
	spin_unlock_irqrestore(&d->status_lock, flags);
	WRITE_ONCE(rd->recovering, false);
	mutex_unlock(&g->lock);
	for (u32 i = 0; i < n; i++) {
		struct g5p_context *c = drop[i]->ctx;

		dropped++;
		g5p_context_put(c);
		drm_dev_put(&d->drm);
	}
	kfree(drop);
	dev_emerg(
		g->dev,
		"RENDER-V3 recovered n=%u contexts=%u unbound=%u streak=%u: held jobs resume, "
		"the next job attaches end=v3\n",
		rd->recoveries, count, dropped, rd->recover_streak);
	return true;
}

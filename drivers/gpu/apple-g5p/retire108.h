/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RETIRE108_H
#define G5P_RETIRE108_H

#include "bits-compat.h"

/*
 * G5P outgoing retire layer (series gpu-recovery108, patch 0006).
 *
 * Design: docs/gpu/experiments/recovery108-driver.md section 3.
 *
 * Wraps the recovery98 teardown ladder (teardown.h, g5p_td_run()) without
 * changing it: the TD98 lines of a run are byte for byte the ones of the
 * same run without this layer.  It adds, around a run:
 *
 *   ADMIT     atomically close job admission (one cmpxchg word), then a
 *             bounded RAM-only wait for submitters already inside the
 *             admission window; still inside at the deadline = explicit
 *             ADMIT_OPEN flag, never a block.
 *   CLASSIFY  the epoch class from RAM inputs only (struct g5p_r8_in).
 *   RECEIPT   one TR108 line with the class and the generation, printed
 *             before the ladder makes its first AIC or R1 access.
 *   (ladder)  g5p_td_run(FINAL) through a callback, unchanged.
 *   SLOTS     three named post-ladder slots.  Each runs only with a glue op
 *             present AND its own parameter on; every skip is named:
 *               fw-sleep    validated firmware sleep before stable-off (G2:
 *                           no recovered command; the glue has no op)
 *               stable-off  one R1 read of isStableOff after a verified
 *                           Fender stop leg (G6: default off)
 *               pmgr-gate   outgoing PMGR power gate (G1: not implemented,
 *                           there is no op and no parameter)
 *   RETIRE    the retired latch: from here on g5p_r8_mmio_ok() refuses R0,
 *             R1 and every write except the always-on AIC ack of a late
 *             handler entry.  The latch is set whatever the verdict.
 *
 * Generations: gen counts real runs (CAPTURE and FINAL) since boot and is
 * monotonic; a REENTRY (a second FINAL after retire, e.g. notifier then
 * .shutdown then remove) prints a line but does not advance it.
 *
 * Pure policy, like teardown.h: no register access, no kernel API except
 * cmpxchg(), READ_ONCE()/WRITE_ONCE(), scnprintf() and the errno names.  The
 * same file is compiled by the kernel glue (teardown.inc) and by the offline
 * tests (patches/gpu-recovery108/tests).  Needs teardown.h first.
 */

#define G5P_R8_CLOSED 0x80000000u /* admission word: closed bit */
#define G5P_R8_INSIDE 0x7fffffffu /* admission word: submitters inside */
#define G5P_R8_POLL_US 100
#define G5P_R8_MAX_STEPS 512 /* ADMIT wait cap, independent of the clock */
#define G5P_R8_ADMIT_US 20000 /* ADMIT wait budget */

/* Admission: one word, closed bit + count of submitters inside the window. */
struct g5p_r8_admit {
	u32 word;
};

/* A submitter enters before its first check of driver state and leaves on
 * every return path.  False once admission is closed: the submit must fail
 * with -ESHUTDOWN and touch nothing.
 */
static inline bool g5p_r8_admit_enter(struct g5p_r8_admit *a)
{
	u32 cur = READ_ONCE(a->word), old;

	for (;;) {
		if ((cur & G5P_R8_CLOSED) ||
		    (cur & G5P_R8_INSIDE) == G5P_R8_INSIDE)
			return false;
		old = cmpxchg(&a->word, cur, cur + 1);
		if (old == cur)
			return true;
		cur = old;
	}
}

static inline void g5p_r8_admit_exit(struct g5p_r8_admit *a)
{
	u32 cur = READ_ONCE(a->word), old;

	for (;;) {
		if (!(cur & G5P_R8_INSIDE))
			return; /* unbalanced exit: never wrap into the closed bit */
		old = cmpxchg(&a->word, cur, cur - 1);
		if (old == cur)
			return;
		cur = old;
	}
}

/* Close (idempotent).  Returns the submitters inside at the instant of closing. */
static inline u32 g5p_r8_admit_close(struct g5p_r8_admit *a)
{
	u32 cur = READ_ONCE(a->word), old;

	for (;;) {
		old = cmpxchg(&a->word, cur, cur | G5P_R8_CLOSED);
		if (old == cur)
			return cur & G5P_R8_INSIDE;
		cur = old;
	}
}

static inline u32 g5p_r8_admit_inside(const struct g5p_r8_admit *a)
{
	return READ_ONCE(a->word) & G5P_R8_INSIDE;
}

static inline bool g5p_r8_admit_closed(const struct g5p_r8_admit *a)
{
	return READ_ONCE(a->word) & G5P_R8_CLOSED;
}

/* Epoch classes.  The dirty ones are never reported as clean (H10). */
enum g5p_r8_class {
	G5P_R8C_NEVER_BOOTED, /* shared pages never mapped */
	G5P_R8C_UNRELEASED, /* mapped, firmware never released */
	G5P_R8C_IDLE, /* released and ALIVE, no context ever claimed (not-run) */
	G5P_R8C_CLEAN, /* claimed, every job signalled, no holder left */
	G5P_R8C_HELD, /* clean, but a process still holds the claimed context */
	G5P_R8C_PARKED, /* GPC=0 already verified by a park */
	G5P_R8C_IN_FLIGHT, /* a job accepted and not signalled, or a worker inside a wait */
	G5P_R8C_TIMEOUT, /* terminal latch = timeout (a late completion stays a timeout) */
	G5P_R8C_FAULT, /* failed, event failure or a fault terminal */
	G5P_R8C_ABORTED, /* bootstrap power abort latched (the r87b/C3 epoch) */
	G5P_R8C_UNKNOWN, /* released, never ALIVE; or an unverified earlier stop */
	G5P_R8C_N,
};

static const char *const g5p_r8_class_name[G5P_R8C_N] = {
	"never-booted", "unreleased", "idle",  "clean",	  "held",    "parked",
	"in-flight",	"timeout",    "fault", "aborted", "unknown",
};

static inline bool g5p_r8_class_dirty(enum g5p_r8_class c)
{
	return c >= G5P_R8C_IN_FLIGHT;
}

/* RAM-only inputs, read by the glue before the run (never from a register). */
struct g5p_r8_in {
	bool booted; /* b->maps_ready */
	bool released; /* b->released */
	bool alive; /* b->alive before FREEZE, or ALIVE was proven this boot */
	bool failed; /* b->failed */
	bool fault_event; /* b->event_failed */
	int terminal; /* teardown terminal latch: 0 none, 1 complete, 2 timeout/fault */
	bool claimed; /* a context claimed the device (DRM quarantined_context) */
	bool held; /* ... and a render node file still references it */
	u32 queued; /* accepted render jobs not yet signalled */
	bool worker_busy; /* the command lock was busy: a worker may be inside a wait */
	bool parked; /* b->parked */
	bool stopped_unparked; /* b->stopped && !b->parked before this run */
	bool power_abort; /* r73b early_power_abort */
};

static inline u32 g5p_r8_in_bits(const struct g5p_r8_in *i)
{
	return i->booted | i->released << 1 | i->alive << 2 | i->failed << 3 |
	       i->fault_event << 4 | i->claimed << 5 | i->held << 6 |
	       (i->queued != 0) << 7 | i->worker_busy << 8 | i->parked << 9 |
	       i->stopped_unparked << 10 | i->power_abort << 11;
}

/* First match wins; the order is the precedence (docs section 3.2). */
static inline enum g5p_r8_class g5p_r8_classify(const struct g5p_r8_in *i)
{
	if (!i->booted)
		return G5P_R8C_NEVER_BOOTED;
	if (!i->released)
		return G5P_R8C_UNRELEASED;
	if (i->stopped_unparked)
		return G5P_R8C_UNKNOWN;
	if (i->power_abort)
		return G5P_R8C_ABORTED;
	if (i->failed || i->fault_event)
		return G5P_R8C_FAULT;
	if (i->terminal == 2)
		return G5P_R8C_TIMEOUT;
	if (i->queued || i->worker_busy)
		return G5P_R8C_IN_FLIGHT;
	if (i->parked)
		return G5P_R8C_PARKED;
	if (!i->alive)
		return G5P_R8C_UNKNOWN;
	if (i->claimed)
		return i->held ? G5P_R8C_HELD : G5P_R8C_CLEAN;
	return G5P_R8C_IDLE;
}

enum g5p_r8_slot {
	G5P_R8S_FW_SLEEP,
	G5P_R8S_STABLE_OFF,
	G5P_R8S_PMGR_GATE,
	G5P_R8S_N,
};

enum g5p_r8_skip {
	G5P_R8K_RAN, /* the slot ran (see its result) */
	G5P_R8K_NO_MECHANISM, /* the glue has no op for it */
	G5P_R8K_OFF, /* its parameter is off (default) */
	G5P_R8K_NOT_FINAL, /* CAPTURE never runs a slot */
	G5P_R8K_NEEDS_RT1, /* needs a verified Fender stop leg in this run */
	G5P_R8K_DIRTY, /* the ladder ended dirty: no register access */
	G5P_R8K_N,
};

static const char *const g5p_r8_skip_name[G5P_R8K_N] = {
	"ran", "no-mechanism", "off", "not-final", "needs-rt1", "dirty",
};

#define G5P_R8F_ADMIT_OPEN \
	BIT(0) /* submitters still inside at the ADMIT deadline */
#define G5P_R8F_ADMIT_LATE BIT(1) /* ... or the step cap ended the wait */
#define G5P_R8F_REENTRY BIT(2) /* answered after retire: nothing ran */
#define G5P_R8F_BUSY BIT(3) /* a concurrent entrant was inside */
#define G5P_R8F_SLOT_FAIL BIT(4) /* a slot ran and failed */
#define G5P_R8F_STABLE_OFF BIT(5) /* stable-off slot read isStableOff == 1 */
/* jobs still queued at DETACH (abandoned to their worker) */
#define G5P_R8F_DETACH_OPEN BIT(6)

/*
 * Ops.  td_run runs the recovery98 ladder and returns its rc; td_flags/td_verdict
 * read its result.  slot[] may hold NULL (no mechanism).  Only slot ops touch a
 * register, and only after the ladder, never after RETIRE.
 */
struct g5p_r8_ops {
	void *ctx;
	u64 (*now_ns)(void *ctx);
	void (*relax_us)(void *ctx, u32 us);
	int (*td_run)(void *ctx, enum g5p_td_mode mode);
	u32 (*td_flags)(void *ctx);
	enum g5p_td_verdict (*td_verdict)(void *ctx);
	int (*slot[G5P_R8S_N])(void *ctx, u32 *value);
	void (*emit)(void *ctx, const char *line);
};

struct g5p_r8 {
	struct g5p_r8_admit admit;
	u32 admit_us; /* ADMIT budget */
	bool slot_on[G5P_R8S_N]; /* the slots' parameters (default all off) */
	/* latches */
	int running;
	int retired; /* 0, or the generation that retired the device */
	u32 gen; /* monotonic: real runs since boot */
	/* last run */
	enum g5p_td_mode mode;
	enum g5p_r8_class cls;
	u32 in_bits, flags, inside0, inside1, steps, detach_queued;
	enum g5p_r8_skip skip[G5P_R8S_N];
	int slot_rc[G5P_R8S_N];
	u32 slot_val[G5P_R8S_N];
	int rc;
	enum g5p_td_verdict verdict;
	u32 lines; /* TR108 lines emitted since boot */
};

static inline void g5p_r8_init(struct g5p_r8 *s)
{
	*s = (struct g5p_r8){ .admit_us = G5P_R8_ADMIT_US };
}

/*
 * The predicate for every register access outside the ladder: after RETIRE only
 * RAM, a PMGR read and the AIC (the always-on swirq ack of a late handler entry)
 * remain.  R0 and R1 are refused forever, whatever the verdict.
 */
static inline bool g5p_r8_mmio_ok(const struct g5p_r8 *s, enum g5p_td_dom d)
{
	if (d == G5P_TDD_RAM || d == G5P_TDD_PMGR || d == G5P_TDD_AIC)
		return true;
	return !READ_ONCE(s->retired);
}

static inline bool g5p_r8_retired(const struct g5p_r8 *s)
{
	return READ_ONCE(s->retired) != 0;
}

/* The DRM frontend's DETACH callback reports what it left to the job workers. */
static inline void g5p_r8_note_detach(struct g5p_r8 *s, u32 queued)
{
	WRITE_ONCE(s->detach_queued, queued);
	if (queued)
		s->flags |= G5P_R8F_DETACH_OPEN;
}

static inline void g5p_r8_line(struct g5p_r8 *s, const struct g5p_r8_ops *o,
			       const char *line)
{
	s->lines++;
	if (o->emit)
		o->emit(o->ctx, line);
}

/* Bounded RAM-only wait for submitters inside the admission window. */
static inline void g5p_r8_admit_wait(struct g5p_r8 *s,
				     const struct g5p_r8_ops *o)
{
	u64 deadline = o->now_ns(o->ctx) + (u64)s->admit_us * 1000;

	s->inside0 = g5p_r8_admit_close(&s->admit);
	s->inside1 = s->inside0;
	s->steps = 0;
	while (s->inside1) {
		if (s->steps >= G5P_R8_MAX_STEPS) {
			s->flags |= G5P_R8F_ADMIT_LATE;
			break;
		}
		s->steps++;
		if (o->now_ns(o->ctx) >= deadline)
			break;
		o->relax_us(o->ctx, G5P_R8_POLL_US);
		s->inside1 = g5p_r8_admit_inside(&s->admit);
	}
	if (s->inside1)
		s->flags |= G5P_R8F_ADMIT_OPEN;
}

static inline void g5p_r8_slots(struct g5p_r8 *s, const struct g5p_r8_ops *o)
{
	u32 tdf = o->td_flags ? o->td_flags(o->ctx) : 0;
	unsigned int i;

	for (i = 0; i < G5P_R8S_N; i++) {
		s->slot_rc[i] = 0;
		s->slot_val[i] = 0;
		if (s->mode != G5P_TD_FINAL)
			s->skip[i] = G5P_R8K_NOT_FINAL;
		else if (!o->slot[i])
			s->skip[i] = G5P_R8K_NO_MECHANISM;
		else if (!s->slot_on[i])
			s->skip[i] = G5P_R8K_OFF;
		else if (s->verdict == G5P_TDV_DIRTY)
			s->skip[i] = G5P_R8K_DIRTY;
		else if (!(tdf & G5P_TDF_RT1_OK))
			s->skip[i] = G5P_R8K_NEEDS_RT1;
		else
			s->skip[i] = G5P_R8K_RAN;
	}
	/* One attempt each, in slot order, no retry; a failed slot ends the slots. */
	for (i = 0; i < G5P_R8S_N; i++) {
		if (s->skip[i] != G5P_R8K_RAN)
			continue;
		if (s->flags & G5P_R8F_SLOT_FAIL) {
			s->skip[i] = G5P_R8K_DIRTY;
			continue;
		}
		s->slot_rc[i] = o->slot[i](o->ctx, &s->slot_val[i]);
		if (s->slot_rc[i])
			s->flags |= G5P_R8F_SLOT_FAIL;
		else if (i == G5P_R8S_STABLE_OFF && s->slot_val[i] == 1)
			s->flags |= G5P_R8F_STABLE_OFF;
	}
}

/*
 * One run.  mode is the teardown mode the glue chose (teardown=1 CAPTURE,
 * teardown=2 FINAL).  Returns the ladder's rc, -EBUSY for a concurrent entrant,
 * and the cached rc after retire.  Lines:
 *   TR108 gen= phase=classify why= mode= class= dirty= in= inside= after= flags= admit_us=
 *   (TD98 lines of the ladder, unchanged)
 *   TR108 gen= phase=retired|captured verdict= class= flags= fw-sleep= stable-off= pmgr-gate=
 *         so=<isStableOff> detach_queued= rc=
 *   TR108 gen= phase=reentry why= retired_gen= rc=      (after retire; no gen step)
 */
static inline int g5p_r8_run(struct g5p_r8 *s, const struct g5p_r8_ops *o,
			     enum g5p_td_mode mode, const struct g5p_r8_in *in,
			     const char *why)
{
	char line[256];
	u64 t0;
	int rc;

	if (cmpxchg(&s->running, 0, 1)) {
		/* A concurrent entrant touches nothing; it does not reach the ladder either. */
		scnprintf(line, sizeof(line),
			  "TR108 gen=%u phase=busy why=%s mode=%s rc=%d",
			  READ_ONCE(s->gen), why, g5p_td_mode_name[mode],
			  -EBUSY);
		if (o->emit)
			o->emit(o->ctx, line);
		return -EBUSY;
	}
	if (READ_ONCE(s->retired)) {
		/* The ladder answers from its cache too (REENTRY); nothing else runs. */
		rc = o->td_run(o->ctx, mode);
		scnprintf(line, sizeof(line),
			"TR108 gen=%u phase=reentry why=%s mode=%s retired_gen=%d rc=%d",
			s->gen, why, g5p_td_mode_name[mode],
			READ_ONCE(s->retired), rc);
		g5p_r8_line(s, o, line);
		WRITE_ONCE(s->running, 0);
		return s->rc;
	}
	s->gen++;
	s->mode = mode;
	s->flags = 0;
	s->inside0 = 0;
	s->inside1 = 0;
	s->steps = 0;
	WRITE_ONCE(s->detach_queued, 0);
	t0 = o->now_ns(o->ctx);
	if (mode == G5P_TD_FINAL)
		g5p_r8_admit_wait(s, o);
	s->cls = g5p_r8_classify(in);
	s->in_bits = g5p_r8_in_bits(in);
	scnprintf(line, sizeof(line),
		"TR108 gen=%u phase=classify why=%s mode=%s class=%s dirty=%u in=%03x term=%d queued=%u inside=%u after=%u flags=%02x admit_us=%u",
		s->gen, why, g5p_td_mode_name[mode], g5p_r8_class_name[s->cls],
		g5p_r8_class_dirty(s->cls), s->in_bits, in->terminal,
		in->queued, s->inside0, s->inside1, s->flags,
		(u32)((o->now_ns(o->ctx) - t0) / 1000));
	/* The receipt is out before the ladder's first AIC or R1 access. */
	g5p_r8_line(s, o, line);
	s->rc = o->td_run(o->ctx, mode);
	s->verdict = o->td_verdict ? o->td_verdict(o->ctx) : G5P_TDV_NONE;
	g5p_r8_slots(s, o);
	if (mode == G5P_TD_FINAL)
		WRITE_ONCE(s->retired, (int)s->gen);
	scnprintf(line, sizeof(line),
		"TR108 gen=%u phase=%s verdict=%s class=%s flags=%02x fw-sleep=%s stable-off=%s pmgr-gate=%s so=%u detach_queued=%u rc=%d",
		s->gen, mode == G5P_TD_FINAL ? "retired" : "captured",
		g5p_td_verdict_name[s->verdict], g5p_r8_class_name[s->cls],
		s->flags, g5p_r8_skip_name[s->skip[G5P_R8S_FW_SLEEP]],
		g5p_r8_skip_name[s->skip[G5P_R8S_STABLE_OFF]],
		g5p_r8_skip_name[s->skip[G5P_R8S_PMGR_GATE]],
		s->slot_val[G5P_R8S_STABLE_OFF], READ_ONCE(s->detach_queued),
		s->rc);
	g5p_r8_line(s, o, line);
	WRITE_ONCE(s->running, 0);
	return s->rc;
}

#endif

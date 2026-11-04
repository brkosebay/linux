/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_TEARDOWN_H
#define G5P_TEARDOWN_H

#include "bits-compat.h"

/*
 * G5P GPU teardown state machine (series gpu-recovery98).
 *
 * Design and evidence: docs/gpu/experiments/gpu-warm-kexec-hang-proof.md.
 *
 * Pure policy: no register access, no kernel API except cmpxchg(), scnprintf()
 * and the errno names. Every hardware or firmware-visible access goes through
 * struct g5p_td_ops, and every call is checked against g5p_td_allow[phase][domain]
 * before it is made. The same file is compiled by the kernel glue (teardown.inc)
 * and by the offline fault-injection tests (patches/gpu-recovery/tests).
 *
 * Two modes:
 *   CAPTURE  timeout / fault / attach failure, and (gpu-recovery99) any number of looks at a
 * live epoch. Observation only: reads RAM rings,
 *            changes nothing the host could still want to export. Never writes
 *            driver or firmware-shared state, never masks the IRQ, never touches
 *            a register. Arms the quiet latch for diagnostics (g5p_td_mmio_ok()).
 *   FINAL    shutdown / reboot notifier / remove. Runs the whole ladder and leaves
 *            a known state for the next kernel.
 *
 * Domains: RAM (DMA pages + driver structs), AIC (swirq 0x20e104000 ack and the
 * IRQ-chip mask), R1 (Fender wrapper window), R0 (core register window), PMGR
 * (0x20e080340 read-only). R0 has no op at all and is forbidden in every phase.
 * PMGR is read-only in every phase. R1 is reachable only in the RESET phase.
 *
 * Nothing here frees or reuses memory the GPU might still access.
 */

#define G5P_TD_NCMD 4 /* command rings: shared objects 0x60 0x64 0x68 0x6c */
#define G5P_TD_NRING (G5P_TD_NCMD + 1) /* + event ring (object 0x70) */
#define G5P_TD_MAX_RCPT 12
#define G5P_TD_MAX_STEPS 4096 /* hard stop independent of the clock */
#define G5P_TD_POLL_US 100
#define G5P_TD_EV_SLACK \
	16 /* events produced during the phase above this = storm */

enum g5p_td_mode {
	G5P_TD_CAPTURE = 1,
	G5P_TD_FINAL = 2,
};

enum g5p_td_phase {
	G5P_TDP_FREEZE, /* stopped=1 alive=0, watchdog disarmed (RAM) */
	G5P_TDP_ACCOUNT, /* RAM-only ring/event accounting */
	G5P_TDP_DRAIN, /* bounded RAM-only wait for the firmware to self-drain */
	G5P_TDP_EVENTS, /* bounded RAM-only event-ring quiesce */
	G5P_TDP_IRQ, /* AIC ack, IRQ mask, settle */
	G5P_TDP_DETACH, /* host detach, RAM only */
	G5P_TDP_RESET, /* RT1 Fender stop leg when eligible; PMGR read */
	G5P_TDP_DONE,
	G5P_TDP_N,
};

enum g5p_td_dom {
	G5P_TDD_RAM,
	G5P_TDD_AIC,
	G5P_TDD_R1,
	G5P_TDD_R0,
	G5P_TDD_PMGR,
	G5P_TDD_N,
};

#define G5P_TDA_R 1
#define G5P_TDA_W 2
#define G5P_TDA_RW 3

/* Hard stop boundaries. A phase may only use the domains marked here. */
static const unsigned char g5p_td_allow[G5P_TDP_N][G5P_TDD_N] = {
	/*                        RAM          AIC         R1           R0  PMGR */
	[G5P_TDP_FREEZE] = { G5P_TDA_RW, 0, 0, 0, 0 },
	[G5P_TDP_ACCOUNT] = { G5P_TDA_R, 0, 0, 0, 0 },
	[G5P_TDP_DRAIN] = { G5P_TDA_R, 0, 0, 0, 0 },
	[G5P_TDP_EVENTS] = { G5P_TDA_RW, 0, 0, 0, 0 },
	[G5P_TDP_IRQ] = { G5P_TDA_R, G5P_TDA_W, 0, 0, 0 },
	[G5P_TDP_DETACH] = { G5P_TDA_RW, 0, 0, 0, 0 },
	[G5P_TDP_RESET] = { G5P_TDA_R, 0, G5P_TDA_RW, 0, G5P_TDA_R },
	[G5P_TDP_DONE] = { G5P_TDA_R, 0, 0, 0, 0 },
};

#define G5P_TDF_NOT_BOOTED \
	BIT(0) /* shared pages not mapped: nothing to tear down */
#define G5P_TDF_UNRELEASED BIT(1) /* firmware never released */
#define G5P_TDF_PROTOCOL \
	BIT(2) /* ring index > 255, read error, producer moved */
#define G5P_TDF_TERMINAL BIT(3) /* TA complete/timeout latched before entry */
#define G5P_TDF_LATE_TERMINAL BIT(4) /* ... or during the teardown (race) */
#define G5P_TDF_ABANDON BIT(5) /* >=1 ring pending after the drain */
#define G5P_TDF_SELF_DRAINED \
	BIT(6) /* >=1 ring consumed by the firmware, no bell */
#define G5P_TDF_FAULT_EVENT \
	BIT(7) /* event type other than 0/1 left unconsumed */
#define G5P_TDF_EV_UNQUIET \
	BIT(8) /* event ring still non-empty after the budget */
#define G5P_TDF_IRQ_UNSETTLED BIT(9) /* IRQ count still moving after the mask */
#define G5P_TDF_RT1_OK BIT(10) /* Fender stop leg verified GPC=0 */
#define G5P_TDF_RT1_FAIL BIT(11) /* stop leg attempted, not verified */
#define G5P_TDF_RT1_SKIPPED BIT(12) /* no register access; see skip reason */
#define G5P_TDF_OVERRUN BIT(13) /* step or time budget exhausted */
#define G5P_TDF_VIOLATION BIT(14) /* a gate refused an access (machine bug) */
#define G5P_TDF_CAPTURE_ONLY BIT(15) /* CAPTURE run: nothing was changed */
#define G5P_TDF_REENTRY BIT(16) /* second entry answered from the cache */

enum g5p_td_skip {
	G5P_TDS_NONE,
	G5P_TDS_NOT_BOOTED,
	G5P_TDS_NOT_RELEASED,
	G5P_TDS_NOT_POWERED,
	G5P_TDS_ALREADY_PARKED, /* clean: GPC=0 already verified by park */
	G5P_TDS_STOPPED_UNPARKED, /* dirty: an earlier stop leg did not verify */
	G5P_TDS_POWER_ABORT,
	G5P_TDS_FW_POLICY, /* meta+0x6c8 != 0: firmware also writes R1+0x1000 */
	G5P_TDS_LOCK_BUSY,
	G5P_TDS_CAPTURE_MODE,
	G5P_TDS_BAD_STATE, /* protocol/overrun/unsettled IRQ: no register access */
	G5P_TDS_N,
};

enum g5p_td_verdict {
	G5P_TDV_NONE,
	G5P_TDV_OBSERVED, /* CAPTURE: nothing changed */
	G5P_TDV_UNTOUCHED, /* never booted or never released: nothing to do */
	G5P_TDV_QUIESCED, /* rings clear, events clear, IRQ masked, GPC=0 verified */
	G5P_TDV_ABANDONED, /* as QUIESCED, but >=1 command abandoned after the drain */
	G5P_TDV_RAM_ONLY, /* IRQ masked, no register access: GPU left as found */
	G5P_TDV_DIRTY, /* a step failed: warm continue forbidden, cold DFU */
	G5P_TDV_N,
};

static const char *const g5p_td_phase_name[G5P_TDP_N] = {
	"freeze", "account", "drain", "events",
	"irq",	  "detach",  "reset", "done",
};

static const char *const g5p_td_mode_name[3] = { "none", "capture", "final" };

static const char *const g5p_td_verdict_name[G5P_TDV_N] = {
	"none",	     "observed", "untouched", "quiesced",
	"abandoned", "ram-only", "dirty",
};

struct g5p_td_budget {
	u32 drain_us; /* self-drain wait; DC3 took 2.34 ms (r87b) */
	u32 events_us;
	u32 settle_us; /* IRQ count must stay constant this long after the mask */
	u32 events_pass_max; /* entries per pass; the ring has 256 slots */
	u32 events_passes;
	u32 total_us; /* RESET is skipped when the RAM phases already overran this */
};

#define G5P_TD_BUDGET_DEFAULT { 20000, 20000, 2000, 256, 4, 100000 }

/* Inputs the glue reads from RAM before the run (never from a register). */
struct g5p_td_pre {
	bool booted; /* maps_ready: shared pages exist */
	bool released; /* firmware was released */
	bool powered; /* ps_gfx held on by the retained PM reference */
	bool fender_ready;
	bool already_parked; /* b->parked: GPC=0 verified by park */
	bool stopped_unparked; /* b->stopped && !b->parked: an earlier stop did not verify */
	bool policy_host_owned; /* init meta+0x6c8 == 0 */
	bool power_abort; /* r73b early_power_abort latched */
	bool lock_held; /* the glue holds g->lock: no worker is inside a hardware wait */
};

/*
 * Every op is bounded by its own contract; the machine additionally bounds how
 * often and for how long it calls them. Ops marked RAM never touch a register.
 */
struct g5p_td_ops {
	void *ctx;
	u64 (*now_ns)(void *ctx);
	void (*relax_us)(void *ctx, u32 us); /* sleep, no register access */
	void (*freeze)(void *ctx); /* RAM: stopped=1 alive=0, disarm */
	/* RAM: idx 0..NCMD-1 command rings, NCMD = event ring. -ENODEV: not mapped. */
	int (*ring_read)(void *ctx, u32 idx, u32 *cons, u32 *prod);
	/* RAM: consume at most max entries of type 0/1, stop at the first other type. */
	int (*event_pass)(void *ctx, u32 max, u32 *consumed, u32 *left,
			  bool *fault);
	bool (*terminal_seen)(void *ctx); /* RAM: TA complete/timeout latch */
	void (*detach)(void *ctx); /* RAM: signal/forget host jobs */
	void (*aic_ack)(void *ctx); /* AIC: swirq+0x94 = 0x4000 */
	void (*irq_mask)(void *ctx); /* AIC: ack stays, line masked */
	u32 (*irq_total)(void *ctx); /* RAM: handler entry counter */
	int (*fender_stop)(void *ctx); /* R1: g5p_fender_idle() */
	int (*pmgr_actual)(void *ctx, u32 *target,
			   u32 *actual); /* PMGR read; may be NULL */
	void (*emit)(void *ctx, const char *line); /* printk; may be NULL */
};

struct g5p_td_receipt {
	unsigned char seq, phase, mode, verdict;
	s32 rc;
	u32 us, steps, flags;
	unsigned char cnt[G5P_TDD_N]
			 [2]; /* [dom][0]=reads [dom][1]=writes, op level */
	u32 a, b; /* phase specific, see g5p_td_run() */
};

struct g5p_td {
	struct g5p_td_budget budget;
	/* latches shared with the render path; touched by cmpxchg only */
	int running; /* 0 idle, 1 inside g5p_td_run() */
	int terminal; /* 0, 1 complete, 2 timeout: first writer wins */
	int quiet; /* diagnostics MMIO forbidden from here on */
	/* 0 idle, 1 captured (informational), 2 finalized: only 2 answers from the cache */
	int state;
	u32 snaps; /* CAPTURE runs since init: the n= of the TD99 snap line */
	/* per run */
	enum g5p_td_mode mode;
	enum g5p_td_phase phase;
	enum g5p_td_skip skip;
	enum g5p_td_verdict verdict;
	bool eligible;
	u32 flags, steps, violations, seq, nrcpt;
	u64 t0_ns;
	unsigned char cnt[G5P_TDD_N]
			 [2]; /* [enum g5p_td_dom][0=reads,1=writes], op level */
	u32 pending_mask, pending_initial, abandoned, drained;
	u32 ring_cons[G5P_TD_NRING], ring_prod[G5P_TD_NRING];
	u32 ev_consumed, ev_left, ev_initial, irq_before, irq_after;
	u32 us_total;
	u32 pmgr_target, pmgr_actual;
	int rc;
	struct g5p_td_receipt rcpt[G5P_TD_MAX_RCPT];
};

static inline void g5p_td_init(struct g5p_td *t)
{
	const struct g5p_td_budget b = G5P_TD_BUDGET_DEFAULT;

	*t = (struct g5p_td){ .budget = b };
}

/* One latch for "the TA reached a terminal state"; the first writer wins. */
static inline bool g5p_td_note_terminal(struct g5p_td *t, int kind)
{
	if (cmpxchg(&t->terminal, 0, kind) != 0)
		return false;
	WRITE_ONCE(t->quiet, 1);
	return true;
}

/*
 * The single predicate for diagnostic register access (R0 snapshots, end banks,
 * exports): false once the epoch is terminal or a teardown has started. Nothing
 * in this series calls a diagnostic; this exists so the render path can.
 */

/* RT1 (Fender stop leg) eligibility. Pure function of RAM-derived inputs. */
static inline bool g5p_td_eligible(const struct g5p_td_pre *p,
				   enum g5p_td_mode mode, enum g5p_td_skip *why)
{
	enum g5p_td_skip s = G5P_TDS_NONE;

	if (!p->booted)
		s = G5P_TDS_NOT_BOOTED;
	else if (!p->released)
		s = G5P_TDS_NOT_RELEASED;
	else if (!p->powered || !p->fender_ready)
		s = G5P_TDS_NOT_POWERED;
	else if (p->already_parked)
		s = G5P_TDS_ALREADY_PARKED;
	else if (p->stopped_unparked)
		s = G5P_TDS_STOPPED_UNPARKED;
	else if (p->power_abort)
		s = G5P_TDS_POWER_ABORT;
	else if (!p->policy_host_owned)
		s = G5P_TDS_FW_POLICY;
	else if (!p->lock_held)
		s = G5P_TDS_LOCK_BUSY;
	else if (mode != G5P_TD_FINAL)
		s = G5P_TDS_CAPTURE_MODE;
	if (why)
		*why = s;
	return s == G5P_TDS_NONE;
}

static inline int g5p_td_format(const struct g5p_td_receipt *r, char *buf,
				size_t n)
{
	return scnprintf(buf, n,
		"TD98 seq=%u phase=%s mode=%s rc=%d us=%u steps=%u flags=%08x aic=%u/%u r1=%u/%u r0=%u/%u pmgr=%u/%u a=%u b=%u verdict=%s",
		r->seq, g5p_td_phase_name[r->phase], g5p_td_mode_name[r->mode],
		r->rc, r->us, r->steps, r->flags, r->cnt[G5P_TDD_AIC][0],
		r->cnt[G5P_TDD_AIC][1], r->cnt[G5P_TDD_R1][0],
		r->cnt[G5P_TDD_R1][1], r->cnt[G5P_TDD_R0][0],
		r->cnt[G5P_TDD_R0][1], r->cnt[G5P_TDD_PMGR][0],
		r->cnt[G5P_TDD_PMGR][1], r->a, r->b,
		g5p_td_verdict_name[r->verdict]);
}

static inline u32 g5p_td_us_since(const struct g5p_td *t,
				  const struct g5p_td_ops *o)
{
	u64 d = o->now_ns(o->ctx) - t->t0_ns;

	return d > 0xffffffffULL * 1000 ? 0xffffffffu : (u32)(d / 1000);
}

static inline void g5p_td_receipt(struct g5p_td *t, const struct g5p_td_ops *o,
				  int rc, u32 a, u32 b)
{
	struct g5p_td_receipt *r =
		&t->rcpt[t->nrcpt < G5P_TD_MAX_RCPT ? t->nrcpt :
						      G5P_TD_MAX_RCPT - 1];
	char line[224];
	unsigned int d;

	r->seq = (unsigned char)t->seq++;
	r->phase = t->phase;
	r->mode = t->mode;
	r->verdict = t->verdict;
	r->rc = rc;
	r->us = g5p_td_us_since(t, o);
	r->steps = t->steps;
	r->flags = t->flags;
	for (d = 0; d < G5P_TDD_N; d++) {
		r->cnt[d][0] = t->cnt[d][0];
		r->cnt[d][1] = t->cnt[d][1];
	}
	r->a = a;
	r->b = b;
	if (t->nrcpt < G5P_TD_MAX_RCPT)
		t->nrcpt++;
	if (o->emit) {
		g5p_td_format(r, line, sizeof(line));
		o->emit(o->ctx, line);
	}
}

/* Count and check one access. A refused access is a bug in the machine itself. */
static inline int g5p_td_gate(struct g5p_td *t, enum g5p_td_dom d,
			      unsigned int acc)
{
	if ((g5p_td_allow[t->phase][d] & acc) != acc) {
		t->violations++;
		t->flags |= G5P_TDF_VIOLATION;
		return -EACCES;
	}
	if ((acc & G5P_TDA_R) && t->cnt[d][0] < 255)
		t->cnt[d][0]++;
	if ((acc & G5P_TDA_W) && t->cnt[d][1] < 255)
		t->cnt[d][1]++;
	return 0;
}

/* One loop iteration or hardware op. False once the hard step cap is hit. */
static inline bool g5p_td_step(struct g5p_td *t)
{
	if (t->steps >= G5P_TD_MAX_STEPS) {
		t->flags |= G5P_TDF_OVERRUN;
		return false;
	}
	t->steps++;
	return true;
}

static inline bool g5p_td_ring_ok(u32 cons, u32 prod)
{
	return cons <= 255 && prod <= 255;
}

static inline void g5p_td_note_late(struct g5p_td *t,
				    const struct g5p_td_ops *o)
{
	if (o->terminal_seen && o->terminal_seen(o->ctx) &&
	    !(t->flags & G5P_TDF_TERMINAL))
		t->flags |= G5P_TDF_LATE_TERMINAL;
}

static inline int g5p_td_account(struct g5p_td *t, const struct g5p_td_ops *o)
{
	u32 i, bad = 0, pending = 0, cons, prod;
	int rc;

	for (i = 0; i < G5P_TD_NRING; i++) {
		if (!g5p_td_step(t))
			return -ETIMEDOUT;
		if (g5p_td_gate(t, G5P_TDD_RAM, G5P_TDA_R))
			return -EACCES;
		rc = o->ring_read(o->ctx, i, &cons, &prod);
		if (rc == -ENODEV) {
			t->flags |= G5P_TDF_NOT_BOOTED;
			break;
		}
		if (rc || !g5p_td_ring_ok(cons, prod)) {
			t->flags |= G5P_TDF_PROTOCOL;
			bad++;
			continue;
		}
		t->ring_cons[i] = cons;
		t->ring_prod[i] = prod;
		if (i < G5P_TD_NCMD && cons != prod) {
			t->pending_mask |= 1u << i;
			pending++;
		}
	}
	t->pending_initial = pending;
	t->ev_left = (t->ring_prod[G5P_TD_NCMD] - t->ring_cons[G5P_TD_NCMD]) &
		     255;
	t->ev_initial = t->ev_left;
	g5p_td_receipt(t, o, bad ? -EPROTO : 0, pending, bad);
	return 0;
}

/*
 * gpu-recovery99 0004: one extra line per CAPTURE run, after the ACCOUNT receipt, so a live
 * epoch can be looked at
 * several times (S1a). RAM only: it formats what ACCOUNT stored in struct g5p_td and reads the
 * handler entry counter
 * (RAM) and the clock; no register, no state change. The prefix is not TD98 on purpose: receipt
 * parsers keyed on
 * "TD98 seq=" / "TD98 begin" ignore it.
 *   n      CAPTURE runs since init (1 based)
 *   t_ns   ops->now_ns(), for the IRQ rate between two lines
 *   irq    handler entry counter (ops->irq_total), 0 if there is no such op
 *   term   terminal latch (0 none, 1 complete, 2 timeout); quiet = the diagnostics latch
 *   own    bit0 booted 1 released 2 powered 3 fender_ready 4 already_parked 5 stopped_unparked
 *          6 policy_host_owned 7 power_abort 8 lock_held
 *   cN/pN  command ring N consumer / producer; ce/pe the event ring. Zero when the shared pages
 * are not mapped.
 */
static inline void g5p_td_snap(struct g5p_td *t, const struct g5p_td_ops *o,
			       const struct g5p_td_pre *pre)
{
	char line[224];
	u32 own, irq = 0;

	t->snaps++;
	if (!o->emit)
		return;
	if (!g5p_td_gate(t, G5P_TDD_RAM, G5P_TDA_R) && o->irq_total)
		irq = o->irq_total(o->ctx);
	own = pre->booted | pre->released << 1 | pre->powered << 2 |
	      pre->fender_ready << 3 | pre->already_parked << 4 |
	      pre->stopped_unparked << 5 | pre->policy_host_owned << 6 |
	      pre->power_abort << 7 | pre->lock_held << 8;
	scnprintf(line, sizeof(line),
		"TD99 snap n=%u t_ns=%llu irq=%u term=%d quiet=%d own=%03x c0=%u p0=%u c1=%u p1=%u c2=%u p2=%u c3=%u p3=%u ce=%u pe=%u",
		t->snaps, (unsigned long long)o->now_ns(o->ctx), irq,
		READ_ONCE(t->terminal), READ_ONCE(t->quiet), own,
		t->ring_cons[0], t->ring_prod[0], t->ring_cons[1],
		t->ring_prod[1], t->ring_cons[2], t->ring_prod[2],
		t->ring_cons[3], t->ring_prod[3], t->ring_cons[G5P_TD_NCMD],
		t->ring_prod[G5P_TD_NCMD]);
	o->emit(o->ctx, line);
}

static inline void g5p_td_drain(struct g5p_td *t, const struct g5p_td_ops *o)
{
	u64 deadline = o->now_ns(o->ctx) + (u64)t->budget.drain_us * 1000;
	u32 i, cons, prod;

	while (t->pending_mask) {
		if (!g5p_td_step(t))
			break;
		if (o->now_ns(o->ctx) >= deadline) {
			t->flags |= G5P_TDF_ABANDON;
			break;
		}
		o->relax_us(o->ctx, G5P_TD_POLL_US);
		for (i = 0; i < G5P_TD_NCMD; i++) {
			if (!(t->pending_mask & (1u << i)))
				continue;
			if (g5p_td_gate(t, G5P_TDD_RAM, G5P_TDA_R))
				return;
			if (o->ring_read(o->ctx, i, &cons, &prod) ||
			    !g5p_td_ring_ok(cons, prod) ||
			    prod != t->ring_prod[i]) {
				/* The producer is host-written and the host is frozen. */
				t->flags |= G5P_TDF_PROTOCOL;
				t->pending_mask &= ~(1u << i);
				continue;
			}
			if (cons == prod) {
				t->pending_mask &= ~(1u << i);
				t->flags |= G5P_TDF_SELF_DRAINED;
			}
		}
	}
	for (i = 0; i < G5P_TD_NCMD; i++)
		if (t->pending_mask & (1u << i))
			t->abandoned++;
	if (t->abandoned)
		t->flags |= G5P_TDF_ABANDON;
	t->drained = t->pending_initial - t->abandoned;
}

static inline void g5p_td_events(struct g5p_td *t, const struct g5p_td_ops *o)
{
	u64 deadline = o->now_ns(o->ctx) + (u64)t->budget.events_us * 1000;
	u32 pass, consumed, left = t->ev_left;
	bool fault = false;
	int rc;

	for (pass = 0; pass < t->budget.events_passes && left; pass++) {
		if (!g5p_td_step(t))
			break;
		if (g5p_td_gate(t, G5P_TDD_RAM, G5P_TDA_RW))
			return;
		consumed = 0;
		rc = o->event_pass(o->ctx, t->budget.events_pass_max, &consumed,
				   &left, &fault);
		t->ev_consumed += consumed;
		if (rc) {
			t->flags |= G5P_TDF_PROTOCOL;
			break;
		}
		if (fault) {
			/* Retained for the host; stop consuming, the IRQ is masked next. */
			t->flags |= G5P_TDF_FAULT_EVENT;
			break;
		}
		if (!left || o->now_ns(o->ctx) >= deadline)
			break;
		o->relax_us(o->ctx, G5P_TD_POLL_US);
	}
	t->ev_left = left;
	/*
	 * A ring holds at most 255 entries, so consuming more than the backlog plus a
	 * few ALIVE ticks means the producer outran us: an empty ring at the last
	 * look is a snapshot, not a quiesce.
	 */
	if (!fault && (left || (s32)(t->ev_consumed + left - t->ev_initial) >
				       G5P_TD_EV_SLACK))
		t->flags |= G5P_TDF_EV_UNQUIET;
}

static inline void g5p_td_irq(struct g5p_td *t, const struct g5p_td_ops *o)
{
	u32 tries, n0, n1;

	if (g5p_td_gate(t, G5P_TDD_AIC, G5P_TDA_W) ||
	    g5p_td_gate(t, G5P_TDD_AIC, G5P_TDA_W))
		return;
	t->irq_before = o->irq_total(o->ctx);
	/* Same pair as the IRQ-cap path: ack the software source, then mask the line. */
	o->aic_ack(o->ctx);
	o->irq_mask(o->ctx);
	for (tries = 0; tries < 3; tries++) {
		if (!g5p_td_step(t))
			break;
		n0 = o->irq_total(o->ctx);
		o->relax_us(o->ctx, t->budget.settle_us);
		n1 = o->irq_total(o->ctx);
		t->irq_after = n1;
		if (n0 == n1)
			return;
	}
	t->flags |= G5P_TDF_IRQ_UNSETTLED;
}

static inline void g5p_td_reset(struct g5p_td *t, const struct g5p_td_ops *o,
				const struct g5p_td_pre *pre)
{
	enum g5p_td_skip why = G5P_TDS_NONE;
	bool bad = t->flags & (G5P_TDF_PROTOCOL | G5P_TDF_OVERRUN |
			       G5P_TDF_IRQ_UNSETTLED | G5P_TDF_VIOLATION);
	int rc = 0;

	t->eligible = g5p_td_eligible(pre, t->mode, &why);
	if (t->eligible &&
	    (bad || g5p_td_us_since(t, o) > t->budget.total_us)) {
		t->eligible = false;
		why = G5P_TDS_BAD_STATE;
	}
	t->skip = why;
	if (!t->eligible) {
		t->flags |= G5P_TDF_RT1_SKIPPED;
	} else if (!g5p_td_step(t) || g5p_td_gate(t, G5P_TDD_R1, G5P_TDA_RW)) {
		t->flags |= G5P_TDF_RT1_SKIPPED;
		t->skip = G5P_TDS_BAD_STATE;
	} else {
		/*
		 * Apple stopFirmwareForPowerDown 6c9fa10, R1 only; flown here after a DC
		 * timeout (round 10). Never retried, even on failure.
		 */
		rc = o->fender_stop(o->ctx);
		t->flags |= rc ? G5P_TDF_RT1_FAIL : G5P_TDF_RT1_OK;
	}
	if (o->pmgr_actual && !g5p_td_gate(t, G5P_TDD_PMGR, G5P_TDA_R))
		o->pmgr_actual(o->ctx, &t->pmgr_target, &t->pmgr_actual);
	g5p_td_receipt(t, o, rc, t->eligible ? 0 : (u32)t->skip,
		       t->pmgr_actual);
}

static inline void g5p_td_decide(struct g5p_td *t, const struct g5p_td_pre *pre)
{
	bool dirty = t->flags & (G5P_TDF_PROTOCOL | G5P_TDF_EV_UNQUIET |
				 G5P_TDF_IRQ_UNSETTLED | G5P_TDF_RT1_FAIL |
				 G5P_TDF_OVERRUN | G5P_TDF_VIOLATION) ||
		     t->skip == G5P_TDS_STOPPED_UNPARKED ||
		     t->skip == G5P_TDS_BAD_STATE;

	if (t->mode == G5P_TD_CAPTURE)
		t->verdict = (t->flags & G5P_TDF_VIOLATION) ? G5P_TDV_DIRTY :
							      G5P_TDV_OBSERVED;
	else if (dirty)
		t->verdict = G5P_TDV_DIRTY;
	else if (t->flags & (G5P_TDF_NOT_BOOTED | G5P_TDF_UNRELEASED))
		t->verdict = G5P_TDV_UNTOUCHED;
	else if (t->flags & G5P_TDF_RT1_OK || t->skip == G5P_TDS_ALREADY_PARKED)
		t->verdict = (t->flags & G5P_TDF_ABANDON) ? G5P_TDV_ABANDONED :
							    G5P_TDV_QUIESCED;
	else
		t->verdict = G5P_TDV_RAM_ONLY;
	(void)pre;
}

/*
 * Run the teardown once per mode. Returns 0 for QUIESCED/ABANDONED/UNTOUCHED/OBSERVED,
 * -EAGAIN for RAM_ONLY (clean, but the GPU was left as found), -EIO for DIRTY,
 * -EBUSY if another caller is inside the machine. Every phase ends in a receipt.
 * Receipt a/b: account pending/bad, drain drained/abandoned, events consumed/left,
 * irq before/after, reset skip-reason/pmgr-actual, done skip-reason/abandoned.
 * Counters in the receipts are op-level (one count per gate): the AIC pair is
 * 2 writes, the whole Fender stop leg is one R1 read+write op.
 */
static inline int g5p_td_run(struct g5p_td *t, const struct g5p_td_ops *o,
			     enum g5p_td_mode mode,
			     const struct g5p_td_pre *pre)
{
	if (cmpxchg(&t->running, 0, 1))
		return -EBUSY;
	WRITE_ONCE(t->quiet, 1);
	if (t->state == 2) {
		/* Answered from the cache (after FINAL only; CAPTURE re-runs, gpu-
		 * recovery99 0004): no phase runs, nothing is accessed.
		 */
		u32 saved_flags = t->flags;

		t->t0_ns = o->now_ns(o->ctx) - (u64)t->us_total * 1000;
		t->flags |= G5P_TDF_REENTRY;
		t->phase = G5P_TDP_DONE;
		g5p_td_receipt(t, o, t->rc, t->flags, t->steps);
		t->flags = saved_flags;
		WRITE_ONCE(t->running, 0);
		return t->rc;
	}
	t->mode = mode;
	t->phase = G5P_TDP_FREEZE;
	t->t0_ns = o->now_ns(o->ctx);
	t->seq = 0;
	t->nrcpt = 0;
	t->flags = mode == G5P_TD_CAPTURE ? G5P_TDF_CAPTURE_ONLY : 0;
	t->steps = 0;
	t->violations = 0;
	t->abandoned = 0;
	t->drained = 0;
	t->pending_mask = 0;
	t->pending_initial = 0;
	t->ev_consumed = 0;
	t->ev_left = 0;
	t->ev_initial = 0;
	t->irq_before = 0;
	t->irq_after = 0;
	t->skip = G5P_TDS_NONE;
	t->verdict = G5P_TDV_NONE;
	memset(t->cnt, 0, sizeof(t->cnt));
	/* a CAPTURE may now run many times: nothing of the previous look may leak into this one */
	memset(t->ring_cons, 0, sizeof(t->ring_cons));
	memset(t->ring_prod, 0, sizeof(t->ring_prod));
	if (READ_ONCE(t->terminal))
		t->flags |= G5P_TDF_TERMINAL;
	if (!pre->released)
		t->flags |= G5P_TDF_UNRELEASED;

	if (mode == G5P_TD_FINAL) {
		if (!g5p_td_gate(t, G5P_TDD_RAM, G5P_TDA_W))
			o->freeze(o->ctx);
		g5p_td_receipt(t, o, 0, pre->released, pre->policy_host_owned);
	}
	t->phase = G5P_TDP_ACCOUNT;
	if (g5p_td_account(t, o))
		t->flags |= G5P_TDF_OVERRUN;
	g5p_td_note_late(t, o);
	if (mode == G5P_TD_CAPTURE)
		g5p_td_snap(t, o, pre);

	if (mode == G5P_TD_FINAL &&
	    !(t->flags & (G5P_TDF_NOT_BOOTED | G5P_TDF_UNRELEASED |
			  G5P_TDF_PROTOCOL | G5P_TDF_OVERRUN))) {
		t->phase = G5P_TDP_DRAIN;
		g5p_td_drain(t, o);
		g5p_td_note_late(t, o);
		g5p_td_receipt(t, o,
			       (t->flags & G5P_TDF_ABANDON) ? -ETIMEDOUT : 0,
			       t->drained, t->abandoned);

		t->phase = G5P_TDP_EVENTS;
		g5p_td_events(t, o);
		g5p_td_receipt(t, o,
			       (t->flags & G5P_TDF_EV_UNQUIET) ? -ETIMEDOUT : 0,
			       t->ev_consumed, t->ev_left);
	}
	if (mode == G5P_TD_FINAL) {
		/* The IRQ is masked even after an overrun: ack + mask is the safe exit. */
		t->phase = G5P_TDP_IRQ;
		if (!(t->flags & G5P_TDF_NOT_BOOTED))
			g5p_td_irq(t, o);
		g5p_td_receipt(t, o,
			       (t->flags & G5P_TDF_IRQ_UNSETTLED) ? -ETIMEDOUT :
								    0,
			       t->irq_before, t->irq_after);

		t->phase = G5P_TDP_DETACH;
		if (!g5p_td_gate(t, G5P_TDD_RAM, G5P_TDA_W))
			o->detach(o->ctx);
		g5p_td_receipt(t, o, 0, 0, 0);

		t->phase = G5P_TDP_RESET;
		g5p_td_reset(t, o, pre);
	}
	g5p_td_note_late(t, o);
	t->phase = G5P_TDP_DONE;
	g5p_td_decide(t, pre);
	switch (t->verdict) {
	case G5P_TDV_DIRTY:
		t->rc = -EIO;
		break;
	case G5P_TDV_RAM_ONLY:
		t->rc = -EAGAIN;
		break;
	default:
		t->rc = 0;
	}
	t->us_total = g5p_td_us_since(t, o);
	g5p_td_receipt(t, o, t->rc, (u32)t->skip, t->abandoned);
	t->state = mode == G5P_TD_FINAL ? 2 : 1;
	WRITE_ONCE(t->running, 0);
	return t->rc;
}

#endif

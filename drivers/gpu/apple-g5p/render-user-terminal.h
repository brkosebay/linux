/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * TYPED_V3 terminal state of the epoch.  Pure functions over one small struct; the
 * kernel supplies READ_ONCE/WRITE_ONCE/cmpxchg and the host suite shims them, so the
 * host compiles this file as it is.
 *
 * One latch, first writer wins.  The first terminal event of the epoch (the natural TA
 * complete, a timeout, a fault or a protocol error) fixes `kind`; from then on `quiet`
 * is set and never cleared, and g5p_ru_mmio_ok() is false: the host touches no register
 * (R0, R1, AIC, doorbell) and no diagnostic bank.  RAM stays accessible.  A later
 * failure (the 3D after a completed TA, job N+1 after job N) cannot change `kind` but
 * sets the sticky `failed`, which is what the admission uses: after any timeout or fault
 * no new job starts, from any queue, even one that passed the pre-lock context checks
 * before the failing job marked the context bad.
 *
 * The recovery series (patches/gpu-recovery) is independent of this file.  Its
 * g5p_td_note_terminal() takes {1 complete, 2 timeout}; g5p_ru_td_kind() maps ours.
 */
#ifndef G5P_RENDER_USER_TERMINAL_H
#define G5P_RENDER_USER_TERMINAL_H

enum g5p_ru_term_kind {
	G5P_RU_TERM_NONE,
	G5P_RU_TERM_COMPLETE, /* the natural TA completed (RAM) */
	G5P_RU_TERM_TIMEOUT, /* a phase ran out its deadline */
	G5P_RU_TERM_FAULT, /* protocol error, drain error, attach or publish failure */
	G5P_RU_TERM_KINDS,
};

static const char *const g5p_ru_term_names[G5P_RU_TERM_KINDS] = {
	"none",
	"complete",
	"timeout",
	"fault",
};

struct g5p_ru_terminal {
	int kind; /* first writer wins */
	int quiet; /* set with the first note, never cleared */
	int failed; /* sticky: any timeout or fault was noted */
	int captured; /* the CAPTURE entry (recovery hook) was taken, once */
};

/* A new firmware epoch (the in-boot recovery, recover.inc) starts a new latch; nothing else
 * clears it.
 */
static inline void g5p_ru_term_reset(struct g5p_ru_terminal *t)
{
	WRITE_ONCE(t->kind, 0);
	WRITE_ONCE(t->quiet, 0);
	WRITE_ONCE(t->failed, 0);
	WRITE_ONCE(t->captured, 0);
}

/* The kind a job's return value stands for (only called for ret < 0, ret != -ECANCELED). */
static inline int g5p_ru_term_from_ret(int ret)
{
	return ret == -ETIMEDOUT ? G5P_RU_TERM_TIMEOUT : G5P_RU_TERM_FAULT;
}

/* Returns true for the first note of the epoch. */
static inline bool g5p_ru_term_note(struct g5p_ru_terminal *t, int kind)
{
	bool first;

	if (kind <= G5P_RU_TERM_NONE || kind >= G5P_RU_TERM_KINDS)
		kind = G5P_RU_TERM_FAULT;
	first = cmpxchg(&t->kind, G5P_RU_TERM_NONE, kind) == G5P_RU_TERM_NONE;
	if (kind >= G5P_RU_TERM_TIMEOUT)
		WRITE_ONCE(t->failed, 1);
	WRITE_ONCE(t->quiet, 1);
	return first;
}

static inline int g5p_ru_term_kind(const struct g5p_ru_terminal *t)
{
	return READ_ONCE(t->kind);
}

static inline bool g5p_ru_term_failed(const struct g5p_ru_terminal *t)
{
	return READ_ONCE(t->failed);
}

/* The single predicate for register access: false from the first terminal note on. */
static inline bool g5p_ru_mmio_ok(const struct g5p_ru_terminal *t)
{
	return !READ_ONCE(t->quiet);
}

/* True once per epoch for a timeout or fault: the caller may take the recovery CAPTURE entry. */
static inline bool g5p_ru_term_capture_due(struct g5p_ru_terminal *t, int kind)
{
	return kind >= G5P_RU_TERM_TIMEOUT && cmpxchg(&t->captured, 0, 1) == 0;
}

/* Recovery's terminal encoding: 1 complete, 2 timeout (a fault is a no-completion exit: 2). */
static inline int g5p_ru_td_kind(int kind)
{
	return kind == G5P_RU_TERM_COMPLETE ? 1 : 2;
}

#endif

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * TYPED_V3 3D completion: target coherence and engine idle, decided separately.  Pure
 * functions, no kernel services beyond the integer types, so the host suite compiles this
 * file as it is.  Include render-user-wait.h first.
 *
 * combined107 P4R seq 4 (build/gpu/combined107/raw/20261031T145710Z): the 3D target state
 * was coherent (ring consumer = producer = target 4, both stamps 4, W+0x888/0x88c ours,
 * W+0x890 = 4, all five channel watermarks at endpoint 8, and the firmware's type-1 3D
 * completion event with the endpoint pair 7/8 decoded at publish + 19999 ms), yet the
 * wait timed out at 60002 ms.  The wait is a 1-2 ms RAM poll (no waitqueue, no wake), so
 * this was not a lost edge: the done predicate also required the 3D global's idle words
 * (G+0x724/0x728/0x72c/0x734/0x738) to be zero, and they never cleared.
 *
 * The 3D phase is complete when either
 *   (a) the target is coherent and the idle words are zero (the flown r96 predicate), or
 *   (b) the target is coherent and the matching 3D completion event was decoded after this
 *       job's 3D publish (idle_stuck).  The event is the firmware's completion report, the
 *       signal a fence follows on Asahi too; the idle words lag it by an arbitrary time.
 *       settle_ms (default 0) only adds a minimum length of the coherent run, or
 *   (c) the target has been coherent, unbroken, for G5P_RU_RT_HOLD_MS (at least settle_ms):
 *       tahang1, as the TA's rule (c), so a lost or late event does not hold a job whose
 *       stamps landed.  The PM state needs no clause here: the TA phase, which the 3D
 *       publish waits for, required it saved.
 * Target coherent = ring consumer == target && g5p_ru_retired(): both stamps == seq, the W
 * stamp pointers ours, the W sequence word == seq, every channel watermark == endpoint.
 * The stamps carry the identity (31-bit sequence, one per job); the watermarks and the
 * event's endpoint pair repeat every eight jobs and only corroborate.  The TA phase has its
 * own rules (tahang1, below).
 */
#ifndef G5P_RENDER_USER_RETIRE_H
#define G5P_RENDER_USER_RETIRE_H

#define G5P_RU_RT_DEF_SETTLE_MS 0U /* default of render_user_idle_settle_ms */
#define G5P_RU_RT_MAX_SETTLE_MS \
	5000U /* far below the smallest per-phase bound (20 s) */
#define G5P_RU_RT_GRID 16U /* the channel watermark grid, G5P_WORK_SLOTS */
#define G5P_RU_RT_HOLD_MS \
	500U /* rule (c), raised to settle_ms when that is longer */

/* Clauses of the 3D predicate, as the receipt's `unsat` bits (set = not satisfied). */
#define G5P_RU_RT_CONSUMED 0x1U /* ring consumer != target */
#define G5P_RU_RT_RETIRED 0x2U /* g5p_ru_retired() false */
#define G5P_RU_RT_IDLE 0x4U /* an idle word is non-zero */
#define G5P_RU_RT_EVENT 0x8U /* no matching completion event latched */
#define G5P_RU_RT_SETTLE \
	0x10U /* coherent, run shorter than the settle or hold window */
#define G5P_RU_RT_PM 0x20U /* TA receipt only: the PM state is not saved */

enum g5p_ru_rt_how {
	G5P_RU_RT_PENDING,
	G5P_RU_RT_IDLE_CLEAR, /* rule (a) */
	G5P_RU_RT_IDLE_STUCK, /* rule (b) */
	G5P_RU_RT_HOLD, /* rule (c) */
};

struct g5p_ru_retire {
	u64 settle_ns; /* minimum coherent run before rule (b); 0 = none */
	u64 hold_ns; /* rule (c): max(settle_ns, G5P_RU_RT_HOLD_MS) */
	u64 run_ns; /* first sample of the current coherent run; 0 = none */
	u64 coherent_ns; /* first coherent sample of the phase */
	u64 event_ns; /* first sample that saw the matching event latched */
	u64 busy_ns; /* first sample with an idle word set */
	u64 clear_ns; /* first all-clear sample after busy_ns */
	u32 run; /* samples in the current coherent run */
	u32 breaks; /* coherent runs that ended without a decision */
	u32 unsat; /* G5P_RU_RT_* of the last sample */
	u32 how; /* enum g5p_ru_rt_how of the last sample */
	u32 decided; /* enum g5p_ru_rt_how of the sample that decided COMPLETE */
};

static inline bool g5p_ru_rt_settle_valid(u32 settle_ms)
{
	return settle_ms <= G5P_RU_RT_MAX_SETTLE_MS;
}

static inline void g5p_ru_rt_start(struct g5p_ru_retire *t, u32 settle_ms)
{
	u32 hold_ms = settle_ms > G5P_RU_RT_HOLD_MS ? settle_ms :
						      G5P_RU_RT_HOLD_MS;

	*t = (struct g5p_ru_retire){
		.settle_ns = (u64)settle_ms * 1000000ULL,
		.hold_ns = (u64)hold_ms * 1000000ULL,
	};
}

/* The firmware's 3D completion is a type-1 event with notification bit 1 (engine 1) whose
 * words 8/9 are the channel endpoint pair (2 seq - 1, 2 seq): 1/2, 3/4, 5/6, 7/8 for
 * combined107 seq 1..4.  The grid wraps at G5P_RU_RT_GRID, so the pair is compared modulo
 * the grid whether the firmware reports it wrapped or as a running count: endpoint 0
 * (seq 8) matches 15/0 and 15/16 alike.
 */
static inline bool g5p_ru_rt_event_match(const u32 words[11], u32 endpoint)
{
	return words[0] == 1 && (words[1] & 2) &&
	       words[9] % G5P_RU_RT_GRID == endpoint % G5P_RU_RT_GRID &&
	       (words[9] - words[8]) % G5P_RU_RT_GRID == 1;
}

/* One RAM sample of the 3D phase at now_ns.  Returns the rule that completes it (enum
 * g5p_ru_rt_how); the caller passes `!= G5P_RU_RT_PENDING` to g5p_ru_wait_step()/close().
 */
static inline u32 g5p_ru_rt_step(struct g5p_ru_retire *t, u64 now_ns,
				 bool consumed, bool retired, bool idle,
				 bool event)
{
	t->unsat = (consumed ? 0 : G5P_RU_RT_CONSUMED) |
		   (retired ? 0 : G5P_RU_RT_RETIRED) |
		   (idle ? 0 : G5P_RU_RT_IDLE) | (event ? 0 : G5P_RU_RT_EVENT);
	if (event && !t->event_ns)
		t->event_ns = now_ns;
	if (!idle && !t->busy_ns)
		t->busy_ns = now_ns;
	if (idle && t->busy_ns && !t->clear_ns)
		t->clear_ns = now_ns;
	t->how = G5P_RU_RT_PENDING;
	if (!consumed || !retired) {
		if (t->run)
			t->breaks++;
		t->run = 0;
		t->run_ns = 0;
		return t->how;
	}
	if (!t->coherent_ns)
		t->coherent_ns = now_ns;
	if (!t->run++)
		t->run_ns = now_ns;
	if (idle)
		t->how = G5P_RU_RT_IDLE_CLEAR;
	else if (event && now_ns - t->run_ns >= t->settle_ns)
		t->how = G5P_RU_RT_IDLE_STUCK;
	else if (now_ns - t->run_ns >= t->hold_ns)
		t->how = G5P_RU_RT_HOLD;
	else
		t->unsat |= G5P_RU_RT_SETTLE;
	return t->how;
}

/* After g5p_ru_wait_step()/close() of the same sample: records which rule decided. */
static inline void g5p_ru_rt_note(struct g5p_ru_retire *t, u32 result)
{
	if (result == G5P_RU_WAIT_COMPLETE && !t->decided)
		t->decided = t->how;
}

/* ms from `from_ns` to `to_ns` for the receipt; -1 when either is missing or out of order. */
static inline s64 g5p_ru_rt_ms(u64 from_ns, u64 to_ns)
{
	if (!from_ns || !to_ns || to_ns < from_ns)
		return -1;
	return (s64)((to_ns - from_ns) / 1000000ULL);
}

/*
 * tahang1: the TA phase.  mcfold1 desktop seq 251 (after T1) and seq 339 (one client): ring
 * consumed at 2 ms, the firmware's type-1 TA completion event decoded at 7.7-8.4 ms, both
 * stamps, the W words and the watermarks at the job and the PM state saved, yet a TA idle word
 * (G+0x724/0x728/0x72c) set from 7-8 ms on never cleared, and the r74b predicate (which needs
 * the idle words zero) timed out at 60 s (`unsat=0x4`).  On 49 of 241 good TAs of the same
 * boot that word was set at 7-12 ms and cleared 2 ms later.  The 3D engine shows the same lag
 * (rule (b) above).
 *
 * The TA phase is complete when the target is coherent (ring consumer == target and
 * g5p_ru_retired()) with the PM state saved, and
 *   (a) the idle words are zero (the flown r74b predicate), or
 *   (b) a type-1 TA completion event was decoded after this job's TA publish, or
 *   (c) the target has stayed coherent, unbroken, for G5P_RU_TA_HOLD_MS.
 * The stamps carry the job's identity; the TA event does not (its words are the same every
 * job), it only reports that the firmware finished a TA after the publish.  Rule (c) keeps a
 * lost or late event from holding a job whose stamps already landed.
 */
#define G5P_RU_TA_HOLD_MS 500U

enum g5p_ru_ta_how {
	G5P_RU_TA_PENDING,
	G5P_RU_TA_IDLE_CLEAR, /* rule (a) */
	G5P_RU_TA_EVENT, /* rule (b) */
	G5P_RU_TA_HOLD, /* rule (c) */
	G5P_RU_TA_HOWS,
};

struct g5p_ru_ta {
	u64 hold_ns; /* rule (c) */
	u64 run_ns; /* first sample of the current coherent run */
	u64 coherent_ns; /* first coherent sample of the phase; 0 = none */
	u64 event_ns; /* first sample that saw the event latched; 0 = none */
	u32 run; /* samples in the current coherent run */
	u32 breaks; /* coherent runs that ended without a decision */
	u32 unsat; /* G5P_RU_RT_* of the last sample (SETTLE: run below the hold) */
	u32 how; /* enum g5p_ru_ta_how of the last sample */
	u32 decided; /* enum g5p_ru_ta_how of the sample that decided COMPLETE */
};

static inline void g5p_ru_ta_start(struct g5p_ru_ta *t)
{
	*t = (struct g5p_ru_ta){ .hold_ns =
					 (u64)G5P_RU_TA_HOLD_MS * 1000000ULL };
}

/* The firmware's TA completion: a type-1 event with notification bit 0 (engine 0). */
static inline bool g5p_ru_ta_event_match(const u32 words[11])
{
	return words[0] == 1 && (words[1] & 1);
}

/* One RAM sample of the TA phase at now_ns.  Returns the rule that completes it (enum
 * g5p_ru_ta_how); the caller passes `!= G5P_RU_TA_PENDING` to g5p_ru_wait_step()/close().
 */
static inline u32 g5p_ru_ta_step(struct g5p_ru_ta *t, u64 now_ns, bool consumed,
				 bool retired, bool pm, bool idle, bool event)
{
	t->unsat = (consumed ? 0 : G5P_RU_RT_CONSUMED) |
		   (retired ? 0 : G5P_RU_RT_RETIRED) |
		   (idle ? 0 : G5P_RU_RT_IDLE) | (event ? 0 : G5P_RU_RT_EVENT) |
		   (pm ? 0 : G5P_RU_RT_PM);
	if (event && !t->event_ns)
		t->event_ns = now_ns;
	t->how = G5P_RU_TA_PENDING;
	if (!consumed || !retired || !pm) {
		if (t->run)
			t->breaks++;
		t->run = 0;
		return t->how;
	}
	if (!t->coherent_ns)
		t->coherent_ns = now_ns;
	if (!t->run++)
		t->run_ns = now_ns;
	if (idle)
		t->how = G5P_RU_TA_IDLE_CLEAR;
	else if (event)
		t->how = G5P_RU_TA_EVENT;
	else if (now_ns - t->run_ns >= t->hold_ns)
		t->how = G5P_RU_TA_HOLD;
	else
		t->unsat |= G5P_RU_RT_SETTLE;
	return t->how;
}

/* After g5p_ru_wait_step()/close() of the same sample: records which rule decided. */
static inline void g5p_ru_ta_note(struct g5p_ru_ta *t, u32 result)
{
	if (result == G5P_RU_WAIT_COMPLETE && !t->decided)
		t->decided = t->how;
}

#endif

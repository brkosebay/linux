/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * TYPED_V3 serial wait policy.  Pure functions, no kernel services beyond the
 * integer types, so the host suite compiles this file as it is.
 *
 * Evidence behind the numbers (r96 raw timelines, 2026-10-30):
 *   - the natural TA completion event arrived +12.0060 s and +12.0062 s after the
 *     publish in the two boots that a 12000 ms cap turned into timeouts; the whole
 *     delay is inside the firmware's start23, and once it returns the hardware and
 *     the retirement take less than ~1 ms;
 *   - nothing the host does shortens the delay, so the wait is RAM-only: after the
 *     DC3 attach the host touches no register until the job has ended.
 *
 * Policy:
 *   - one phase (TA, then 3D) waits at most `bound` from its own publish: 20 s by
 *     default, never below 20 s, never above the absolute cap;
 *   - the whole job (TA publish to the end of the 3D phase) never exceeds the
 *     absolute cap of 30 s;
 *   - the deadline is decided on RAM samples.  A completion seen in the sample that
 *     reaches the deadline wins; a deadline is only final after one closing sample
 *     taken at or after it, which closes the gap between the last poll and the
 *     expiry; a completion after that is evidence only (`late_after`), never the
 *     result;
 *   - a fault wins over a completion in the same sample and over the deadline.
 */
#ifndef G5P_RENDER_USER_WAIT_H
#define G5P_RENDER_USER_WAIT_H

#define G5P_RU_V3_WAIT_MS 20000U /* default and minimum per-phase bound */
#define G5P_RU_V3_CAP_MS \
	30000U /* absolute job bound, and the largest per-phase bound */

/*
 * latency101 (default off: render_user_soft_ms = render_user_hard_ms = 0 leaves every number above,
 * and the behaviour of every function below, exactly as it was).  When on, the per-phase deadline
 * is the HARD bound and the job bound is two hard bounds; the SOFT bound marks a phase "slow"
 * (RAM-only record, no print, no latch).  A silent event stream means a dead firmware from the
 * phase's publish on (faultiso; before it, only from the soft bound).  See
 * docs/gpu/experiments/prod98c-bimodal-latency.md and docs/gpu/experiments/faultiso.md.
 */
#define G5P_RU_LAT_MIN_SOFT_MS \
	20000U /* never below the proven per-phase bound */
#define G5P_RU_LAT_DEF_SOFT_MS 45000U
#define G5P_RU_LAT_MAX_HARD_MS \
	60000U /* largest hard bound; the job bound is twice this */
#define G5P_RU_LAT_DEF_OBS_MS \
	2000U /* RAM-only look after a timeout, pre-receipt */
#define G5P_RU_LAT_MAX_OBS_MS 5000U
#define G5P_RU_LAT_SILENT_MS \
	1000U /* no event decoded for this long in a phase = silent */
/* The silence clock runs from the publish (the host suites key on this). */
#define G5P_RU_LAT_SILENT_FROM_PUBLISH 1

enum g5p_ru_wait_result {
	G5P_RU_WAIT_PENDING,
	G5P_RU_WAIT_COMPLETE,
	G5P_RU_WAIT_TIMEOUT,
	G5P_RU_WAIT_FAULT,
};

struct g5p_ru_wait {
	u64 start_ns; /* the publish (producer store) of this phase */
	u64 deadline_ns; /* start + bound, clamped to the job's absolute deadline */
	u64 end_ns; /* the sample that decided */
	u32 result;
	bool expired; /* a sample at or after the deadline has been seen */
	bool late_gap; /* completion first seen by the closing sample */
	bool late_after; /* completion seen after a timeout was decided: evidence only */
	/* latency101; all zero (and never read) unless soft_ns != 0 */
	u64 soft_ns; /* start + soft bound; 0 = the grace is off */
	u64 soft_seen_ns; /* the first sample at or after soft_ns */
	u64 late_ns; /* the first retired sample after the timeout was decided */
	u64 event_ns; /* sample time of the last change of the decoded-event counter */
	u64 busy_ns; /* the first sample whose idle words showed the engine busy */
	u32 events; /* the decoded-event counter at that sample */
	bool slow; /* the phase passed its soft bound and is still pending */
	bool silent; /* the event stream stopped during the phase: firmware dead */
	bool late_soft; /* completed after the soft bound: a success, never a failure */
};

/* latency101 parameters.  Off is hard = soft = 0.  On: hard in [soft, 60000], soft (0 =
 * min(45000, hard)) in [20000, hard], obs <= 5000.  The job bound the waiters must outlast is two
 * hard bounds plus one observation window.
 */
static inline u32 g5p_ru_lat_soft(u32 soft_ms, u32 hard_ms)
{
	if (soft_ms)
		return soft_ms;
	return hard_ms > G5P_RU_LAT_DEF_SOFT_MS ? G5P_RU_LAT_DEF_SOFT_MS :
						  hard_ms;
}

static inline bool g5p_ru_lat_valid(u32 soft_ms, u32 hard_ms, u32 obs_ms)
{
	if (!hard_ms)
		return !soft_ms;
	soft_ms = g5p_ru_lat_soft(soft_ms, hard_ms);
	return hard_ms <= G5P_RU_LAT_MAX_HARD_MS &&
	       soft_ms >= G5P_RU_LAT_MIN_SOFT_MS && soft_ms <= hard_ms &&
	       obs_ms <= G5P_RU_LAT_MAX_OBS_MS;
}

static inline u32 g5p_ru_lat_job_ms(u32 hard_ms, u32 obs_ms)
{
	return 2 * hard_ms + obs_ms;
}

/* A per-phase bound is valid when it is in [G5P_RU_V3_WAIT_MS, G5P_RU_V3_CAP_MS]. */
static inline bool g5p_ru_v3_bound_valid(u32 ms)
{
	return ms >= G5P_RU_V3_WAIT_MS && ms <= G5P_RU_V3_CAP_MS;
}

/* The deadline of a phase: its own bound, clamped to the job's absolute deadline.
 * A job deadline of 0 means "no job deadline yet" (the first phase).
 */
static inline u64 g5p_ru_wait_deadline(u64 start_ns, u32 bound_ms,
				       u64 job_deadline_ns)
{
	u64 d = start_ns + (u64)bound_ms * 1000000ULL;

	return job_deadline_ns && job_deadline_ns < d ? job_deadline_ns : d;
}

static inline void g5p_ru_wait_start(struct g5p_ru_wait *w, u64 start_ns,
				     u32 bound_ms, u64 job_deadline_ns)
{
	*w = (struct g5p_ru_wait){
		.start_ns = start_ns,
		.deadline_ns = g5p_ru_wait_deadline(start_ns, bound_ms,
						    job_deadline_ns),
	};
}

/*
 * latency101: hard_ms is the per-phase deadline (clamped to the job's deadline as above),
 * soft_ms
 * (never past the deadline) the point where a still-pending phase is called slow.  `events` seeds
 * the silence clock.
 */
static inline void g5p_ru_wait_start_lat(struct g5p_ru_wait *w, u64 start_ns,
					 u32 soft_ms, u32 hard_ms,
					 u64 job_deadline_ns, u32 events)
{
	g5p_ru_wait_start(w, start_ns, hard_ms, job_deadline_ns);
	w->soft_ns = start_ns + (u64)soft_ms * 1000000ULL;
	if (w->soft_ns > w->deadline_ns)
		w->soft_ns = w->deadline_ns;
	w->events = events;
	w->event_ns = start_ns;
}

/* The first sample whose idle words (G+0x724..) show the engine busy, i.e. the firmware's
 * start path has run.  Recorded once; it splits a late start (busy_ns close to the
 * completion) from slow hardware (busy_ns early).  A busy window shorter than the poll
 * period can be missed: 0 = not seen.
 */
static inline void g5p_ru_wait_busy(struct g5p_ru_wait *w, u64 now_ns,
				    bool busy)
{
	if (busy && !w->busy_ns)
		w->busy_ns = now_ns;
}

/* One look at the decoded-event counter, taken with every RAM sample, before g5p_ru_wait_step().
 * The ALIVE stream runs at about 290 events/s whatever the work does (r96c: 29442 polls, no gap
 * over 3.4 ms; latency101: 287 events/s through every stall of 0.55-33 s), so a stream that has
 * been silent for G5P_RU_LAT_SILENT_MS is a dead firmware, not a slow one, at any point of the
 * phase.  Returns true then; the caller reports it as a fault.  The silence clock starts at the
 * publish (vpds rung vk, 2026-11-01: the counter froze 1 s after the TA's publish, and a clock
 * that only ran from the soft bound called it at 45 s, with every other client held behind it).
 * The caller drains the ring before each sample, so a host that slept longer than the window
 * sees the events it missed and is not silent.  Never true when latency101 is off.
 */
static inline bool g5p_ru_wait_grace(struct g5p_ru_wait *w, u64 now_ns,
				     u32 events)
{
	if (!w->soft_ns)
		return false;
	if (events != w->events) {
		w->events = events;
		w->event_ns = now_ns;
	}
	if (now_ns >= w->soft_ns && !w->soft_seen_ns)
		w->soft_seen_ns = now_ns;
	w->silent = now_ns - w->event_ns >=
		    (u64)G5P_RU_LAT_SILENT_MS * 1000000ULL;
	return w->silent;
}

/* One RAM sample taken at `now_ns`.  Returns the result so far (PENDING until
 * decided).  A sample at or after the deadline only marks the wait expired unless
 * it shows completion or a fault; the caller must then take one closing sample and
 * call g5p_ru_wait_close().
 */
static inline u32 g5p_ru_wait_step(struct g5p_ru_wait *w, u64 now_ns,
				   bool complete, bool fault)
{
	if (w->result)
		return w->result;
	if (w->soft_ns && now_ns >= w->soft_ns)
		w->slow = true;
	if (fault) {
		w->result = G5P_RU_WAIT_FAULT;
		w->end_ns = now_ns;
	} else if (complete) {
		w->result = G5P_RU_WAIT_COMPLETE;
		w->end_ns = now_ns;
		w->late_soft = w->slow;
	} else if (now_ns >= w->deadline_ns) {
		w->expired = true;
	}
	return w->result;
}

/* The closing sample, taken after the loop left because of the deadline.  This is
 * the only place a timeout is decided.
 */
static inline u32 g5p_ru_wait_close(struct g5p_ru_wait *w, u64 now_ns,
				    bool complete, bool fault)
{
	if (w->result)
		return w->result;
	if (fault) {
		w->result = G5P_RU_WAIT_FAULT;
	} else if (complete) {
		w->result = G5P_RU_WAIT_COMPLETE;
		w->late_gap = true;
		w->late_soft = w->slow;
	} else {
		w->result = G5P_RU_WAIT_TIMEOUT;
	}
	w->end_ns = now_ns;
	return w->result;
}

/* A sample after the result was decided: a completion after a timeout is recorded
 * and nothing else happens (no 3D, no recovery, no register access).
 */
static inline void g5p_ru_wait_after(struct g5p_ru_wait *w, bool complete)
{
	if (w->result == G5P_RU_WAIT_TIMEOUT && complete)
		w->late_after = true;
}

/* latency101: the RAM-only observation window after a timeout was decided, before the receipt is
 * printed and the terminal latch is taken.  g5p_ru_wait_after() looks once, microseconds after the
 * decision; a completion that arrives milliseconds later (prod98c seq 3: 5.49 ms) was never
 * recorded, so `late_after=0/0` was a false negative.  Returns true while the window is open.  A
 * retired sample ends it and is recorded in late_after/late_ns; it never changes the result (a
 * deadline stays final).  obs_ms = 0 closes it at once (the legacy behaviour).
 */
static inline bool g5p_ru_wait_observe(struct g5p_ru_wait *w, u64 now_ns,
				       bool complete, u32 obs_ms)
{
	if (w->result != G5P_RU_WAIT_TIMEOUT || w->late_after)
		return false;
	if (complete) {
		w->late_after = true;
		w->late_ns = now_ns;
		return false;
	}
	return now_ns < w->end_ns + (u64)obs_ms * 1000000ULL;
}

#endif

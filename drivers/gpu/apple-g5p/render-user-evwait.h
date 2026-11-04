/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * The phase wait's pause between two RAM samples (perf60-fillrate.md §11.3 cut 1): woken by the
 * firmware event, not a fixed 1-2 ms sleep.  Host test: scripts/gpu/perf60-host/test_evwait.c.
 *
 * Protocol (g5p_rv3_wait):  drain -> seen = event_serial -> sample RAM -> decide -> pause.
 * The IRQ thread's drain decodes each firmware event, bumps event_serial under status_lock and
 * wakes the queue.  The pause sleeps until event_serial != seen or the safety timeout, so an
 * event decoded at any point after `seen` was read (before, during or after the sample) ends the
 * pause at once: no lost wakeup.  The timeout is the safety net for a RAM state that changes
 * without an event; the predicate stays level-triggered on every sample, as before.
 * Once the phase's own completion event is latched (s->event) but the RAM rules still wait
 * (idle words, retire stamp, PM receipt land a little after the event), the pause is a short
 * G5P_EW_POST_US poll: there is no further event to wait for.
 */
#ifndef G5P_RENDER_USER_EVWAIT_H
#define G5P_RENDER_USER_EVWAIT_H

#define G5P_EW_POST_US \
	100U /* after the phase's own event: completion is imminent */
#define G5P_EW_MAX_US 20000U /* safety timeout ceiling */

enum g5p_ew_act {
	G5P_EW_LEGACY, /* render_user_event_wake 0: usleep_range(1000, 2000), release8 */
	G5P_EW_SHORT, /* the phase's event seen: sleep G5P_EW_POST_US */
	G5P_EW_EVENT, /* sleep until any event since `seen`, at most *wait_us */
};

static inline enum g5p_ew_act g5p_ew_plan(u32 knob_us, bool phase_event,
					  u32 *wait_us)
{
	if (!knob_us) {
		*wait_us = 0;
		return G5P_EW_LEGACY;
	}
	if (phase_event) {
		*wait_us = G5P_EW_POST_US;
		return G5P_EW_SHORT;
	}
	*wait_us = knob_us < G5P_EW_POST_US ? G5P_EW_POST_US :
		   knob_us > G5P_EW_MAX_US  ? G5P_EW_MAX_US :
					      knob_us;
	return G5P_EW_EVENT;
}

/* The wait condition: an event was decoded since `seen` was read. */
static inline bool g5p_ew_woken(u64 seen, u64 serial)
{
	return serial != seen;
}

#endif

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Release of a closed render context (docs/gpu/multi-client-design.md, stage 2).
 *
 * A TYPED_V3 context whose handle is gone (DESTROY_CONTEXT or file close) gets one close item.
 * It runs after the context's last job, in the job order, and decides from plain values what
 * the firmware may still hold:
 *
 *   RELEASE  the firmware never held the pair: drop the references, free everything;
 *   UNBIND   the pair is bound and every use retired: RAM-only DC3 unbind
 *            (g5p_rv3_unbind_slots()), then free on success, retain on failure;
 *   RETAIN   the firmware may still use the pair or its memory: keep the context, its
 *            slots and its memory until reboot (the kexec quiesce detaches them then).
 *
 * The rule follows iOS AGXGart_detachSlotsAndDestroyAddressSpace (firmware-re README round 15
 * §1, "Linux sequence" step 8): stop new jobs, wait for every engine use, publish PA and tag
 * zero, consume the DC3, and only then recycle the slots and free the translations.  A slot
 * returns to the allocator only after its unbind was consumed and verified, so an old unbind
 * can never clear a new owner's PA, and the lowest free pair may be reused at once (iOS pops
 * a freed slot off its free stack first).
 *
 * Pure function, host-tested (scripts/gpu/multiclient2-host/test_render_close.c).
 */
#ifndef G5P_RENDER_USER_CLOSE_H
#define G5P_RENDER_USER_CLOSE_H

enum g5p_ru_close_plan {
	G5P_RU_CLOSE_RELEASE,
	G5P_RU_CLOSE_UNBIND,
	G5P_RU_CLOSE_RETAIN,
};

/* Why a bound or published pair is retained (bits). */
#define G5P_RU_CLOSE_UNVERIFIED \
	0x01U /* a bind was published but never checked out */
#define G5P_RU_CLOSE_BAD \
	0x02U /* a job of this context failed on the hardware */
#define G5P_RU_CLOSE_TERMINAL 0x04U /* the epoch noted a timeout or fault */
#define G5P_RU_CLOSE_FAILED 0x08U /* the device failed */
#define G5P_RU_CLOSE_STOPPED 0x10U /* the device stopped (teardown, quiesce) */
#define G5P_RU_CLOSE_SHUTDOWN \
	0x20U /* admission closed for good (reboot quiesce) */
#define G5P_RU_CLOSE_QUEUED 0x40U /* a job of this context is still queued */
#define G5P_RU_CLOSE_PENDING 0x80U /* an earlier DC batch was never consumed */
#define G5P_RU_CLOSE_LOCK 0x100U /* the command lock could not be taken */

struct g5p_ru_close_in {
	bool retained; /* the firmware may hold the pair: a DC3 bind was published */
	bool bound; /* that bind was consumed and checked */
	bool bad, term_failed, dev_failed, stopped, shutdown, dc_pending;
	u32 queued;
};

static inline const char *g5p_ru_close_name(int plan)
{
	return plan == G5P_RU_CLOSE_RELEASE ? "release" :
	       plan == G5P_RU_CLOSE_UNBIND  ? "unbind" :
					      "retain";
}

/* The plan, and in *why the G5P_RU_CLOSE_* bits that made it RETAIN (0 otherwise). */
static inline int g5p_ru_close_plan(const struct g5p_ru_close_in *in, u32 *why)
{
	u32 w = 0;

	*why = 0;
	if (!in->retained)
		return in->queued ? (*why = G5P_RU_CLOSE_QUEUED,
				     G5P_RU_CLOSE_RETAIN) :
				    G5P_RU_CLOSE_RELEASE;
	if (!in->bound)
		w |= G5P_RU_CLOSE_UNVERIFIED;
	if (in->bad)
		w |= G5P_RU_CLOSE_BAD;
	if (in->term_failed)
		w |= G5P_RU_CLOSE_TERMINAL;
	if (in->dev_failed)
		w |= G5P_RU_CLOSE_FAILED;
	if (in->stopped)
		w |= G5P_RU_CLOSE_STOPPED;
	if (in->shutdown)
		w |= G5P_RU_CLOSE_SHUTDOWN;
	if (in->queued)
		w |= G5P_RU_CLOSE_QUEUED;
	if (in->dc_pending)
		w |= G5P_RU_CLOSE_PENDING;
	*why = w;
	return w ? G5P_RU_CLOSE_RETAIN : G5P_RU_CLOSE_UNBIND;
}

#endif

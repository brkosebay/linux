/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * A client job that faults (docs/gpu/experiments/faultiso.md).  Pure decisions, host-tested
 * (scripts/gpu/faultiso-host/test_fault_iso.c).
 *
 * On this firmware a GPU page fault is not survivable in place.  The firmware acknowledges the
 * MMU fault itself (R0+0x138), reports it as event 4 and parks (firmware-re README "Fault
 * ownership"); iOS answers event 4 only with its restart tail (kext 6c81dac -> 6c834a0: firmware
 * stop, context and dependency reset, shared state rebuilt, firmware boot), and has no
 * per-context ACK, channel reset or slot unbind for it.  The no-reset path iOS does have, the
 * firmware's own job kill (event 8, then device-control command 0xa), is not a page fault.  So a
 * job that stops the firmware, by a fault event or by a silent event stream, loses its context
 * and the GPU is recovered in the boot (render_user_recover), or the epoch ends.
 */
#ifndef G5P_RENDER_USER_FAULT_H
#define G5P_RENDER_USER_FAULT_H

/* The errno a lost job reports: a timeout's own, else the fault's (-EIO for a fault event or a
 * silent stream, -EPROTO, or the drain's error).  `result` is the phase's G5P_RU_WAIT_*.
 */
static inline int g5p_ru_lost_errno(u32 result, int err)
{
	return result == G5P_RU_WAIT_TIMEOUT ? -ETIMEDOUT : err;
}

/*
 * Does a job that failed after touching the hardware fail the device too?  The legacy profiles
 * always do.  A TYPED_V3 job does not when it timed out (nothing is wrong with the protocol and
 * the firmware may still finish), when its bind failed (the DC ring is not on the path of the
 * contexts already bound), or when it was lost (the executor ruled: the channel drains, or a
 * recovery replaces the epoch).  Its context is quarantined in every case.
 */
static inline bool g5p_ru_job_fails_device(bool v3, int ret, bool bind_failed,
					   bool lost)
{
	return !v3 || (ret != -ETIMEDOUT && !bind_failed && !lost);
}

/*
 * A device failure noted while a job or a drain runs (a fault event decoded mid-phase), or while
 * a recovery is pending, is not yet ruled on: the executor either loses that job's context and
 * recovers, or ends the epoch.  Until then the other contexts wait (submit accepted and held,
 * VM_BIND busy, context status not lost) instead of failing.  The executor sets `recovering`
 * before it clears `executing`, so a reader that loads `executing` with acquire and then
 * `recovering` always sees one of them through a recovery.
 */
static inline bool g5p_ru_ruling(bool executing, bool recovering)
{
	return executing || recovering;
}

/* render_user_test_fault_job: the context's job about to start (it has run `jobs` before) gets
 * the unmapped control stream.  Never the epoch's first job: its attach and bell are special.
 */
static inline bool g5p_ru_fault_due(u32 fault_job, u32 jobs, bool first)
{
	return fault_job && jobs + 1 == fault_job && !first;
}

#endif

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * In-boot GPU recovery: the firmware's recovery packet and the recovery decision.  Pure,
 * host-tested (scripts/gpu/perf60-host/test_recover.c).
 *
 * The recovery packet is firmware event type 4 (44 bytes).  The AGX kext's event drain
 * (AGXFirmware_drainFirmwareEventRing 6c99788, case 4) reads +0x0c (the faulting slot, 0x80 for
 * -1), +0x10 (a flag, accel+0x1af8 bit 3), +0x18, +0x20 and +0x24 (accel+0x1a1c/0x1af4/0x1af0),
 * marks a recovery pending (accel+0x1a24 = 2) and calls IOGPUScheduler_signalHardwareError(sched,
 * 1) (6c67b4c): "GPURestartSignaled", the scheduler enqueues one GPU restart (a second signal
 * before it runs is "Redundant hardware error").  The restart is
 * AGXMetaFirmware_stopForRecoveryOptionalPowerCycle (6ca1e40) and
 * AGXMetaFirmware_rebootAfterRecovery (6ca1efc), which panics only if the firmware does not boot
 * again.  There is no per-boot limit: every recovery packet restarts the GPU.  The
 * kext does not read +0x04.
 *
 * On A9/Linux the slot matches the phase the host waited in: 0 on every TA fault, 1 on the 3D fault
 * of cf7 (2026-11-02); +0x20 = 8 and +0x24 = 0x81818105 (TA) / 0x81810581 (3D) so far.
 */
#ifndef G5P_RENDER_USER_RECOVER_H
#define G5P_RENDER_USER_RECOVER_H

#define G5P_FW_EVENT_RECOVERY 4
#define G5P_FW_RECOVERY_NOSLOT 0x80

struct g5p_fw_recovery {
	u32 code; /* +0x04, not read by the kext */
	u32 slot; /* +0x0c, G5P_FW_RECOVERY_NOSLOT for -1 */
	u32 flag; /* +0x10 != 0 */
	u32 w18, w20, w24;
};

/* `w` is the 11-word event entry, type 4. */
static inline struct g5p_fw_recovery g5p_fw_recovery_decode(const u32 *w)
{
	struct g5p_fw_recovery r = {
		.code = w[1],
		.slot = w[3] == 0xffffffffU ? G5P_FW_RECOVERY_NOSLOT : w[3],
		.flag = w[4] != 0,
		.w18 = w[6],
		.w20 = w[8],
		.w24 = w[9],
	};

	return r;
}

/*
 * May a job that stopped the firmware (a recovery packet, a silent event stream, a hard timeout)
 * be recovered from in this boot, the job's context lost and every other kept?
 *
 *   budget == 0      never (render_user_recover 0: the epoch ends, as before);
 *   retired/closed   never (the boot is shutting the GPU down);
 *   streak_max == 0  the old rule: `budget` recoveries per boot, never on the epoch's first job;
 *   streak_max != 0  Apple's rule, every time, with two guards: at most streak_max recoveries in a
 *                    row with no job completing between them (a GPU that faults again before it
 *                    finishes anything is not recovering), and at most G5P_RU_RECOVER_BOOT_MAX per
 *                    boot (recover.inc leaves each old epoch allocated, ~0.8 MiB: 50 x 16 KiB pages
 *                    and its struct, so a stale pointer still reads valid memory; this bounds that
 *                    to ~26 MiB).  The boot's first epoch's first job stays terminal (a failed
 *                    bring-up, not a client); the first job of a later epoch (the attach after a
 *                    recovery) is recoverable like any other.
 */
#define G5P_RU_RECOVER_BOOT_MAX 32

struct g5p_ru_recover_in {
	bool first; /* the job runs its epoch's attach */
	bool retired; /* the GPU was retired (kexec / reboot quiesce) */
	bool closed; /* admission closed */
	u32 recoveries; /* recoveries this boot */
	u32 streak; /* recoveries since the last job completed */
	u32 budget; /* render_user_recover */
	u32 streak_max; /* render_user_recover_streak */
};

static inline bool g5p_ru_recoverable(const struct g5p_ru_recover_in *in)
{
	if (!in->budget || in->retired || in->closed)
		return false;
	if (!in->streak_max)
		return !in->first && in->recoveries < in->budget;
	if (in->first && !in->recoveries)
		return false;
	return in->streak < in->streak_max &&
	       in->recoveries < G5P_RU_RECOVER_BOOT_MAX;
}

/* Why g5p_ru_recoverable() said no (for the terminal line), or "-" when it said yes. */
static inline const char *
g5p_ru_recover_refusal(const struct g5p_ru_recover_in *in)
{
	if (!in->budget)
		return "recover-off";
	if (in->retired || in->closed)
		return "shutdown";
	if (!in->streak_max)
		return in->first		    ? "first-job" :
		       in->recoveries >= in->budget ? "per-boot-budget" :
						      "-";
	if (in->first && !in->recoveries)
		return "first-job";
	if (in->streak >= in->streak_max)
		return "streak";
	return in->recoveries >= G5P_RU_RECOVER_BOOT_MAX ? "boot-max" : "-";
}

#endif

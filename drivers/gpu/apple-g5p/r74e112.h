/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_R74E112_H
#define G5P_R74E112_H

#ifndef G5P_GPC_ON
#define G5P_GPC_ON(raw) (((raw) & 0x1f) == 1) /* core.inc overrides it */
#endif

/*
 * Wake112: the R74E GPC1 power proof when ALIVE preempts the on-poll.
 *
 * R74E (boot.inc) requests GPC=1 and polls R1+0x1000 until the state field reads 1.
 * The poll stops as soon as ALIVE is decoded: after ALIVE only the first-ALIVE owner
 * may read R1.  On a warm, kexec-cycled boot the firmware can reach ALIVE before the
 * poll's first read (Wake111 20261031T201022Z: off_ns=1500 on_ns=375 raw=ffffffff,
 * the U32_MAX "not read" sentinel).  Wake111 then latched a power abort although the
 * state was never sampled.
 *
 * Wake112 defers that one proof to the owner instead, but only when the on-poll took
 * no sample: the poll counts its reads, and the U32_MAX raw is never taken as "not read".
 * The owner's existing first-ALIVE read of R1+0x1000 (g5p_r73e_irq_submit, before any
 * register write) is the proof.  State 1 sets release_ready and the unchanged cold
 * sequence continues.  Any other value (0, 0x10, 0x11, ffffffff, ...) is an absorbing
 * power abort.  An on-poll that reads state 1 never defers.  An on-poll that sampled
 * before ALIVE (0x11 in rounds 74e..79b, or any other word) and every other R74E failure
 * (off-poll, timeout, GPC1 write refused) abort exactly as before.
 *
 * States (r73b->early_proof):
 *   NONE      no deferral: the poll decided, or the release has not run
 *   DEFERRED  ALIVE preempted the on-poll; one owner read is owed (generation recorded)
 *   TAKEN     the once-owner claimed it; it settles with no exit in between
 *   PROVEN    that read had state 1: release_ready
 *   FAILED    any other value, another generation, a lost precondition, or an ALIVE
 *             wait that ended with the proof still owed: power abort, absorbing
 *
 * Pure policy: no register access, no lock, no kernel API.
 */

#define G5P_PX_STATE_MASK 0x1fU
#define G5P_PX_STATE_ON 0x01U

enum g5p_px {
	G5P_PX_NONE,
	G5P_PX_DEFERRED,
	G5P_PX_TAKEN,
	G5P_PX_PROVEN,
	G5P_PX_FAILED,
	G5P_PX_N,
};

static const char *const g5p_px_name[G5P_PX_N] = {
	"none", "deferred", "taken", "proven", "failed",
};

/* Why a deferred proof failed (NONE: it did not). */
enum g5p_px_why {
	G5P_PXW_NONE,
	G5P_PXW_STATE, /* the read's state field is not 1 */
	G5P_PXW_STALE, /* deferred in another reset generation */
	G5P_PXW_LOST, /* ALIVE or the release lost, stopped or failed since */
	G5P_PXW_RETIRED, /* a FINAL teardown forbids R1 */
	G5P_PXW_UNTAKEN, /* the ALIVE wait ended with the proof still owed */
	G5P_PXW_N,
};

static const char *const g5p_px_why_name[G5P_PXW_N] = {
	"none", "state", "stale", "lost", "retired", "untaken",
};

/* What the R74E release path latches. */
enum g5p_pxp {
	G5P_PXP_READY, /* the on-poll read state 1: release_ready (cold) */
	G5P_PXP_DEFER, /* ALIVE preempted the on-poll before its first read */
	G5P_PXP_ABORT, /* anything else: power abort (cold) */
};

/*
 * on_ran: the core request, the off-poll and the GPC1 write succeeded and the on-poll
 * ran.  ret: the last step's result.  preempted: the on-poll stopped on ALIVE.
 * samples: the on-poll's R1+0x1000 reads.  Only a preemption with no read defers (no
 * abort, no release_ready); a sampled one saw the state and aborts.
 */
static inline enum g5p_pxp g5p_px_poll(bool on_ran, int ret, bool preempted,
				       u32 samples)
{
	if (!on_ran)
		return G5P_PXP_ABORT;
	if (!ret)
		return G5P_PXP_READY;
	return preempted && !samples ? G5P_PXP_DEFER : G5P_PXP_ABORT;
}

/* The first-ALIVE owner's admission. */
enum g5p_pxg {
	G5P_PXG_READY, /* release_ready: the cold owner path */
	G5P_PXG_PROVE, /* a deferred proof is owed: the once-owner's read decides */
	G5P_PXG_WAIT, /* nothing proven yet, or another owner holds the proof */
	G5P_PXG_REFUSE, /* power abort or a failed proof: no register access */
};

static inline enum g5p_pxg g5p_px_gate(bool power_abort, bool release_ready,
				       u32 px)
{
	if (power_abort || px == G5P_PX_FAILED)
		return G5P_PXG_REFUSE;
	if (release_ready)
		return G5P_PXG_READY;
	if (px == G5P_PX_DEFERRED)
		return G5P_PXG_PROVE;
	return G5P_PXG_WAIT;
}

/* RAM facts the once-owner samples after taking the proof, before its one read. */
struct g5p_px_pre {
	u32 gen, gen_now; /* DS108 generation at the deferral and now */
	bool alive, released, stopped, failed;
	bool r1_allowed; /* g5p_r8_r1_allowed() */
};

static inline enum g5p_px_why g5p_px_check(const struct g5p_px_pre *p)
{
	if (!p->gen || p->gen != p->gen_now)
		return G5P_PXW_STALE;
	if (!p->alive || !p->released || p->stopped || p->failed)
		return G5P_PXW_LOST;
	if (!p->r1_allowed)
		return G5P_PXW_RETIRED;
	return G5P_PXW_NONE;
}

/* The proof: only the state field 1 is power-on. */
static inline bool g5p_px_on(u32 raw)
{
	return G5P_GPC_ON(raw);
}

#endif

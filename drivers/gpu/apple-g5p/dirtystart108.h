/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_DIRTYSTART108_H
#define G5P_DIRTYSTART108_H

#include "bits-compat.h"

#ifndef G5P_GPC_ON
#define G5P_GPC_ON(raw) (((raw) & 0x1f) == 1) /* core.inc overrides it */
#endif

/*
 * G5P incoming dirty-start state machine (series gpu-recovery108, patch 0007).
 *
 * Design: docs/gpu/experiments/recovery108-driver.md section 4.
 *
 * The bootstrap is stepped by userspace (map, load, release, events,
 * control), so this is an event-driven machine, not one run: the glue feeds
 * one event per driver step with the step's result and RAM-only inputs, and
 * asks the machine at two gates (before release, at the DRM claim).  It
 * assumes nothing pristine: the origin of the power state is classified from
 * the kexec flag, the requested incoming PMGR cycle and the PMGR word, and
 * every step's own result is evidence.
 *
 *   PROBE    origin inputs: booted from kexec, kexec_gpu_cycle requested
 *   POWER    the PMGR gfx word, read once after the PM reference (PMGR read)
 *   RESET    the bounded incoming power cycle (g5p_r33_gfx_off(), one attempt
 *            per boot, only in mode 3, only before the first wrapper wake)
 *   WAKE     the Fender wake result
 *   MAPS     fresh shared state: every ring control word in range (after the wake:
 *            map needs the core id, read through the woken wrapper)
 *   LOAD     the firmware image memcmp of the RAM copy (b->copied)
 *   RELEASE  the reset-stream/release result, the R74E power result
 *   ALIVE    the first ALIVE wait result and, when read, the first-ALIVE
 *            GPC word (r73e); 0x10/0x11 is the C3 closed gate
 *   CLAIM    the DRM claim asks; answers from the ALIVE proof
 *
 * Modes (parameter dirty_start):
 *   0  observe: receipts only, both gates open (combined107 behaviour)
 *   1  gate (default): fail closed on evidence the driver itself produced (a
 *      failed or out-of-order step, a power abort, a GPC word other than
 *      0x01 at the first ALIVE, a ring out of range); a claim needs ALIVE
 *      proven in the current reset generation.  Equal to combined107 on every
 *      bootstrap that succeeds.
 *   2  strict: also refuse release and claim for an origin that is not cold
 *      and not a witnessed cycle (warm-unproven, warm-uncycled, unknown),
 *      unless a reset succeeded in this generation
 *   3  strict + reset: as 2, and the glue may spend the one RESET attempt
 *      on such an origin before the wrapper wake (destructive: PMGR write)
 *
 * Generations: gen starts at 1 at PROBE and steps once per RESET attempt.  A
 * reset voids every proof of the previous generation; ALIVE proven in an
 * earlier generation never exposes.  gen never decreases.
 *
 * Pure policy: no register access, no kernel API except scnprintf().  Each
 * event emits exactly one DS108 line through the emit callback.
 */

enum g5p_ds_ev {
	G5P_DSE_PROBE,
	G5P_DSE_POWER,
	G5P_DSE_RESET,
	G5P_DSE_WAKE,
	G5P_DSE_MAPS,
	G5P_DSE_LOAD,
	G5P_DSE_RELEASE,
	G5P_DSE_ALIVE,
	G5P_DSE_CLAIM,
	G5P_DSE_RECOVER, /* an in-boot recovery cycled the gate after a proven epoch */
	G5P_DSE_N,
};

static const char *const g5p_ds_ev_name[G5P_DSE_N] = {
	"probe", "power",   "reset", "wake",  "maps",
	"load",	 "release", "alive", "claim", "recover",
};

enum g5p_ds_origin {
	G5P_DSO_UNKNOWN, /* not probed, or the PMGR word could not be read */
	G5P_DSO_COLD, /* not booted from kexec */
	G5P_DSO_WARM_CYCLED, /* kexec, incoming cycle requested and witnessed (was_pwrgated) */
	G5P_DSO_WARM_UNPROVEN, /* kexec, cycle requested, no witness in the PMGR word */
	G5P_DSO_WARM_UNCYCLED, /* kexec, no incoming cycle requested */
	G5P_DSO_N,
};

static const char *const g5p_ds_origin_name[G5P_DSO_N] = {
	"unknown", "cold", "warm-cycled", "warm-unproven", "warm-uncycled",
};

enum g5p_ds_why {
	G5P_DSW_NONE,
	G5P_DSW_PROTOCOL, /* a step out of order (its predecessor not ok in this gen) */
	G5P_DSW_WAKE, /* the wrapper wake failed */
	G5P_DSW_MAPS, /* map failed or a ring control word out of range */
	G5P_DSW_LOAD, /* the RAM image is not the firmware (memcmp) */
	G5P_DSW_RELEASE, /* release / reset stream failed */
	G5P_DSW_POWER_ABORT, /* the R74E power poll or the first-ALIVE gate aborted */
	G5P_DSW_GPC, /* first-ALIVE GPC word not 0x01 (0x10/0x11: C3) */
	G5P_DSW_ALIVE, /* no ALIVE, or the ALIVE wait failed */
	G5P_DSW_RESET, /* the incoming power cycle failed: power state unknown */
	G5P_DSW_N,
};

static const char *const g5p_ds_why_name[G5P_DSW_N] = {
	"none",	   "protocol",	  "wake", "maps",  "load",
	"release", "power-abort", "gpc",  "alive", "reset",
};

/* Gate answers (not latched): why a gate is closed right now. */
enum g5p_ds_gate {
	G5P_DSG_OPEN,
	G5P_DSG_FAILED, /* the fail-closed latch */
	G5P_DSG_NOT_PROVEN, /* no ALIVE proof in the current generation */
	G5P_DSG_ORIGIN, /* strict: origin neither cold nor a witnessed cycle */
	G5P_DSG_STATE, /* the claim snapshot contradicts the proof */
	G5P_DSG_N,
};

static const char *const g5p_ds_gate_name[G5P_DSG_N] = {
	"open", "failed", "not-proven", "origin", "state",
};

#define G5P_DS_MODE_MAX 3
#define G5P_DS_GPC_ON 0x01u
#define G5P_DS_PMGR_PWRGATED \
	BIT(8) /* sticky was_pwrgated history bit of 0x80340 */

#define G5P_DSF_RINGS_BUSY \
	BIT(0) /* a ring not empty at map (data, not a failure) */
#define G5P_DSF_ALIVE_LATE \
	BIT(1) /* ALIVE first seen at the claim (no ALIVE event) */
#define G5P_DSF_RESET_RAN BIT(2) /* a RESET attempt ran this boot */
#define G5P_DSF_RESET_OK \
	BIT(3) /* ... and succeeded (gen of the success = gen) */
/* a second attempt or one after the wake was refused */
#define G5P_DSF_RESET_REFUSED BIT(4)
#define G5P_DSF_DUP BIT(5) /* an event repeated (recorded, not a failure) */
#define G5P_DSF_PMGR_UNREAD BIT(6) /* the PMGR word could not be read */

/* One flat input record; each event reads only its own fields (RAM or step results). */
struct g5p_ds_in {
	bool kexec; /* PROBE: chosen linux,booted-from-kexec */
	bool cycle_requested; /* PROBE: apple_pmgr.kexec_gpu_cycle=1 on the cmdline */
	bool pmgr_ok; /* POWER: the PMGR read succeeded */
	u32 pmgr_raw; /* POWER: the gfx word */
	int ret; /* RESET/WAKE/MAPS/LOAD/RELEASE/ALIVE: the step's return */
	u32 rings_bad; /* MAPS: control words > 255 */
	u32 rings_busy; /* MAPS: rings not empty */
	int power_ret; /* RELEASE: R74E power result */
	u32 power_raw; /* RELEASE: its last R1+0x1000 sample */
	bool power_abort; /* RELEASE/ALIVE/CLAIM: r73b early_power_abort */
	bool gpc_valid; /* ALIVE: the r73e first-ALIVE word was read */
	u32 gpc; /* ALIVE: that word */
	bool alive, released, failed,
		control_complete; /* ALIVE/CLAIM snapshot */
};

struct g5p_ds {
	u32 gen; /* reset generation, monotonic, 0 before PROBE */
	u32 seen, ok; /* event bitmasks, current generation */
	u32 flags;
	enum g5p_ds_origin origin;
	bool kexec, cycle; /* PROBE inputs, kept for POWER */
	u32 pmgr_raw;
	u32 resets; /* RESET attempts this boot (at most one runs) */
	u32 reset_ok_gen; /* gen in which a RESET succeeded, 0 never */
	u32 alive_gen; /* gen in which ALIVE was proven, 0 never */
	bool failed; /* fail-closed latch (absorbing) */
	enum g5p_ds_why why;
	u32 events, lines, claims, refused;
	enum g5p_ds_ev last;
	enum g5p_ds_gate last_gate;
};

static inline bool g5p_ds_ok(const struct g5p_ds *d, enum g5p_ds_ev e)
{
	return d->ok & (1u << e);
}

static inline void g5p_ds_fail(struct g5p_ds *d, enum g5p_ds_why why)
{
	if (!d->failed)
		d->why = why;
	d->failed = true;
}

static inline enum g5p_ds_origin g5p_ds_origin_of(bool kexec, bool cycle,
						  bool pmgr_ok, u32 raw)
{
	if (!pmgr_ok)
		return G5P_DSO_UNKNOWN;
	if (!kexec)
		return G5P_DSO_COLD;
	if (!cycle)
		return G5P_DSO_WARM_UNCYCLED;
	return (raw & G5P_DS_PMGR_PWRGATED) ? G5P_DSO_WARM_CYCLED :
					      G5P_DSO_WARM_UNPROVEN;
}

/* The origin is trusted (cold, witnessed cycle) or a reset succeeded in this generation. */
static inline bool g5p_ds_origin_trusted(const struct g5p_ds *d)
{
	return d->origin == G5P_DSO_COLD || d->origin == G5P_DSO_WARM_CYCLED ||
	       (d->reset_ok_gen && d->reset_ok_gen == d->gen);
}

/* Should the glue spend the one RESET attempt now (mode 3, before the wake)? */
static inline bool g5p_ds_reset_wanted(const struct g5p_ds *d, u32 mode)
{
	return mode >= 3 && !d->failed && !d->resets &&
	       !(d->seen & (1u << G5P_DSE_WAKE)) &&
	       (d->seen & (1u << G5P_DSE_POWER)) && !g5p_ds_origin_trusted(d);
}

/* The release gate (asked by the glue right before the reset stream runs). */
static inline enum g5p_ds_gate g5p_ds_release_gate(const struct g5p_ds *d,
						   u32 mode)
{
	if (!mode)
		return G5P_DSG_OPEN;
	if (d->failed)
		return G5P_DSG_FAILED;
	if (mode >= 2 && !g5p_ds_origin_trusted(d))
		return G5P_DSG_ORIGIN;
	return G5P_DSG_OPEN;
}

/* The ALIVE proof the claim needs: proven in this generation, nothing failed since. */
static inline bool g5p_ds_proven(const struct g5p_ds *d)
{
	return !d->failed && d->alive_gen && d->alive_gen == d->gen &&
	       g5p_ds_ok(d, G5P_DSE_RELEASE) && g5p_ds_ok(d, G5P_DSE_LOAD);
}

static inline int g5p_ds_line(struct g5p_ds *d, char *buf, size_t n,
			      enum g5p_ds_ev e, int ret, u32 a, u32 b)
{
	d->lines++;
	return scnprintf(buf, n,
		"DS108 gen=%u ev=%s ret=%d origin=%s failed=%u why=%s seen=%03x ok=%03x flags=%02x pmgr=%08x a=%x b=%x gate=%s",
		d->gen, g5p_ds_ev_name[e], ret, g5p_ds_origin_name[d->origin],
		d->failed, g5p_ds_why_name[d->why], d->seen, d->ok, d->flags,
		d->pmgr_raw, a, b, g5p_ds_gate_name[d->last_gate]);
}

/*
 * Feed one event.  Returns, for CLAIM, 0 (expose) or -EAGAIN/-EIO (refuse);
 * for every other event 0.  mode is the current dirty_start parameter; at mode 0
 * the claim answers 0 whatever the state (the legacy driver decides alone).
 * line/n receive the event's receipt (always written).
 */
static inline int g5p_ds_event(struct g5p_ds *d, enum g5p_ds_ev e,
			       const struct g5p_ds_in *in, u32 mode, char *line,
			       size_t n)
{
	u32 bit = 1u << e, a = 0, b = 0;
	int ret = 0;

	if (mode > G5P_DS_MODE_MAX)
		mode = G5P_DS_MODE_MAX;
	d->events++;
	d->last = e;
	if ((d->seen & bit) && e != G5P_DSE_CLAIM)
		d->flags |= G5P_DSF_DUP;
	switch (e) {
	case G5P_DSE_PROBE:
		/* The only event that may start a generation: probe runs once per boot. */
		if (!d->gen)
			d->gen = 1;
		d->origin = G5P_DSO_UNKNOWN;
		a = in->kexec;
		b = in->cycle_requested;
		d->kexec = in->kexec;
		d->cycle = in->cycle_requested;
		d->seen |= bit;
		d->ok |= bit;
		break;
	case G5P_DSE_POWER:
		if (!(d->seen & (1u << G5P_DSE_PROBE))) {
			g5p_ds_fail(d, G5P_DSW_PROTOCOL);
			break;
		}
		d->origin = g5p_ds_origin_of(d->kexec, d->cycle, in->pmgr_ok,
					     in->pmgr_raw);
		if (!in->pmgr_ok)
			d->flags |= G5P_DSF_PMGR_UNREAD;
		d->pmgr_raw = in->pmgr_ok ? in->pmgr_raw : 0;
		a = in->pmgr_ok;
		d->seen |= bit;
		d->ok |= bit;
		break;
	case G5P_DSE_RESET:
		a = in->ret;
		if (d->resets || (d->seen & (1u << G5P_DSE_WAKE))) {
			/* The glue must not have run it; record a refused attempt and fail
			 * closed.
			 */
			d->flags |= G5P_DSF_RESET_REFUSED;
			g5p_ds_fail(d, G5P_DSW_PROTOCOL);
			break;
		}
		d->resets++;
		d->flags |= G5P_DSF_RESET_RAN;
		/* A reset voids the previous generation's proofs (power epoch boundary). */
		d->gen++;
		/* Unreachable today (a reset runs before the wake, so before every later
		 * step); kept so that a reordered glue cannot carry a proof across it.
		 */
		d->seen &= (1u << G5P_DSE_PROBE) | (1u << G5P_DSE_POWER);
		d->ok &= (1u << G5P_DSE_PROBE) | (1u << G5P_DSE_POWER);
		d->seen |= bit;
		if (in->ret) {
			g5p_ds_fail(d, G5P_DSW_RESET);
			break;
		}
		d->ok |= bit;
		d->flags |= G5P_DSF_RESET_OK;
		d->reset_ok_gen = d->gen;
		break;
	case G5P_DSE_WAKE:
		d->seen |= bit;
		if (in->ret)
			g5p_ds_fail(d, G5P_DSW_WAKE);
		else
			d->ok |= bit;
		break;
	case G5P_DSE_MAPS:
		d->seen |= bit;
		a = in->rings_bad;
		b = in->rings_busy;
		if (in->rings_busy)
			d->flags |= G5P_DSF_RINGS_BUSY;
		/* map needs the core id, read through the woken wrapper */
		if (!g5p_ds_ok(d, G5P_DSE_WAKE))
			g5p_ds_fail(d, G5P_DSW_PROTOCOL);
		else if (in->ret || in->rings_bad)
			g5p_ds_fail(d, G5P_DSW_MAPS);
		else
			d->ok |= bit;
		break;
	case G5P_DSE_LOAD:
		d->seen |= bit;
		if (!g5p_ds_ok(d, G5P_DSE_MAPS))
			g5p_ds_fail(d, G5P_DSW_PROTOCOL);
		else if (in->ret)
			g5p_ds_fail(d, G5P_DSW_LOAD);
		else
			d->ok |= bit;
		break;
	case G5P_DSE_RELEASE:
		d->seen |= bit;
		a = (u32)in->power_ret;
		b = in->power_raw;
		if (!g5p_ds_ok(d, G5P_DSE_LOAD))
			g5p_ds_fail(d, G5P_DSW_PROTOCOL);
		else if (in->power_abort || in->power_ret)
			g5p_ds_fail(d, G5P_DSW_POWER_ABORT);
		else if (in->ret)
			g5p_ds_fail(d, G5P_DSW_RELEASE);
		else
			d->ok |= bit;
		break;
	case G5P_DSE_ALIVE:
		d->seen |= bit;
		a = in->gpc_valid;
		b = in->gpc;
		if (!g5p_ds_ok(d, G5P_DSE_RELEASE)) {
			g5p_ds_fail(d, G5P_DSW_PROTOCOL);
		} else if (in->power_abort) {
			g5p_ds_fail(d, G5P_DSW_POWER_ABORT);
		} else if (in->gpc_valid && !G5P_GPC_ON(in->gpc)) {
			g5p_ds_fail(d, G5P_DSW_GPC);
		} else if (in->ret || !in->alive || in->failed) {
			g5p_ds_fail(d, G5P_DSW_ALIVE);
		} else {
			d->ok |= bit;
			d->alive_gen = d->gen;
		}
		break;
	case G5P_DSE_RECOVER:
		/* The render frontend's in-boot recovery (recover.inc): the gate was cycled
		 * after this generation's release, with the firmware frozen.  Like a RESET it
		 * starts a generation that voids every proof of the last one, and it is a
		 * witnessed cycle for the origin gate; the bootstrap's steps then run again
		 * and must each succeed in the new generation.  Not after a failure, and not
		 * before a release: there is no epoch to recover then.
		 */
		a = in->ret;
		if (d->failed || !g5p_ds_ok(d, G5P_DSE_RELEASE)) {
			g5p_ds_fail(d, G5P_DSW_PROTOCOL);
			break;
		}
		d->gen++;
		d->seen &= (1u << G5P_DSE_PROBE) | (1u << G5P_DSE_POWER);
		d->ok &= (1u << G5P_DSE_PROBE) | (1u << G5P_DSE_POWER);
		d->seen |= bit;
		/* The old ALIVE proof is void; the claim's late path may prove the new one. */
		d->alive_gen = 0;
		if (in->ret) {
			g5p_ds_fail(d, G5P_DSW_RESET);
			break;
		}
		d->ok |= bit;
		d->reset_ok_gen = d->gen;
		break;
	case G5P_DSE_CLAIM:
		d->claims++;
		d->seen |= bit;
		a = in->alive | in->released << 1 | in->failed << 2 |
		    in->control_complete << 3 | in->power_abort << 4;
		/* ALIVE reached through a path without the ALIVE event: accept the RAM
		 * latch as the proof only after a successful release of this generation.
		 */
		if (!d->failed && !d->alive_gen && in->alive && !in->failed &&
		    !in->power_abort && g5p_ds_ok(d, G5P_DSE_RELEASE)) {
			d->alive_gen = d->gen;
			d->ok |= 1u << G5P_DSE_ALIVE;
			d->flags |= G5P_DSF_ALIVE_LATE;
		}
		if (!mode)
			d->last_gate = G5P_DSG_OPEN;
		else if (d->failed)
			d->last_gate = G5P_DSG_FAILED;
		else if (!g5p_ds_proven(d))
			d->last_gate = G5P_DSG_NOT_PROVEN;
		else if (!in->alive || !in->released || in->failed ||
			 in->power_abort || !in->control_complete)
			d->last_gate = G5P_DSG_STATE;
		else if (mode >= 2 && !g5p_ds_origin_trusted(d))
			d->last_gate = G5P_DSG_ORIGIN;
		else
			d->last_gate = G5P_DSG_OPEN;
		if (d->last_gate == G5P_DSG_OPEN) {
			d->ok |= bit;
		} else {
			d->refused++;
			ret = d->last_gate == G5P_DSG_FAILED ? -EIO : -EAGAIN;
		}
		break;
	default:
		break;
	}
	g5p_ds_line(d, line, n, e, e == G5P_DSE_CLAIM ? ret : in->ret, a, b);
	return ret;
}



static inline void g5p_ds_init(struct g5p_ds *d)
{
	*d = (struct g5p_ds){ .last = G5P_DSE_N };
}

#endif

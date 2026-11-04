/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * The GPU quiesce before a kexec, reboot or power-off (docs/gpu/experiments/mcfold2-kexec.md).
 *
 * Without it the firmware keeps running from the outgoing kernel's RAM across the relocation:
 * it still writes its event rings, and the MMU still holds a CBASE for every bound render slot
 * pointing at page tables the next kernel reuses, until the next kernel's PMGR GFX cycle about
 * 0.4 s into its boot.  The quiesce ends that window on the outgoing side, in this order:
 *
 *   1 close job admission and wait (bounded) for submitters inside;
 *   2 take the command lock (bounded trylock), so no job or command is in flight;
 *   3 DETACH: every user slot the firmware may hold is unbound by a RAM-only DC3 batch
 *     (render-user-bind.h), only when the lock is ours, the firmware is healthy, no timeout
 *     or fault was noted and the DC ring is idle;
 *   4 freeze the boot state and mask the GPU IRQ (no host register access after this);
 *   5 CYCLE: one bounded PMGR GFX off/on, the same gate cycle the next kernel performs with
 *     apple_pmgr.kexec_gpu_cycle=1, only when no host thread can be inside a GPU register
 *     access: the lock is ours, or its holder is the TYPED_V3 executor in its RAM-only regime.
 *
 * Pure functions over the ops below, host-tested (scripts/gpu/mcfold2-host/test_kexec_quiesce.c).
 */
#ifndef G5P_RENDER_USER_QUIESCE_H
#define G5P_RENDER_USER_QUIESCE_H

/* kexec_quiesce module parameter */
#define G5P_RU_KQ_OFF 0U
#define G5P_RU_KQ_RAM 1U /* detach only, no PMGR access */
#define G5P_RU_KQ_FULL 2U /* detach and cycle (default) */

/* Plan */
#define G5P_RU_KQ_DETACH 0x1U
#define G5P_RU_KQ_CYCLE 0x2U

/* Why a step was skipped */
#define G5P_RU_KQ_WHY_OFF 0x001U /* kexec_quiesce=0 */
#define G5P_RU_KQ_WHY_DOWN 0x002U /* the firmware never ran on this boot */
#define G5P_RU_KQ_WHY_BUSY 0x004U /* the command lock is not ours: no detach */
#define G5P_RU_KQ_WHY_FAILED \
	0x008U /* fault or protocol failure: firmware state unknown */
#define G5P_RU_KQ_WHY_WEDGED \
	0x010U /* a TYPED_V3 timeout or fault: a job may hold its slots */
#define G5P_RU_KQ_WHY_RING 0x020U /* the DC ring is not idle */
#define G5P_RU_KQ_WHY_NONE 0x040U /* no user slot is held */
#define G5P_RU_KQ_WHY_HOLDER \
	0x080U /* unknown lock holder, maybe inside MMIO: no cycle */
#define G5P_RU_KQ_WHY_NO_PMGR 0x100U
#define G5P_RU_KQ_WHY_RAM 0x200U /* kexec_quiesce=1: no cycle */
#define G5P_RU_KQ_WHY_NO_DRM 0x400U /* no render frontend: nobody can detach */

struct g5p_ru_kq_in {
	u32 mode; /* kexec_quiesce */
	bool booted; /* the firmware was released on this boot */
	bool locked; /* the command lock is ours */
	bool failed; /* b->failed or b->event_failed */
	bool frontend; /* a render frontend that can detach is registered */
	bool term_failed; /* the TYPED_V3 latch noted a timeout or fault */
	bool ram_only; /* the TYPED_V3 latch is quiet: its executor touches no register */
	bool executing; /* the TYPED_V3 executor is inside a job */
	bool ring_idle; /* DC consumer == producer <= 255 */
	bool pmgr; /* the PMGR regmap is present */
	u32 held; /* user slots with a PA or a host tag */
	u32 cons, prod; /* the DC ring when sampled */
};

static inline u32 g5p_ru_kq_plan(const struct g5p_ru_kq_in *in, u32 *why)
{
	u32 plan = 0, w = 0;

	if (in->mode == G5P_RU_KQ_OFF || !in->booted) {
		*why = in->mode == G5P_RU_KQ_OFF ? G5P_RU_KQ_WHY_OFF :
						   G5P_RU_KQ_WHY_DOWN;
		return 0;
	}
	if (!in->locked)
		w |= G5P_RU_KQ_WHY_BUSY;
	if (in->failed)
		w |= G5P_RU_KQ_WHY_FAILED;
	if (in->term_failed)
		w |= G5P_RU_KQ_WHY_WEDGED;
	if (!in->ring_idle)
		w |= G5P_RU_KQ_WHY_RING;
	if (!in->held)
		w |= G5P_RU_KQ_WHY_NONE;
	if (!in->frontend)
		w |= G5P_RU_KQ_WHY_NO_DRM;
	if (!w)
		plan |= G5P_RU_KQ_DETACH;
	if (in->mode < G5P_RU_KQ_FULL)
		w |= G5P_RU_KQ_WHY_RAM;
	else if (!in->pmgr)
		w |= G5P_RU_KQ_WHY_NO_PMGR;
	else if (!in->locked && !(in->executing && in->ram_only))
		w |= G5P_RU_KQ_WHY_HOLDER;
	else
		plan |= G5P_RU_KQ_CYCLE;
	*why = w;
	return plan;
}

struct g5p_ru_kq_ops {
	void *ctx;
	u32 (*admit_close)(void *ctx); /* close admission; submitters still inside */
	bool (*lock)(void *ctx); /* bounded trylock of the command lock */
	void (*unlock)(void *ctx);
	void (*inputs)(void *ctx, struct g5p_ru_kq_in *in); /* RAM only */
	int (*detach)(void *ctx); /* RAM only */
	void (*freeze)(void *ctx); /* RAM only */
	void (*irq_off)(void *ctx); /* AIC ack, mask, wait for the handler */
	int (*cycle)(void *ctx); /* PMGR only */
};

struct g5p_ru_kq_out {
	struct g5p_ru_kq_in in;
	u32 inside, plan, why;
	int detach_ret, cycle_ret;
	bool froze;
};

/* One quiesce.  detach_ret/cycle_ret stay 0 for a step that did not run (see plan). */
static inline void g5p_ru_kq_run(const struct g5p_ru_kq_ops *o, u32 mode,
				 struct g5p_ru_kq_out *out)
{
	*out = (struct g5p_ru_kq_out){ .in.mode = mode };
	if (mode == G5P_RU_KQ_OFF) {
		out->why = G5P_RU_KQ_WHY_OFF;
		return;
	}
	out->inside = o->admit_close(o->ctx);
	out->in.locked = o->lock(o->ctx);
	o->inputs(o->ctx, &out->in);
	out->plan = g5p_ru_kq_plan(&out->in, &out->why);
	if (out->plan & G5P_RU_KQ_DETACH)
		out->detach_ret = o->detach(o->ctx);
	if (out->in.booted) {
		o->freeze(o->ctx);
		o->irq_off(o->ctx);
		out->froze = true;
	}
	if (out->plan & G5P_RU_KQ_CYCLE)
		out->cycle_ret = o->cycle(o->ctx);
	if (out->in.locked)
		o->unlock(o->ctx);
}

#endif

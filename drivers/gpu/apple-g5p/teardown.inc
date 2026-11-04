// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Kernel glue for the GPU teardown state machine (teardown.h).
 * Design, evidence and device plan: docs/gpu/experiments/gpu-warm-kexec-hang-proof.md
 *
 * Included from core.inc after lifecycle.inc, so it may use g5p_dc_root_word(),
 * g5p_fender_write()/poll() and g5p_r73b_disarm(). No state is added to struct
 * g5p_boot or struct g5p_diag: one GPU, one file-static teardown instance.
 *
 * Compiled with W=1 against a9/tri96 (patches/gpu-recovery/README.md); NOT run on hardware.
 *
 * gpu-recovery108 (0006): every run goes through the retire layer (retire108.h):
 * admission close, RAM-only epoch class and a TR108 receipt before the ladder, the
 * named post-ladder slots, and the retired latch after a FINAL.  The TD98 lines are
 * unchanged.  docs/gpu/experiments/recovery108-driver.md.
 */
#include <linux/limits.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include "teardown.h"
#include "retire108.h"
#include "render-user-bind.h"
#include "render-user-quiesce.h"

/* 0 off (default; the notifier and .shutdown do nothing), 1 observe-only receipts
 * (CAPTURE: RAM reads, no change), 2 full ladder (FINAL).
 */

/* gpu-recovery108 G6 slot: one R1 read of isStableOff after a verified Fender stop leg. */

/* Defined in render-r96.inc (same translation unit, included later by render-fixed.inc). */

static struct g5p_td g5p_td_state;
static struct g5p_diag *g5p_td_dev;
static void (*g5p_td_detach_cb)(void *cookie);
static void *g5p_td_detach_cookie;

/* gpu-recovery108: the retire layer around the ladder (retire108.h), one GPU, file static. */
static struct g5p_r8 g5p_r8_state = { .admit_us = G5P_R8_ADMIT_US };
static void (*g5p_r8_epoch_cb)(void *cookie, struct g5p_r8_in *in);
static void *g5p_r8_epoch_cookie;

/* The DRM frontend reports its share of the epoch inputs (claim, holders, queued jobs), RAM only. */
static void __maybe_unused g5p_r8_register_epoch(void (*cb)(void *,
							    struct g5p_r8_in *),
						 void *cookie)
{
	g5p_r8_epoch_cookie = cookie;
	smp_store_release(&g5p_r8_epoch_cb, cb);
}

/* Job admission: a submitter enters before its first look at driver state and
 * leaves on every return; false once a FINAL teardown closed admission.
 */
static bool __maybe_unused g5p_r8_enter(void)
{
	return g5p_r8_admit_enter(&g5p_r8_state.admit);
}

static void __maybe_unused g5p_r8_exit(void)
{
	g5p_r8_admit_exit(&g5p_r8_state.admit);
}

/* The post-retire register predicate for the R1 helpers outside the ladder. */
static bool __maybe_unused g5p_r8_r1_allowed(void)
{
	return g5p_r8_mmio_ok(&g5p_r8_state, G5P_TDD_R1);
}

static bool __maybe_unused g5p_r8_is_retired(void)
{
	return g5p_r8_retired(&g5p_r8_state);
}

/* The DRM frontend registers the callback that fails its queued/in-flight job
 * fences (exactly once each, -ENODEV) without touching hardware.
 */
static void __maybe_unused g5p_td_register_detach(void (*cb)(void *),
						  void *cookie)
{
	g5p_td_detach_cookie = cookie;
	smp_store_release(&g5p_td_detach_cb, cb);
}

static u64 g5p_td_op_now(void *ctx)
{
	return ktime_get_mono_fast_ns();
}

static void g5p_td_op_relax(void *ctx, u32 us)
{
	usleep_range(us, us + us / 4 + 1);
}

static void g5p_td_op_freeze(void *ctx)
{
	struct g5p_diag *g = ctx;
	struct g5p_boot *b = g->boot;

	if (!b)
		return;
	/* The IRQ-side plateau banks (g5p_r80_irq_bank/g5p_r85_irq_bank) read R0 until this
	 * latch is set; arm it before anything else (RAM only, no-op without an r96 config).
	 */
	/* Same freeze as g5p_boot_park(): nothing may publish after this. */
	if (b->r73b)
		g5p_r73b_disarm(b->r73b);
	WRITE_ONCE(b->stopped, true);
	WRITE_ONCE(b->alive, false);
}

static int g5p_td_op_ring_read(void *ctx, u32 idx, u32 *cons, u32 *prod)
{
	static const u32 slots[G5P_TD_NRING] = { 0x60, 0x64, 0x68, 0x6c, 0x70 };
	struct g5p_boot *b = ((struct g5p_diag *)ctx)->boot;
	u32 *ctl;

	if (!b || !READ_ONCE(b->maps_ready))
		return -ENODEV;
	ctl = (u32 *)g5p_boot_shared(b, slots[idx]);
	*cons = READ_ONCE(ctl[0]);
	*prod = READ_ONCE(ctl[4]);
	dma_rmb();
	return 0;
}

/* RAM only. Consumes ALIVE (0) and stamp (1) entries; the first other type is
 * left in place for the host, as g5p_boot_drain() does.
 */
static int g5p_td_op_event_pass(void *ctx, u32 max, u32 *consumed, u32 *left,
				bool *fault)
{
	struct g5p_boot *b = ((struct g5p_diag *)ctx)->boot;
	u32 *control = (u32 *)g5p_boot_shared(b, 0x70);
	u32 consumer, producer, n = 0;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&b->event_lock, flags);
	consumer = READ_ONCE(control[0]);
	producer = READ_ONCE(control[4]);
	if (consumer > 255 || producer > 255) {
		ret = -EPROTO;
		goto out;
	}
	dma_rmb();
	while (n < max && consumer != producer) {
		u32 off = b->layout.offset[0x74] + consumer * 44;

		if (g5p_le32(b->shared[off / G5P_GPU_PAGE].cpu +
			     off % G5P_GPU_PAGE) > 1) {
			*fault = true;
			break;
		}
		consumer = (consumer + 1) & 255;
		mb();
		WRITE_ONCE(control[0], consumer);
		n++;
	}
	producer = READ_ONCE(control[4]);
	*left = producer > 255 ? 255 : (producer - consumer) & 255;
out:
	*consumed = n;
	spin_unlock_irqrestore(&b->event_lock, flags);
	return ret;
}

/* The render path latches terminal states through g5p_td_note_terminal(); until
 * prod98 wires that call this reports "not seen" and the race is only visible as
 * ring state.
 */
static bool g5p_td_op_terminal_seen(void *ctx)
{
	return READ_ONCE(g5p_td_state.terminal) != 0;
}

static void g5p_td_op_detach(void *ctx)
{
	void (*cb)(void *) = smp_load_acquire(&g5p_td_detach_cb);

	if (cb)
		cb(g5p_td_detach_cookie);
}

static void g5p_td_op_aic_ack(void *ctx)
{
	struct g5p_boot *b = ((struct g5p_diag *)ctx)->boot;

	/* The same software-source ACK the IRQ-cap path uses. */
	if (b && b->irq_ready && b->swirq)
		writel(0x4000, b->swirq + 0x94);
}

static void g5p_td_op_irq_mask(void *ctx)
{
	struct g5p_boot *b = ((struct g5p_diag *)ctx)->boot;

	/* Line masked at the interrupt controller; the handler stays registered for
	 * a late entry, which still ACKs. Same call as the IRQ-cap path.
	 */
	if (b && b->irq_ready && b->irq > 0)
		disable_irq_nosync(b->irq);
}

static u32 g5p_td_op_irq_total(void *ctx)
{
	struct g5p_boot *b = ((struct g5p_diag *)ctx)->boot;

	return b ? (u32)atomic_read(&b->irqs) : 0;
}

static int g5p_td_op_fender_stop(void *ctx)
{
	struct g5p_diag *g = ctx;
	const struct g5p_fender_io io = {
		.ctx = g,
		.write = g5p_fender_write,
		.poll = g5p_fender_poll,
	};

	/* Apple stopFirmwareForPowerDown 6c9fa10; no retry, no PMGR off. */
	return g5p_fender_idle(&io);
}

static int g5p_td_op_pmgr(void *ctx, u32 *target, u32 *actual)
{
	struct g5p_diag *g = ctx;
	u32 raw;
	int ret;

	if (!g->pmgr)
		return -ENODEV;
	ret = regmap_read(g->pmgr, 0x80340, &raw);
	if (!ret) {
		*target = raw & 0xf;
		*actual = (raw >> 4) & 0xf;
	}
	return ret;
}

/* dev_emerg so the receipt reaches netconsole at any console loglevel. */
static void g5p_td_op_emit(void *ctx, const char *line)
{
	dev_emerg(((struct g5p_diag *)ctx)->dev, "%s\n", line);
}

static void g5p_td_pre_fill(struct g5p_diag *g, struct g5p_td_pre *p,
			    bool locked)
{
	struct g5p_boot *b = READ_ONCE(g->boot);

	memset(p, 0, sizeof(*p));
	p->lock_held = locked;
	p->powered = READ_ONCE(g->powered);
	p->fender_ready = READ_ONCE(g->fender_ready);
	if (!b)
		return;
	p->booted = READ_ONCE(b->maps_ready);
	p->released = READ_ONCE(b->released);
	p->already_parked = READ_ONCE(b->parked);
	p->stopped_unparked = READ_ONCE(b->stopped) && !READ_ONCE(b->parked);
	p->power_abort = b->r73b && READ_ONCE(b->r73b->early_power_abort);
	/* RAM-only read of the host-written init word (same as g5p_dc_snapshot()). */
	p->policy_host_owned =
		p->booted &&
		!g5p_dc_root_word(b, b->layout.offset[0x4d] + 0x6c8);
}

/* gpu-recovery108: the ladder as the retire layer's td_run op.  The TD98 begin line
 * and the ladder's own lines are exactly the recovery98/99 ones.
 */
struct g5p_r8_ctx {
	struct g5p_diag *g;
	const struct g5p_td_ops *ops;
	struct g5p_td_pre *pre;
	const char *why;
	bool locked;
};

static int g5p_r8_op_td_run(void *ctx, enum g5p_td_mode mode)
{
	struct g5p_r8_ctx *c = ctx;

	dev_emerg(c->g->dev,
		  "TD98 begin why=%s mode=%s teardown=%u locked=%u\n", c->why,
		  g5p_td_mode_name[mode], (0), c->locked);
	return g5p_td_run(&g5p_td_state, c->ops, mode, c->pre);
}

static u32 g5p_r8_op_td_flags(void *ctx)
{
	return g5p_td_state.flags;
}

static enum g5p_td_verdict g5p_r8_op_td_verdict(void *ctx)
{
	return g5p_td_state.verdict;
}

static u64 g5p_r8_op_now(void *ctx)
{
	return ktime_get_mono_fast_ns();
}

static void g5p_r8_op_relax(void *ctx, u32 us)
{
	usleep_range(us, us + us / 4 + 1);
}

/* G6: R1+0x0 bit 30 after a verified stop leg (the slot runs only then, before RETIRE). */
static int g5p_r8_op_stable_off(void *ctx, u32 *value)
{
	struct g5p_r8_ctx *c = ctx;

	*value = readl(c->g->window[1]) >> 30;
	return 0;
}

static void g5p_r8_op_emit(void *ctx, const char *line)
{
	dev_emerg(((struct g5p_r8_ctx *)ctx)->g->dev, "%s\n", line);
}

static void g5p_r8_in_fill(struct g5p_diag *g, struct g5p_r8_in *in,
			   bool locked, enum g5p_td_mode mode)
{
	void (*cb)(void *, struct g5p_r8_in *) =
		smp_load_acquire(&g5p_r8_epoch_cb);
	struct g5p_boot *b = READ_ONCE(g->boot);

	memset(in, 0, sizeof(*in));
	in->terminal = READ_ONCE(g5p_td_state.terminal);
	/* FINAL without the lock: a worker (or a sysfs writer) holds it, maybe inside a wait. */
	in->worker_busy = mode == G5P_TD_FINAL && !locked;
	if (b) {
		in->booted = READ_ONCE(b->maps_ready);
		in->released = READ_ONCE(b->released);
		in->alive = READ_ONCE(b->alive);
		in->failed = READ_ONCE(b->failed);
		in->fault_event = READ_ONCE(b->event_failed);
		in->parked = READ_ONCE(b->parked);
		in->stopped_unparked = READ_ONCE(b->stopped) &&
				       !READ_ONCE(b->parked);
		in->power_abort = b->r73b &&
				  READ_ONCE(b->r73b->early_power_abort);
	}
	if (cb)
		cb(g5p_r8_epoch_cookie, in);
}

static int g5p_td_execute(struct g5p_diag *g, enum g5p_td_mode mode,
			  const char *why)
{
	const struct g5p_td_ops ops = {
		.ctx = g,
		.now_ns = g5p_td_op_now,
		.relax_us = g5p_td_op_relax,
		.freeze = g5p_td_op_freeze,
		.ring_read = g5p_td_op_ring_read,
		.event_pass = g5p_td_op_event_pass,
		.terminal_seen = g5p_td_op_terminal_seen,
		.detach = g5p_td_op_detach,
		.aic_ack = g5p_td_op_aic_ack,
		.irq_mask = g5p_td_op_irq_mask,
		.irq_total = g5p_td_op_irq_total,
		.fender_stop = g5p_td_op_fender_stop,
		.pmgr_actual = g5p_td_op_pmgr,
		.emit = g5p_td_op_emit,
	};
	struct g5p_td_pre pre;
	struct g5p_r8_in in;
	struct g5p_r8_ctx c = { .g = g, .ops = &ops, .pre = &pre, .why = why };
	/* fw-sleep (G2) and pmgr-gate (G1) have no mechanism: their ops stay NULL. */
	const struct g5p_r8_ops r8ops = {
		.ctx = &c,
		.now_ns = g5p_r8_op_now,
		.relax_us = g5p_r8_op_relax,
		.td_run = g5p_r8_op_td_run,
		.td_flags = g5p_r8_op_td_flags,
		.td_verdict = g5p_r8_op_td_verdict,
		.slot = { [G5P_R8S_STABLE_OFF] = g5p_r8_op_stable_off },
		.emit = g5p_r8_op_emit,
	};
	bool locked = false;
	int ret;

	/* Never queue behind a worker or sysfs writer that may be inside MMIO. CAPTURE
	 * reads RAM only and needs no lock.
	 */
	if (mode == G5P_TD_FINAL)
		locked = mutex_trylock(&g->lock);
	c.locked = locked;
	g5p_td_pre_fill(g, &pre, locked);
	g5p_r8_in_fill(g, &in, locked, mode);
	g5p_r8_state.slot_on[G5P_R8S_STABLE_OFF] = (0);
	ret = g5p_r8_run(&g5p_r8_state, &r8ops, mode, &in, why);
	if (g5p_td_state.flags & G5P_TDF_RT1_OK && g->boot)
		WRITE_ONCE(g->boot->parked, true);
	if (locked)
		mutex_unlock(&g->lock);
	return ret;
}

/* Observation only: arms the quiet predicate, changes nothing else. */
static int __maybe_unused g5p_td_capture(struct g5p_diag *g, const char *why)
{
	return g5p_td_execute(g, G5P_TD_CAPTURE, why);
}

static int g5p_td_final(struct g5p_diag *g, const char *why)
{
	return g5p_td_execute(g, G5P_TD_FINAL, why);
}

/*
 * The quiesce before a kexec, reboot or power-off (render-user-quiesce.h).  It runs from the
 * reboot notifier whatever `teardown` is, once per boot; .shutdown and the `td-kexec` command
 * reach the same run.  It leaves the GPU powered with its firmware stopped and the boot state
 * frozen: the next kernel boots it as after any warm kexec.
 */

#define G5P_KQ_LOCK_MS 2000U
#define G5P_KQ_ADMIT_US 20000U
#define G5P_KQ_OFF_MS 10U

/* The render frontend's share: its TYPED_V3 epoch inputs and the RAM-only unbind of slots. */
struct g5p_kq_frontend {
	void (*state)(void *cookie, struct g5p_ru_kq_in *in);
	int (*detach)(void *cookie, const u32 *slots, u32 n);
};

static const struct g5p_kq_frontend *g5p_kq_fe;
static void *g5p_kq_fe_cookie;
static atomic_t g5p_kq_ran = ATOMIC_INIT(0);

struct g5p_kq_ctx {
	struct g5p_diag *g;
	u32 slots[G5P_RU_USER_SLOTS];
	u32 held;
};

static void __maybe_unused g5p_kq_register(const struct g5p_kq_frontend *fe,
					   void *cookie)
{
	g5p_kq_fe_cookie = cookie;
	smp_store_release(&g5p_kq_fe, fe);
}

static u32 g5p_kq_admit_close(void *ctx)
{
	struct g5p_r8_admit *a = &g5p_r8_state.admit;
	u64 deadline = ktime_get_ns() + (u64)G5P_KQ_ADMIT_US * NSEC_PER_USEC;
	u32 inside = g5p_r8_admit_close(a);

	while (inside && ktime_get_ns() < deadline) {
		usleep_range(G5P_R8_POLL_US, 2 * G5P_R8_POLL_US);
		inside = g5p_r8_admit_inside(a);
	}
	return inside;
}

/* Never queue behind a holder that may be inside MMIO: bounded, like g5p_render_run(). */
static bool g5p_kq_lock(void *ctx)
{
	struct g5p_diag *g = ((struct g5p_kq_ctx *)ctx)->g;
	unsigned long deadline = jiffies + msecs_to_jiffies(G5P_KQ_LOCK_MS);

	while (!mutex_trylock(&g->lock)) {
		if (time_after(jiffies, deadline))
			return false;
		msleep(5);
	}
	return true;
}

static void g5p_kq_unlock(void *ctx)
{
	mutex_unlock(&((struct g5p_kq_ctx *)ctx)->g->lock);
}

/* RAM only: the boot state, the held user slots and the DC ring, then the frontend's inputs. */
static void g5p_kq_inputs(void *ctx, struct g5p_ru_kq_in *in)
{
	struct g5p_kq_ctx *c = ctx;
	struct g5p_diag *g = c->g;
	struct g5p_boot *b = READ_ONCE(g->boot);
	const struct g5p_kq_frontend *fe = smp_load_acquire(&g5p_kq_fe);
	u32 *ctl;

	in->pmgr = !!g->pmgr;
	if (!b || !READ_ONCE(b->maps_ready) || !READ_ONCE(b->released) ||
	    READ_ONCE(b->parked))
		return;
	in->booted = true;
	in->failed = READ_ONCE(b->failed) || READ_ONCE(b->event_failed);
	ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	dma_rmb();
	in->cons = READ_ONCE(ctl[0]);
	in->prod = READ_ONCE(ctl[4]);
	in->ring_idle = in->prod <= 255 && in->cons == in->prod;
	c->held = g5p_ru_held_slots(g5p_boot_shared(b, 0x86),
				    g5p_boot_shared(b, 0x84), c->slots,
				    ARRAY_SIZE(c->slots));
	in->held = c->held;
	if (fe) {
		in->frontend = true;
		fe->state(g5p_kq_fe_cookie, in);
	}
}

static int g5p_kq_detach(void *ctx)
{
	struct g5p_kq_ctx *c = ctx;
	const struct g5p_kq_frontend *fe = smp_load_acquire(&g5p_kq_fe);

	return fe ? fe->detach(g5p_kq_fe_cookie, c->slots, c->held) : -ENODEV;
}

static void g5p_kq_freeze(void *ctx)
{
	g5p_td_op_freeze(((struct g5p_kq_ctx *)ctx)->g);
}

/* The IRQ-cap path's AIC ack and mask, then wait out a handler already running (process
 * context; the threaded handler drains RAM and takes no lock the quiesce holds).
 */
static void g5p_kq_irq_off(void *ctx)
{
	struct g5p_diag *g = ((struct g5p_kq_ctx *)ctx)->g;
	struct g5p_boot *b = g->boot;

	g5p_td_op_aic_ack(g);
	g5p_td_op_irq_mask(g);
	if (b && b->irq_ready && b->irq > 0)
		synchronize_irq(b->irq);
}

/* PMGR only: the bounded gate cycle of the r33/dirty-start path (refuses a gate that is not
 * plainly on), which ends with the gate requested on.
 */
static int g5p_kq_cycle(void *ctx)
{
	struct g5p_diag *g = ((struct g5p_kq_ctx *)ctx)->g;

	g5p_mmio_gate(g);
	return g5p_r33_gfx_off(g->dev, g->pmgr, G5P_KQ_OFF_MS);
}

static int g5p_kq_run(struct g5p_diag *g, const char *why)
{
	struct g5p_ru_kq_ops o = {
		.admit_close = g5p_kq_admit_close,
		.lock = g5p_kq_lock,
		.unlock = g5p_kq_unlock,
		.inputs = g5p_kq_inputs,
		.detach = g5p_kq_detach,
		.freeze = g5p_kq_freeze,
		.irq_off = g5p_kq_irq_off,
		.cycle = g5p_kq_cycle,
	};
	struct g5p_ru_kq_out out;
	struct g5p_kq_ctx *c;
	u64 t0 = ktime_get_ns();

	if (atomic_xchg(&g5p_kq_ran, 1)) {
		dev_emerg(
			g->dev,
			"GPU-KQ why=%s already ran this boot; nothing done end=kq\n",
			why);
		return -EALREADY;
	}
	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		dev_emerg(g->dev, "GPU-KQ why=%s ret=%d end=kq\n", why,
			  -ENOMEM);
		return -ENOMEM;
	}
	c->g = g;
	o.ctx = c;
	g5p_ru_kq_run(&o, min((G5P_RU_KQ_FULL), G5P_RU_KQ_FULL), &out);
	dev_emerg(
		g->dev,
		"GPU-KQ why=%s mode=%u inside=%u locked=%u booted=%u failed=%u wedged=%u "
		"ram_only=%u exec=%u dc=%u/%u held=%u plan=%s%s skip=%#x detach=%d froze=%u "
		"cycle=%d ms=%llu end=kq\n",
		why, out.in.mode, out.inside, out.in.locked, out.in.booted,
		out.in.failed, out.in.term_failed, out.in.ram_only,
		out.in.executing, out.in.cons, out.in.prod, out.in.held,
		out.plan & G5P_RU_KQ_DETACH ? "detach" : "-",
		out.plan & G5P_RU_KQ_CYCLE ? "+cycle" : "", out.why,
		out.detach_ret, out.froze, out.cycle_ret,
		(ktime_get_ns() - t0) / NSEC_PER_MSEC);
	kfree(c);
	return out.detach_ret ?: out.cycle_ret;
}

/* sysfs `command`: td-capture / td-final / td-kexec. Runs before the command mutex, like
 * r73b-export, so a wedged stage cannot block it. -EAGAIN (clean, GPU left as
 * found) is reported as success; the verdict is in the TD98 lines.  td-kexec is the
 * reboot quiesce run by hand (the GPU is finished for this boot afterwards).
 */
static int g5p_td_command(struct g5p_diag *g, const char *command)
{
	int ret;

	if (sysfs_streq(command, "td-capture"))
		ret = g5p_td_capture(g, "sysfs");
	else if (sysfs_streq(command, "td-final"))
		ret = g5p_td_final(g, "sysfs");
	else if (sysfs_streq(command, "td-kexec"))
		ret = g5p_kq_run(g, "sysfs");
	else
		return -EINVAL;
	return ret == -EAGAIN ? 0 : ret;
}

/* Runs from kernel_restart_prepare() for reboot, halt, power-off and kexec, i.e.
 * before device_shutdown() and its unbounded wait_for_device_probe(). Panic kexec
 * skips both and is not covered.
 */
static int g5p_td_reboot_notify(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct g5p_diag *g = READ_ONCE(g5p_td_dev);

	if (!g)
		return NOTIFY_DONE;
	if (action != SYS_RESTART && action != SYS_HALT &&
	    action != SYS_POWER_OFF)
		return NOTIFY_DONE;
	/* The quiesce needs a live firmware for its detach, so it runs before the ladder freezes. */
	g5p_kq_run(g, "reboot-notifier");
	{
		return NOTIFY_DONE;
	}
}

static struct notifier_block g5p_td_nb = {
	.notifier_call = g5p_td_reboot_notify,
	.priority = INT_MAX, /* before every other reboot notifier */
};

/* g is devm-allocated and the DRM frontend's probe error path unwinds without
 * g5p_remove(): the notifier must not outlive g.
 */
static void g5p_td_devres_release(void *data)
{
	unregister_reboot_notifier(&g5p_td_nb);
	WRITE_ONCE(g5p_td_dev, NULL);
}

static void g5p_td_attach(struct g5p_diag *g)
{
	g5p_td_init(&g5p_td_state);
	/* A probe after a retire (not reachable on the built-in driver) never reopens admission. */
	if (!g5p_r8_retired(&g5p_r8_state)) {
		u32 gen = g5p_r8_state.gen;

		g5p_r8_init(&g5p_r8_state);
		g5p_r8_state.gen = gen; /* monotonic across a re-probe */
	}
	WRITE_ONCE(g5p_td_dev, g);
	register_reboot_notifier(&g5p_td_nb);
	/* On failure the action has already run: no notifier, teardown stays inert. */
	if (devm_add_action_or_reset(g->dev, g5p_td_devres_release, g))
		dev_warn(g->dev, "teardown not attached\n");
}

/* Returns true when the PM reference must be kept: a PMGR power-off was never
 * validated on the outgoing side and the stop leg is not isStableOff.
 */
static bool g5p_td_remove(struct g5p_diag *g)
{
	unregister_reboot_notifier(&g5p_td_nb);
	;
	;
	WRITE_ONCE(g5p_td_dev, NULL);
	return (0) != 0;
}

/* The .shutdown platform callback: second chance after the notifier. */
static void __maybe_unused g5p_td_shutdown(struct g5p_diag *g)
{
	/* A second chance after the notifier (a no-op once it ran). */
	if (!atomic_read(&g5p_kq_ran))
		g5p_kq_run(g, "shutdown");
	{
		;
		;
	};
}

// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * In-boot GPU recovery (docs/gpu/multi-client-design.md "Stage 2/3 as built", fault isolation).
 *
 * A render job that stopped the firmware (its event stream went silent, or a fault) blocks the
 * one TA/3D channel for every client, and the firmware has no abort.  iOS ends that state with a
 * firmware stop, a platform power cycle and a new firmware boot (kext 6c9e070, firmware-re README
 * "Fault recovery").  This is the same, built from two proven pieces:
 *   - the reboot quiesce's freeze, IRQ mask and PMGR gfx gate cycle (teardown.inc g5p_kq_*), which
 *     on hardware cycled a wedged, failed device safely (deskfold reboot notifier: plan=-+cycle,
 *     then the next kernel came up warm-cycled);
 *   - the bootstrap's own stage functions (wake, map, arm, load, release, events, control), the
 *     same code the sysfs commands run after the next kernel's cycle on every warm hop.
 * The old struct g5p_boot is frozen and left allocated (devm; a few hundred KiB per recovery),
 * so a stale pointer anywhere still reads valid memory; a new one is built by the stages.  id and
 * sample are not rerun (they refuse once the boot saw a reset transition; map needs only the
 * cached core id), nor is irq-test (refused in an r74 epoch).  The firmware image is reused.
 *
 * Core lock held, no job in flight (the render scheduler's worker).  Returns 0 with a live new
 * epoch, or an error with the device left frozen and failed.
 */

/* The bootstrap's stages after the wake, as the sysfs runner orders them. */
static const char *const g5p_recover_stages[] = {
	"map",
	"arm",
	"load",
	"release",
	"events",
	"control-prepare",
	"control-submit",
	"control-wait",
};

static int __maybe_unused g5p_recover(struct g5p_diag *g, const char *why,
				      u32 *stage_out)
{
	struct g5p_boot *old = g->boot;
	struct g5p_kq_ctx kq = { .g = g };
	u64 t0 = ktime_get_ns();
	u32 stage = 0;
	int ret;

	lockdep_assert_held(&g->lock);
	*stage_out = 0;
	if (!old || !READ_ONCE(old->released) || g5p_r8_is_retired() ||
	    g5p_r8_admit_closed(&g5p_r8_state.admit) || !g->pmgr ||
	    !g->id_checked)
		return -EPERM;
	/* The quiesce's freeze and IRQ mask, then the line is given back for the new epoch. */
	g5p_kq_freeze(&kq);
	g5p_kq_irq_off(&kq);
	if (old->irq_ready && old->irq > 0)
		devm_free_irq(g->dev, old->irq, g);
	/* mmio-access.md G5: the GPU is marked unusable before its power is cut (Apple clears
	 * fender+0xc inside setPlatformPower(0), before the gate); g5p_kq_cycle() also sets mmio_off.
	 */
	WRITE_ONCE(g->fender_ready, false);
	ret = g5p_kq_cycle(&kq);
	ret = g5p_ds_hook_recover(g, ret) ?: ret;
	if (ret)
		goto out;
	/* A new epoch: the stages build a new struct g5p_boot from here. */
	WRITE_ONCE(g->boot, NULL);
	stage = 1;
	ret = g5p_fender_wake(g);
	for (u32 i = 0; !ret && i < ARRAY_SIZE(g5p_recover_stages); i++) {
		const char *s = g5p_recover_stages[i];

		stage = i + 2;
		ret = strncmp(s, "control-", 8) ? g5p_boot_command(g, s) :
						  g5p_lifecycle_command(g, s);
		if (ret == -EALREADY)
			ret = 0;
	}
	if (!ret && (!g->boot || !READ_ONCE(g->boot->alive) ||
		     !READ_ONCE(g->boot->control_complete)))
		ret = -EIO;
	/* The claim gate again: the held jobs need ALIVE proven in the new generation. */
	if (!ret)
		ret = g5p_ds_hook_claim(g);
	if (!ret) {
		/* The render claim is kept: no diagnostic command may reset this epoch. */
		WRITE_ONCE(g->boot->user_compute_owned, true);
		stage = 0;
	}
out:
	if (ret && g->boot)
		WRITE_ONCE(g->boot->failed, true);
	g->last_result = ret;
	*stage_out = stage;
	dev_emerg(
		g->dev,
		"GPU-RECOVER why=%s ret=%d stage=%s ms=%llu gen=%u end=recover\n",
		why, ret,
		!stage	   ? (ret ? "cycle" : "done") :
		stage == 1 ? "wake" :
			     g5p_recover_stages[stage - 2],
		(ktime_get_ns() - t0) / NSEC_PER_MSEC, g5p_ds_gen());
	return ret;
}

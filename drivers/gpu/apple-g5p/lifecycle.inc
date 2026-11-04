/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Post-alive transport and park experiments; never implicit recovery. */
static const bool control_experiment = 1;
static const bool irq_experiment = 0;
static const bool teardown_experiment = 1;

/* A complete sparse image: first sample omits only zero words, later samples
 * emit every changed word (including transitions to zero). All allocated
 * root/shared memory is covered, including unknown status/trace buffers.
 * These are CPU coherent RAM observations, never proof of META cache contents.
 * IRQs can update memory concurrently; this is deliberately not an atomic dump.
 */
static u32 g5p_dc_root_word(struct g5p_boot *b, u32 offset)
{
	return READ_ONCE(*(u32 *)(b->root[offset / G5P_GPU_PAGE].cpu +
				  offset % G5P_GPU_PAGE));
}

/* nopatch: the policy record (slot 0x4a) words 0x1000d044 decides readiness from: +8 state,
 * +0xc substate, +0x34 pstate mirror; +0 and +4 (sample period) for context.  iOS keeps +8 at 0.
 */
static const u32 g5p_np_words[5] = { 0, 4, 8, 0xc, 0x34 };

static void g5p_np_read(struct g5p_boot *b, u32 *w)
{
	dma_rmb();
	for (u32 i = 0; i < ARRAY_SIZE(g5p_np_words); i++)
		w[i] = g5p_dc_root_word(b, b->layout.offset[0x4a] +
						   g5p_np_words[i]);
}

static void g5p_np_print(struct g5p_diag *g, const char *stage, u32 seq, u32 e,
			 const u32 *w)
{
	dev_info(
		g->dev,
		"NOPATCH policy stage=%s seq=%u engine=%u +0=%08x +4=%08x +8=%08x +c=%08x "
		"+34=%08x events=%u end=nopatch\n",
		stage, seq, e, w[0], w[1], w[2], w[3], w[4],
		READ_ONCE(g->boot->events));
}

/* RAM only; nothing unless the unpatched image is latched. */
static void g5p_np_policy(struct g5p_diag *g, const char *stage, u32 seq, u32 e)
{
	struct g5p_boot *b = g->boot;
	u32 w[ARRAY_SIZE(g5p_np_words)];

	if (!b || !b->r73b || !b->r73b->nopatch)
		return;
	g5p_np_read(b, w);
	g5p_np_print(g, stage, seq, e, w);
}

/* Read fixed allocated policy objects, never follow firmware-owned pointers.
 * 1000d044/1000d504 test these states;1001c900 stores requested pstate.
 * These are sampled boundary predicates, NOT entry/return breadcrumbs or PC.
 */

/* Firmware MMU recovery uses event mask200 and reads e150/e158.
 * Read raw qword halves without ACK/clear; concurrent updates can tear them.
 * An absent sampled event does not exclude an earlier firmware-handled fault.
 */
static void g5p_fault_snapshot(struct g5p_diag *g, const char *stage)
{
	u32 event, lo, hi, lo2, hi2;

	g5p_mmu_progress(g, stage, 0x130);
	event = readl(g->rgx + 0x130);
	g5p_mmu_progress(g, stage, 0xe150);
	lo = readl(g->rgx + 0xe150);
	g5p_mmu_progress(g, stage, 0xe154);
	hi = readl(g->rgx + 0xe154);
	g5p_mmu_progress(g, stage, 0xe158);
	lo2 = readl(g->rgx + 0xe158);
	g5p_mmu_progress(g, stage, 0xe15c);
	hi2 = readl(g->rgx + 0xe15c);
	g5p_mmu_record(
		g,
		"MMU %s event130=%08x pending=%u e150=%08x:%08x e158=%08x:%08x raw-split read-only\n",
		stage, event, !!(event & 0x200), hi, lo, hi2, lo2);
}

/* Same active-wrapper gate as CDM observations. Never write/ACK a register.
 * 1001c9b8 samples R0+164/160/164; bound host sampling even on rollover.
 * A split sample with different high words is explicitly unstable.
 */

static void g5p_dc_snapshot(struct g5p_diag *g, const char *stage, bool first)
{
	{
		return;
	}
}

static int g5p_control_prepare(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	u32 *work = (u32 *)g5p_boot_shared(b, 0x68);
	u32 consumer = READ_ONCE(ctl[0]), producer = READ_ONCE(ctl[4]);
	u32 ctl_meta = g5p_meta_address(g5p_object_va(&b->layout, 0x6e));
	u32 data_meta = g5p_meta_address(g5p_object_va(&b->layout, 0x6f));
	u8 *init = b->root[0].cpu;
	u8 *handoff = b->image[0].cpu + 0x3ff8;
	u32 *events = (u32 *)g5p_boot_shared(b, 0x70);

	/* c9f868/c9f884: validate the loaded boot handoff, not just its builder. */
	if (g5p_le32(handoff) != g5p_meta_address(G5P_ROOT_VA) ||
	    g5p_le32(handoff + 4) !=
		    g5p_meta_address(g5p_object_va(&b->layout, 0x5f)))
		return -EPROTO;
	/* Drain under event_lock before checking that ALIVE has been returned. */
	if (g5p_r73b_service(g, G5P_R73B_OWNER_PREPARE, true))
		return -EIO;
	if (READ_ONCE(events[0]) > 255 || READ_ONCE(events[4]) > 255)
		return -EPROTO;
	if (READ_ONCE(events[0]) != READ_ONCE(events[4]))
		return -EBUSY;
	dev_info(g->dev,
		 "DC boot handoff verified; event consumer=%u producer=%u\n",
		 READ_ONCE(events[0]), READ_ONCE(events[4]));

	;
	;
	;
	;
	/* c96be8: init+18/+1c are META addresses, never CPU/DMA pointers. */
	if (g5p_le32(init + 0x18) != ctl_meta ||
	    g5p_le32(init + 0x1c) != data_meta)
		return -EPROTO;
	dev_info(
		g->dev,
		"DC mapping ctl GPU=%llx META=%08x data GPU=%llx META=%08x; init verified\n",
		g5p_object_va(&b->layout, 0x6e), ctl_meta,
		g5p_object_va(&b->layout, 0x6f), data_meta);

	if (b->work_ready)
		return -EBUSY;
	if (b->control_ready)
		return b->r73b && 1 ? 0 : -EALREADY;
	if (consumer > 255 || producer > 255)
		return -EPROTO;
	if (consumer != producer || READ_ONCE(work[0]) != READ_ONCE(work[4]) ||
	    (b->work_submitted && !b->work_complete))
		return -EBUSY;
	/* c993f8, size=0: no payload copy, ctrl+1020=0, command={8,0,0,0,0}.
	 * No claim that this is an opcode-independent firmware NOP.
	 */
	g5p_put32(g5p_boot_shared(b, 0x5e) + 0x1020, 0);
	b->control_target = (producer + 1) & 255;
	b->control_ready = true;
	dev_info(
		g->dev,
		"DC prepare cmd=8 payload_words=0 consumer=%u producer=%u target=%u; RAM only\n",
		consumer, producer, b->control_target);
	return 0;
}

static int g5p_r73e_prepare(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	int ret;

	ret = g5p_control_prepare(g);
	if (!ret)
		WRITE_ONCE(b->r73b->fast_window, true);
	dev_info(
		g->dev,
		"R73E cmd8 prepared-before-release ret=%d target=%u end=r73e\n",
		ret, b->control_target);
	return ret;
}

/* Apple's GPC precondition (AGXMetaFender_setGPCPerformanceState, ...6ca4ccc): a request in
 * flight shows R1+0x1000 bit 4; wait for it to clear before judging the state.  R1 reads only,
 * bounded by boot_alive_settle_us; with 0 the first read stands, as before.
 */
static u32 g5p_r74e_settled(struct g5p_diag *g, struct g5p_r73b *r, u32 raw)
{
	u64 start;

	if (!r->settle_us || !(raw & 0x10))
		return raw;
	if (!r->settle_first)
		r->settle_first = raw;
	start = ktime_get_mono_fast_ns();
	while ((raw & 0x10) && ktime_get_mono_fast_ns() - start <
				       (u64)r->settle_us * NSEC_PER_USEC) {
		udelay(10);
		raw = readl(g->window[1] + 0x1000);
		r->settle_samples++;
	}
	r->settle_ns += ktime_get_mono_fast_ns() - start;
	return raw;
}

static int g5p_r73e_irq_submit(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	u32 producer = (b->control_target - 1) & 255;
	u32 off, raw;
	enum g5p_pxg gate;

	if (!b->control_ready || b->control_submitted)
		return -EPERM;
	/* wake112: release_ready, or a deferred R74E proof that the read below settles. */
	gate = g5p_px_gate(READ_ONCE(r->early_power_abort),
			   READ_ONCE(r->early_release_ready),
			   atomic_read_acquire(&r->early_proof));
	if (gate != G5P_PXG_READY && gate != G5P_PXG_PROVE)
		return -EHOSTDOWN;
	if (READ_ONCE(ctl[0]) != producer || READ_ONCE(ctl[4]) != producer)
		return -EPROTO;
	if (gate == G5P_PXG_PROVE && !g5p_r74e_take(g))
		return -EHOSTDOWN;
	r->early_state = g5p_r74e_settled(g, r, readl(g->window[1] + 0x1000));
	if (gate == G5P_PXG_PROVE)
		g5p_r74e_settle(r, r->early_state);
	if (!g5p_gpc_ok(r->early_state))
		return -EHOSTDOWN;
	/* nopatch: five RAM loads, the state the readiness gate will see before this bell. */
	if (r->nopatch)
		g5p_np_read(b, r->np_pre);
	off = b->layout.offset[0x6d] + producer * 20;
	for (u32 i = 0; i < 5; i++) {
		u32 at = off + i * 4;
		u8 *dst = b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE;

		g5p_put32(dst, i ? 0 : 8);
	}
	WRITE_ONCE(b->control_submitted, true);
	dma_wmb();
	WRITE_ONCE(ctl[4], b->control_target);
	mb();
	g5p_r73b_crumb(b, G5P_R73B_DC_PUBLISHED, G5P_R73B_OWNER_IRQ, 0);
	/* Functional pre-kick sequence only; no optional diagnostics here. */
	writel(7, g->window[1] + 0x14);
	raw = readl(g->window[1] + 0x14);
	if (raw & ~7U)
		return -EPROTO;
	r->early_after_state =
		g5p_r74e_settled(g, r, readl(g->window[1] + 0x1000));
	if (!g5p_gpc_ok(r->early_after_state))
		return -EHOSTDOWN;
	dev_emerg(
		g->dev,
		"R73E doorbell begin first-ALIVE W0+00b00 value=00000010 end=r73e\n");
	writel(0x10, g->rgx + 0xb00);
	dev_emerg(g->dev, "R73E doorbell end first-ALIVE W0+00b00 end=r73e\n");
	g5p_r73b_crumb(b, G5P_R73B_DC_KICKED, G5P_R73B_OWNER_IRQ, 0);
	return 0;
}

static int g5p_r73e_alive_submit(struct g5p_diag *g, u32 owner)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	enum g5p_pxg gate;
	u32 px;
	int ret;

	/* wake112: a deferred R74E proof admits the once-owner as release_ready does. */
	gate = g5p_px_gate(READ_ONCE(r->early_power_abort),
			   READ_ONCE(r->early_release_ready),
			   atomic_read_acquire(&r->early_proof));
	if (gate == G5P_PXG_REFUSE)
		return -EHOSTDOWN;
	if (gate == G5P_PXG_WAIT)
		return -EAGAIN;
	if (atomic_cmpxchg(&r->early_busy, 0, 1))
		return -EAGAIN;
	if (atomic_cmpxchg(&r->early_once, 0, 1)) {
		ret = READ_ONCE(r->early_ret);
		goto out;
	}
	r->early_irq_ns = ktime_get_mono_fast_ns();
	r->early_ret = g5p_r73e_irq_submit(g);
	ret = r->early_ret;
	WRITE_ONCE(r->fast_window, false);
	if (ret == -EHOSTDOWN)
		WRITE_ONCE(r->early_power_abort, true);
	dev_emerg(
		g->dev,
		"R74E ALIVE submit case=%u owner=%u state=%08x after=%08x ret=%d "
		"release_ready=%u power_abort=%u submitted=%u end=r74e\n",
		r->r73e_case, owner, r->early_state, r->early_after_state, ret,
		r->early_release_ready, r->early_power_abort,
		READ_ONCE(b->control_submitted));
	if (r->settle_first)
		dev_emerg(
			g->dev,
			"R74E settle first=%08x state=%08x after=%08x samples=%u ns=%llu limit_us=%u "
			"ret=%d end=r74e\n",
			r->settle_first, r->early_state, r->early_after_state,
			r->settle_samples, r->settle_ns, r->settle_us, ret);
	if (r->nopatch && READ_ONCE(b->control_submitted))
		g5p_np_print(g, "pre-bell", 0, 0, r->np_pre);
	px = atomic_read(&r->early_proof);
	/* raw: the proof read, or U32_MAX when it did not happen (a refusal or no take). */
	if (px != G5P_PX_NONE)
		dev_emerg(
			g->dev,
			"R74E112 proof owner=%u state=%s why=%s gen=%u/%u raw=%08x release_ready=%u "
			"power_abort=%u end=r74e112\n",
			owner, g5p_px_name[min_t(u32, px, G5P_PX_N - 1)],
			g5p_px_why_name[min_t(u32, r->early_proof_why,
					      G5P_PXW_N - 1)],
			r->early_proof_gen, r->early_proof_gen_now,
			px == G5P_PX_PROVEN ||
					r->early_proof_why == G5P_PXW_STATE ?
				r->early_state :
				U32_MAX,
			r->early_release_ready, r->early_power_abort);
out:
	atomic_set(&r->early_busy, 0);
	return ret;
}

static int g5p_control_submit(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	u32 producer = (b->control_target - 1) & 255;
	u32 state, off;
	int ret;

	if (b->control_submitted)
		return b->r73b && 1 ? 0 : -EPERM;
	if (!b->control_ready)
		return -EPERM;
	/* R73 synchronous pre-publish pass closes the race where trace records
	 * do not raise a final IRQ. A negative result is scoreable only when the
	 * factor reaches cmd8 with every outbound auxiliary consumer caught up.
	 */
	ret = g5p_r73b_service(g, G5P_R73B_OWNER_PUBLISH, true);
	if (ret)
		return ret;
	if (READ_ONCE(ctl[0]) != producer || READ_ONCE(ctl[4]) != producer)
		return -EPROTO;
	state = readl(g->window[1] + 0x1000);
	dev_info(g->dev, "DC raw R1+01000=%08x (require active before kick)\n",
		 state);
	if (!g5p_gpc_ok(state))
		return -EHOSTDOWN;
	g5p_dc_snapshot(g, "before-publish", true);
	off = b->layout.offset[0x6d] + producer * 20;
	for (u32 i = 0; i < 5; i++) {
		u32 at = off + i * 4;
		u8 *dst = b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE;

		g5p_put32(dst, i ? 0 : 8);
	}
	WRITE_ONCE(b->control_submitted, true);
	dma_wmb();
	WRITE_ONCE(ctl[4], b->control_target);
	/* ca05d4: full dmb ish after producer, before Fender and MTS. */
	mb();
	g5p_r73b_crumb(b, G5P_R73B_DC_PUBLISHED, G5P_R73B_OWNER_PUBLISH, 0);
	for (u32 i = 0; i < 5; i++) {
		u32 at = off + i * 4;
		u8 *src = b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE;

		if (g5p_le32(src) != (i ? 0 : 8))
			return -EBADMSG;
	}
	if (READ_ONCE(ctl[4]) != b->control_target)
		return -EPROTO;
	dev_info(
		g->dev,
		"DC slot verified words=00000008/0/0/0/0 producer=%u; full barrier\n",
		b->control_target);
	ret = g5p_prepare_kick(g, "DC");
	if (ret)
		return ret;
	;
	if (b->r73_aux_drain) {
		ret = g5p_r73b_service(g, G5P_R73B_OWNER_KICK, true);
		if (ret)
			return ret;
	}
	g5p_dc_snapshot(g, "before-kick", false);
	dev_info(
		g->dev,
		"DC publish cmd=8 slot=%u target=%u; W R0+00b00=%08x variant=%u\n",
		producer, b->control_target, b->dc_variant == 4 ? 0x12 : 0x10,
		b->dc_variant);
	/* 0x12 is verified for CL, NOT for DC: variant 4 tests dispatch routing. */
	writel(7 == 4 ? 0x12 : 0x10, g->rgx + 0xb00);
	g5p_r73b_crumb(b, G5P_R73B_DC_KICKED, G5P_R73B_OWNER_KICK, 0);
	g5p_dc_snapshot(g, "after-kick", false);
	return 0;
}

/* perf60 pin >100 (seed 1-5, poweron): the policy record as the firmware left it, from RAM, plus the
 * real Fender state through an R1-only ticket of the register licence (any pstate, bit 29 allowed).
 * The sampler 0x1000aa80 stores its own "now" at policy+0x918 on every call (0x1000ab44) and the
 * current state at +0x34 (0x1000ab18), so fw_now against the host's arch timer names the
 * firmware's clock, and +0x34 / +0 / +0x8f8 name the branch it took.
 */
static void g5p_perf_readback(struct g5p_diag *g, const char *why)
{
	struct g5p_boot *b = g->boot;
	u32 pol = b->layout.offset[0x4a], meta = b->layout.offset[0x4d];
	u64 host = arch_timer_read_counter(), rate = arch_timer_get_rate();
	u64 p8f8, p918, p920, m784;
	struct g5p_mmio_ticket t;

	if (!b->gpu_perf || (!b->perf_seed && !b->perf_poweron))
		return;
	dma_rmb();
	p8f8 = g5p_dc_root_word(b, pol + 0x8f8) |
	       (u64)g5p_dc_root_word(b, pol + 0x8fc) << 32;
	p918 = g5p_dc_root_word(b, pol + 0x918) |
	       (u64)g5p_dc_root_word(b, pol + 0x91c) << 32;
	p920 = g5p_dc_root_word(b, pol + 0x920) |
	       (u64)g5p_dc_root_word(b, pol + 0x924) << 32;
	m784 = g5p_dc_root_word(b, meta + 0x784) |
	       (u64)g5p_dc_root_word(b, meta + 0x788) << 32;
	if (!g5p_mmio_hold(g, &t, G5P_MMIO_CENSUS))
		g5p_mmio_release(g, &t);
	dev_emerg(
		g->dev,
		"GPUPERF readback why=%s seed=%u/%u poweron=%u pol0=%u pol4=%u pol8=%u polc=%u "
		"pol28=%u pol2c=%u pol30=%u pol34=%u p8f8=%llx fw_now918=%llx p920=%llx "
		"meta784=%llx host_now=%llx stamp=%llx fw_counter_minus_stamp_us=%lld "
		"host_minus_fw_counter_us=%lld r1=%s r1_1000=%08x r1_0=%08x end=gpuperf\n",
		why, b->perf_seed_mode, b->perf_seed, b->perf_poweron,
		g5p_dc_root_word(b, pol + 0), g5p_dc_root_word(b, pol + 4),
		g5p_dc_root_word(b, pol + 8), g5p_dc_root_word(b, pol + 0xc),
		g5p_dc_root_word(b, pol + 0x28),
		g5p_dc_root_word(b, pol + 0x2c),
		g5p_dc_root_word(b, pol + 0x30),
		g5p_dc_root_word(b, pol + 0x34), p8f8, p918, p920, m784, host,
		b->seed_stamp,
		b->seed_stamp ?
			div_s64((s64)(p918 - m784 - b->seed_stamp) * 1000000LL,
				rate) :
			0,
		div_s64((s64)(host - (p918 - m784)) * 1000000LL, rate),
		g5p_mmio_did(&t), t.gpc, t.r1_0);
}

static int g5p_r73c_fast_control(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	int ret;

	r->fast_release_ns = ktime_get_mono_fast_ns();
	WRITE_ONCE(r->fast_window, true);
	ret = g5p_boot_events(g);
	r->fast_alive_ns = ktime_get_mono_fast_ns();
	if (!ret)
		ret = g5p_control_prepare(g);
	if (!ret) {
		r->fast_state = readl(g->window[1] + 0x1000);
		ret = g5p_control_submit(g);
	}
	r->fast_submit_ns = ktime_get_mono_fast_ns();
	r->fast_ret = ret;
	WRITE_ONCE(r->fast_window, false);
	g5p_r73b_events_exit(g, ret);
	dev_info(
		g->dev,
		"R73C fast release_to_alive_ns=%llu release_to_submit_ns=%llu state=%08x ret=%d "
		"control_submitted=%u end=r73c\n",
		r->fast_alive_ns - r->fast_release_ns,
		r->fast_submit_ns - r->fast_release_ns, r->fast_state, ret,
		READ_ONCE(b->control_submitted));
	g5p_perf_readback(g, ret ? "alive-timeout" : "alive");
	return ret;
}

/* perf60 pin 200 (docs/gpu/experiments/perf60-clock.md "Pin 200: the guarded drain read" and
 * "Clock-change drain RE" §4): at a boot control timeout with boot_gpu_perf set (before any job,
 * the firmware alive), every read goes through the GPU register licence (mmio.inc) and nothing
 * wakes the GPU (no R1+0x14 write, mmio-access.md G6):
 *   r1:    R1 +0, +8, +0x14, +0x1000, +0x1004, +0x20000 (one R1-only ticket, any pstate)
 *   drain: 11f00 688 1720 1728 e230 e238 e138, then the clock words R0+0x0/0x4/0x8/0xc
 *   timer: R0+0x160/0x164, twice, 1 ms apart
 *   tx:    T0/T1 TXENABLE/TXSTATUS (0x04800000/10, 0x04801000/10) through the META slave port
 *   pc:    (bit 1 only) T0 PC: RQ 0x0480fff8 = 0x10005, poll bit 31, read 0x0480fff0; x8, 100 us
 *          apart.  This is the driver's first slave-port write (CTRL0 = addr, DATAT = value).
 * One ticket per group or slave-port op (<= G5P_MMIO_MAX_OPS), a refusal retried <= 2 ms.
 */

#define G5P_GPP_BUDGET_US 2000U

/* One META slave-port read in one ticket: ready, CTRL0 = addr | 1, read back, ready (<= 4
 * polls), DATAX: 8 ops.  -EBUSY when the port is not ready within the ticket.
 */
static int g5p_mmio_sp_read(struct g5p_diag *g, u32 addr, u32 *out, u32 *tries)
{
	struct g5p_mmio_ticket t;
	u32 c1 = 0;
	int ret;

	if (!g5p_slave_access_allowed(READ_ONCE(g->reset_transition)))
		return -EPERM;
	ret = g5p_mmio_hold_wait(g, &t, 0, G5P_GPP_BUDGET_US, tries);
	if (ret)
		return ret;
	ret = -EBUSY;
	c1 = g5p_mmio_r0_read(g, &t, G5P_MSLVCTRL1);
	if ((c1 & G5P_SP_READY) == G5P_SP_READY &&
	    g5p_mmio_r0_write(g, &t, G5P_MSLVCTRL0, addr | 1)) {
		g5p_mmio_r0_read(g, &t, G5P_MSLVCTRL0);
		for (u32 i = 0; i < 4; i++) {
			c1 = g5p_mmio_r0_read(g, &t, G5P_MSLVCTRL1);
			if ((c1 & G5P_SP_READY) == G5P_SP_READY) {
				*out = g5p_mmio_r0_read(g, &t, G5P_MSLVDATAX);
				ret = 0;
				break;
			}
		}
	}
	if (!g5p_mmio_release(g, &t) && !ret)
		ret = -ESTALE;
	return ret;
}

/* One META slave-port write in one ticket: ready, CTRL0 = addr, DATAT = value, ready (<= 4). */
static int g5p_mmio_sp_write(struct g5p_diag *g, u32 addr, u32 value,
			     u32 *tries)
{
	struct g5p_mmio_ticket t;
	u32 c1;
	int ret;

	if (!g5p_slave_access_allowed(READ_ONCE(g->reset_transition)))
		return -EPERM;
	ret = g5p_mmio_hold_wait(g, &t, 0, G5P_GPP_BUDGET_US, tries);
	if (ret)
		return ret;
	ret = -EBUSY;
	c1 = g5p_mmio_r0_read(g, &t, G5P_MSLVCTRL1);
	if ((c1 & G5P_SP_READY) == G5P_SP_READY &&
	    g5p_mmio_r0_write(g, &t, G5P_MSLVCTRL0, addr) &&
	    g5p_mmio_r0_write(g, &t, G5P_MSLVDATAT, value)) {
		for (u32 i = 0; i < 4; i++) {
			c1 = g5p_mmio_r0_read(g, &t, G5P_MSLVCTRL1);
			if ((c1 & G5P_SP_READY) == G5P_SP_READY) {
				ret = 0;
				break;
			}
		}
	}
	g5p_mmio_release(g, &t);
	return ret;
}

/* A group of plain R0 reads (<= G5P_MMIO_MAX_OPS) under one ticket; returns the ticket's did. */
static const char *g5p_gpp_r0(struct g5p_diag *g, const u32 *off, u32 *v, u32 n,
			      u32 *tries, u32 *gpc)
{
	struct g5p_mmio_ticket t;

	if (g5p_mmio_hold_wait(g, &t, 0, G5P_GPP_BUDGET_US, tries)) {
		*gpc = t.gpc;
		return g5p_mmio_did(&t);
	}
	for (u32 i = 0; i < n; i++)
		v[i] = g5p_mmio_r0_read(g, &t, off[i]);
	g5p_mmio_release(g, &t);
	*gpc = t.gpc;
	g5p_mmio_sample_line(g, G5P_MMS_PROBE, &t, v, n, true);
	return g5p_mmio_did(&t);
}

/* What a slave-port op's return means, for the dc-probe lines. */
static const char *g5p_sp_why(int ret)
{
	switch (ret) {
	case 0:
		return "ok";
	case -ENODATA:
		return "not-run";
	case -EPERM:
		return "forbidden-after-release"; /* g->reset_transition: boot.inc's post-release ban */
	case -ENODEV:
		return "licence-off";
	case -EAGAIN:
		return "licence-refused";
	case -EBUSY:
		return "port-not-ready";
	case -ESTALE:
		return "stale";
	case -ETIMEDOUT:
		return "rq-no-ready";
	default:
		return "error";
	}
}

static void g5p_gpu_perf_probe(struct g5p_diag *g)
{
	static const u32 drain[] = { 0x11f00, 0x688,  0x1720, 0x1728,
				     0xe230,  0xe238, 0xe138 };
	static const u32 clk[] = { 0x0, 0x4, 0x8, 0xc };
	static const u32 timer[] = { 0x160, 0x164 };
	static const u32 r1off[] = { 0x0, 0x8, 0x14, 0x1000, 0x1004, 0x20000 };
	static const u32 tx[] = { 0x04800000, 0x04800010, 0x04801000,
				  0x04801010 };
	u32 mode = (0);
	struct g5p_boot *b = g->boot;
	u32 vd[ARRAY_SIZE(drain)], vc[ARRAY_SIZE(clk)], vt[2][2],
		r1[ARRAY_SIZE(r1off)];
	u32 vx[ARRAY_SIZE(tx)], pc[8], pcrq[8], gate = U32_MAX, tries = 0,
						gpc = U32_MAX, gpc_x;
	int txr[ARRAY_SIZE(tx)], pcr[8];
	const char *d_r1 = "skip-guard", *d_drain = "skip-guard",
		   *d_clk = "skip-guard";
	const char *d_t0 = "skip-guard", *d_t1 = "skip-guard";
	struct g5p_mmio_ticket t;

	if (!mode || !b->gpu_perf)
		return;
	memset(vd, 0xff, sizeof(vd));
	memset(vc, 0xff, sizeof(vc));
	memset(vt, 0xff, sizeof(vt));
	memset(r1, 0xff, sizeof(r1));
	memset(vx, 0xff, sizeof(vx));
	memset(pc, 0xff, sizeof(pc));
	memset(pcrq, 0xff, sizeof(pcrq));
	for (u32 i = 0; i < ARRAY_SIZE(txr); i++)
		txr[i] = -ENODATA;
	for (u32 i = 0; i < ARRAY_SIZE(pcr); i++)
		pcr[i] = -ENODATA;
	if (g->pmgr && regmap_read(g->pmgr, 0x80340, &gate))
		gate = U32_MAX;
	if (READ_ONCE(b->failed))
		goto print;
	/* r1: before anything else, R1-only (valid at any pstate and with bit 29 set) */
	if (!g5p_mmio_hold(g, &t, G5P_MMIO_CENSUS)) {
		for (u32 i = 0; i < ARRAY_SIZE(r1off); i++)
			r1[i] = g5p_mmio_r1_read(g, &t, r1off[i]);
		g5p_mmio_release(g, &t);
	}
	d_r1 = g5p_mmio_did(&t);
	/* gpc: the drain group's licence read; every group's ticket also has its own sample line */
	d_drain = g5p_gpp_r0(g, drain, vd, ARRAY_SIZE(drain), &tries, &gpc);
	d_clk = g5p_gpp_r0(g, clk, vc, ARRAY_SIZE(clk), &tries, &gpc_x);
	d_t0 = g5p_gpp_r0(g, timer, vt[0], 2, &tries, &gpc_x);
	usleep_range(1000, 1100);
	d_t1 = g5p_gpp_r0(g, timer, vt[1], 2, &tries, &gpc_x);
	for (u32 i = 0; i < ARRAY_SIZE(tx); i++)
		txr[i] = g5p_mmio_sp_read(g, tx[i], &vx[i], &tries);
	if (mode & 2) {
		for (u32 i = 0; i < ARRAY_SIZE(pc); i++) {
			pcr[i] = g5p_mmio_sp_write(g, 0x0480fff8, 0x10005,
						   &tries);
			for (u32 k = 0; !pcr[i] && k < 4; k++) {
				pcr[i] = g5p_mmio_sp_read(g, 0x0480fff8,
							  &pcrq[i], &tries);
				if (pcr[i] || (pcrq[i] & BIT(31)))
					break;
				udelay(10);
			}
			if (!pcr[i])
				pcr[i] = pcrq[i] & BIT(31) ?
						 g5p_mmio_sp_read(g, 0x0480fff0,
								  &pc[i],
								  &tries) :
						 -ETIMEDOUT;
			udelay(100);
		}
	}
print:
	dev_err(g->dev,
		"GPUPERF dc-probe did=%s gate=%08x r1=%s r1_0=%08x r1_8=%08x r1_14=%08x "
		"r1_1000=%08x r1_1004=%08x r1_20000=%08x gpc=%08x tries=%u end=gpuperf\n",
		d_drain, gate, d_r1, r1[0], r1[1], r1[2], r1[3], r1[4], r1[5],
		gpc, tries);
	dev_err(g->dev,
		"GPUPERF dc-probe drain=%s 11f00=%08x 688=%08x 1720=%08x 1728=%08x e230=%08x "
		"e238=%08x e138=%08x clk=%s clk=%08x:%08x clkstat=%08x:%08x end=gpuperf\n",
		d_drain, vd[0], vd[1], vd[2], vd[3], vd[4], vd[5], vd[6], d_clk,
		vc[1], vc[0], vc[3], vc[2]);
	dev_err(g->dev,
		"GPUPERF dc-probe timer=%s/%s 160=%08x/%08x 164=%08x/%08x gap_us=1000 "
		"txen0=%08x/%s txst0=%08x/%s txen1=%08x/%s txst1=%08x/%s end=gpuperf\n",
		d_t0, d_t1, vt[0][0], vt[1][0], vt[0][1], vt[1][1], vx[0],
		g5p_sp_why(txr[0]), vx[1], g5p_sp_why(txr[1]), vx[2],
		g5p_sp_why(txr[2]), vx[3], g5p_sp_why(txr[3]));
	if (mode & 2)
		for (u32 i = 0; i < ARRAY_SIZE(pc); i++)
			dev_err(g->dev,
				"GPUPERF dc-probe pc i=%u ret=%s rq=%08x t0pc=%08x end=gpuperf\n",
				i, g5p_sp_why(pcr[i]), pcrq[i], pc[i]);
}

static int g5p_control_wait(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	unsigned long start = jiffies, deadline = start + 3 * HZ;
	bool sampled = false, rekicked = false;
	u32 consumer, producer, pass = 0;
	int ret;

	if (!b->control_submitted || b->control_complete)
		return -EPERM;
	do {
		ret = g5p_r73b_service(g, G5P_R73B_OWNER_WAIT,
				       g5p_r73b_sampled(++pass));
		if (ret)
			return ret;
		consumer = READ_ONCE(ctl[0]);
		producer = READ_ONCE(ctl[4]);
		dma_rmb();
		if (consumer > 255 || producer != b->control_target ||
		    (consumer != b->control_target &&
		     consumer != ((b->control_target - 1) & 255)))
			return -EPROTO;
		if (consumer == b->control_target) {
			if (READ_ONCE(b->failed))
				return -EIO;
			if (g5p_r74_case(b) == 1) {
				WRITE_ONCE(b->control_complete, true);
				g5p_r74_mark(g, G5P_R74_CMD8_CONSUMED,
					     consumer);
				dev_emerg(
					g->dev,
					"R74B cmd8-consumed consumer=%u producer=%u IRQ174=%d ram_only=1 end=r74b\n",
					consumer, producer,
					atomic_read(&b->irqs));
				g5p_np_policy(g, "cmd8-consumed", consumer, 0);
				return 0;
			}
			g5p_dc_snapshot(g, "consumed", false);
			b->control_complete = true;
			;
			dev_info(
				g->dev,
				"DC consumed cmd=8 consumer=%u producer=%u IRQ174=%d; transport only\n",
				consumer, producer, atomic_read(&b->irqs));
			return 0;
		}
		if (!sampled && !time_before(jiffies, start + HZ / 10)) {
			sampled = true;
			if (!g5p_r74b_ram_only(g))
				g5p_dc_snapshot(g, "100ms", false);
			g5p_np_policy(g, "100ms", consumer, 0);
		}
		if (7 == 3 && !rekicked && !time_before(jiffies, start + HZ)) {
			/* Re-notify the SAME slot once; never publish a second command. */
			if (READ_ONCE(b->failed))
				return -EIO;
			ret = g5p_prepare_kick(g, "DC re-kick");
			if (ret)
				return ret;
			mb();
			writel(0x10, g->rgx + 0xb00);
			rekicked = true;
			if (!g5p_r74b_ram_only(g))
				g5p_dc_snapshot(g, "re-kick", false);
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	if (!g5p_r74b_ram_only(g))
		g5p_dc_snapshot(g, "timeout", false);
	g5p_np_policy(g, "timeout", consumer, 0);
	dev_err(g->dev,
		"DC timeout consumer=%u producer=%u target=%u; retain RAM, no retry\n",
		consumer, producer, b->control_target);
	/* perf60 (boot_gpu_perf): the policy-state words at the stall, RAM only.  pol34 = the pstate the
	 * firmware last requested (its setter stores it before the Fender write): q means it reached the
	 * setter, 1 that it stopped before (the clock-change drain 0x1001cec8 -> 0x1001b638).
	 */
	if (b->gpu_perf) {
		u32 pol = b->layout.offset[0x4a];

		dma_rmb();
		dev_err(g->dev,
			"GPUPERF dc-timeout pin=%u pol8=%u/%u q=%u x=%u pol34=%u req=%u/%u ceil=%u/%u "
			"seed=%u pol0=%u pol4=%u pol28=%u pol2c=%u pol30=%u end=gpuperf\n",
			b->gpu_perf, g5p_dc_root_word(b, pol + 8),
			g5p_dc_root_word(b, pol + 0xc),
			g5p_dc_root_word(b, pol + 0x28),
			g5p_dc_root_word(b, pol + 0x38),
			g5p_dc_root_word(b, pol + 0x34),
			g5p_dc_root_word(b, pol + 0x6f0),
			g5p_dc_root_word(b, pol + 0x6f4),
			g5p_dc_root_word(b, pol + 0x688),
			g5p_dc_root_word(b, pol + 0x68c), b->perf_seed,
			g5p_dc_root_word(b, pol + 0),
			g5p_dc_root_word(b, pol + 4),
			g5p_dc_root_word(b, pol + 0x28),
			g5p_dc_root_word(b, pol + 0x2c),
			g5p_dc_root_word(b, pol + 0x30));
		g5p_gpu_perf_probe(g);
		g5p_perf_readback(g, "dc-timeout");
	}
	if (READ_ONCE(b->failed))
		return -EIO;
	b->control_timed_out = true;
	return -ETIMEDOUT;
}

static int g5p_irq_test(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	unsigned long deadline = jiffies + HZ / 10;
	int before = atomic_read(&b->irqs), ret;

	if (b->irq_tested || !b->irq_ready || g5p_r74_case(b))
		return -EPERM;
	/* ADT set/ack addresses, not a GPU slave-port transaction. Firmware may
	 * also interrupt concurrently: this tests delivery, not event provenance.
	 */
	dev_info(g->dev, "IRQ test baseline=%d W AIC+014=00004000\n", before);
	writel(0x4000, b->swirq + 0x14);
	do {
		if (READ_ONCE(b->failed))
			return -EIO;
		if (atomic_read(&b->irqs) > before) {
			ret = g5p_boot_drain(g);
			if (ret)
				return ret;
			b->irq_tested = true;
			;
			dev_info(
				g->dev,
				"IRQ delivery observed before=%d after=%d; no shader completion implied\n",
				before, atomic_read(&b->irqs));
			return 0;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	dev_err(g->dev, "IRQ delivery timeout before=%d after=%d\n", before,
		atomic_read(&b->irqs));
	return -ETIMEDOUT;
}

static int g5p_boot_park(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	const struct g5p_fender_io io = {
		.ctx = g,
		.write = g5p_fender_write,
		.poll = g5p_fender_poll,
	};
	static const u32 slots[] = { 0x60, 0x64, 0x68, 0x6c, 0x70 };
	bool failed_control = READ_ONCE(b->failed) && b->control_timed_out;
	bool failed_work = READ_ONCE(b->failed) && b->work_timed_out;
	bool failed_context = READ_ONCE(b->failed) && b->context_timed_out;
	bool failed_timeout = failed_control || failed_work || failed_context;
	int ret;

	if (b->r73b)
		g5p_r73b_disarm(b->r73b);
	if (READ_ONCE(b->event_failed) ||
	    (b->control_submitted && !b->control_complete && !failed_control) ||
	    (b->work_submitted && !b->work_complete && !failed_work) ||
	    (b->context_submitted && !b->context_complete && !failed_context))
		return -EBUSY;
	/* A failed drain must never be bypassed. On the sole permitted failure
	 * path require the event ring empty below without consuming anything.
	 */
	if (!failed_timeout) {
		ret = g5p_boot_drain(g);
		if (ret)
			return ret;
	}
	for (u32 i = 0; i < ARRAY_SIZE(slots); i++) {
		u32 *ctl = (u32 *)g5p_boot_shared(b, slots[i]);
		u32 consumer = READ_ONCE(ctl[0]), producer = READ_ONCE(ctl[4]);

		if (consumer > 255 || producer > 255)
			return -EPROTO;
		if (((failed_control || failed_context) && slots[i] == 0x6c) ||
		    (failed_work && slots[i] == 0x68)) {
			u32 target = failed_context && slots[i] == 0x6c ?
					     b->context_target :
				     slots[i] == 0x6c ? b->control_target :
							b->work_target;

			if (producer != target ||
			    (consumer != producer &&
			     consumer != ((producer - 1) & 255)))
				return -EPROTO;
		} else if (consumer != producer) {
			return -EBUSY;
		}
	}
	if (READ_ONCE(b->event_failed))
		return -EIO;
	if (failed_timeout)
		dev_info(
			g->dev,
			"PARK failed %s timeout; abandon command, preserve failure and RAM\n",
			failed_context ? "CONTEXT" :
			failed_work    ? "CL" :
					 "DC");
	/* Freeze commands before the first transition; no retry even if it fails.
	 * c9fa10 enabled-Fender branch: core=0, GPC=0, no a18 access.
	 */
	WRITE_ONCE(b->stopped, true);
	WRITE_ONCE(b->alive, false);
	dev_info(
		g->dev,
		"PARK begin core=0 GPC=0; wrapper only, DMA memory retained\n");
	ret = g5p_fender_idle(&io);
	if (ret)
		return ret;
	/* Keep the IRQ/ack handler and every allocation alive until reboot.
	 * GPC=0 alone is not proof that outstanding bus traffic has drained.
	 */
	b->parked = true;
	dev_info(
		g->dev,
		"PARK GPC=0 verified; quarantine until reboot, no PMGR off or DMA free\n");
	return 0;
}

static int g5p_lifecycle_command(struct g5p_diag *g, const char *command)
{
	struct g5p_boot *b = g->boot;
	int ret;

	if (!boot_experiment || !b || !b->alive || b->stopped)
		return -EPERM;
	if (b->failed &&
	    !(sysfs_streq(command, "park") && teardown_experiment &&
	      (b->control_timed_out || b->work_timed_out ||
	       b->context_timed_out)))
		return -EPERM;
	if (!strncmp(command, "control-", 8) && control_experiment) {
		if (sysfs_streq(command, "control-prepare"))
			ret = g5p_control_prepare(g);
		else if (sysfs_streq(command, "control-submit"))
			ret = g5p_control_submit(g);
		else if (sysfs_streq(command, "control-wait"))
			ret = g5p_control_wait(g);
		else
			return -EINVAL;
	} else if (sysfs_streq(command, "irq-test") &&
		   (g5p_r74_case(b) || (1))) {
		/* R74 never runs the AIC self-test r73f case 01 stopped in. */
		if (b->r73b)
			atomic_inc(&b->r73b->r74_irq_refused);
		g5p_r74_mark(g, G5P_R74_IRQ_TEST_REFUSED, -EPERM);
		dev_err(g->dev,
			"R74 refuse irq-test case=%u irq_tested=%u end=r74\n",
			g5p_r74_case(b), b->irq_tested);
		return -EPERM;
	} else if (sysfs_streq(command, "irq-test") && irq_experiment) {
		ret = g5p_irq_test(g);
	} else if (sysfs_streq(command, "park") && teardown_experiment) {
		ret = g5p_boot_park(g);
	} else {
		return -EPERM;
	}
	if (sysfs_streq(command, "control-wait"))
		g5p_r73b_crumb(b, G5P_R73B_WAIT_EXIT, G5P_R73B_OWNER_WAIT, ret);
	if (ret && ret != -EALREADY && ret != -EPERM)
		WRITE_ONCE(b->failed, true);
	return ret;
}

/* Outgoing-side kexec/reboot quiesce; see docs/gpu/experiments/gpu-reset-recovery-production.md
 * section 6.10. NOT a reset: Fender core=0/GPC=0 only, no PMGR off, no DMA free and
 * no R0 access. It acts only when the command lock is free (no job or sysfs command is
 * inside a hardware wait), the firmware was released and the pstate policy is
 * host-owned (init meta+0x6c8 == 0); every other state is left alone. Unlike
 * g5p_boot_park() it does not require drained rings: round 10 flew park after an
 * unconsumed DC command, and at shutdown there is nothing left to drain for.
 * GPC=0 is not a proof that bus traffic drained (see g5p_boot_park).
 */

static void g5p_td_shutdown(struct g5p_diag *g);

static void g5p_shutdown(struct platform_device *pdev)
{
	struct g5p_diag *g = platform_get_drvdata(pdev);
	const struct g5p_fender_io io = {
		.ctx = g,
		.write = g5p_fender_write,
		.poll = g5p_fender_poll,
	};
	struct g5p_boot *b;
	int ret;

	if (!g)
		return;
	/* Idempotent second chance after the reboot notifier; a no-op unless teardown != 0. */
	g5p_td_shutdown(g);
	{
		return;
	};
	/* A worker or sysfs writer stuck in MMIO holds g->lock: never queue behind it. */
	if (!mutex_trylock(&g->lock)) {
		dev_warn(g->dev, "SHUTDOWN park skipped: command lock busy\n");
		return;
	}
	b = g->boot;
	/* R1 is touched only with ps_gfx held on (the retained runtime-PM reference),
	 * the wrapper woken and the firmware released.
	 */
	if (!b || !g->powered || !g->fender_ready || !b->maps_ready ||
	    !b->released || b->stopped || b->parked) {
		dev_info(g->dev, "SHUTDOWN park not needed\n");
		goto out;
	}
	/* With firmware-owned policy (init meta+0x6c8 != 0) the firmware also writes
	 * R1+0x1000, so a host write would race it (dsx-r74b). RAM-only read of the
	 * host-written init word, the same one g5p_dc_snapshot() reports.
	 */
	if (g5p_dc_root_word(b, b->layout.offset[0x4d] + 0x6c8)) {
		dev_info(
			g->dev,
			"SHUTDOWN park refused: firmware-owned policy (meta+0x6c8 != 0)\n");
		goto out;
	}
	if (b->r73b)
		g5p_r73b_disarm(b->r73b);
	/* Freeze commands before the first transition; no retry even if it fails. */
	WRITE_ONCE(b->stopped, true);
	WRITE_ONCE(b->alive, false);
	ret = g5p_fender_idle(&io);
	if (!ret)
		WRITE_ONCE(b->parked, true);
	dev_info(
		g->dev,
		"SHUTDOWN park core=0 GPC=0 result=%d; DMA memory retained, no PMGR off\n",
		ret);
out:
	mutex_unlock(&g->lock);
}

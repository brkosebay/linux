/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* R73b crash-first IRQ174 diagnostics. Included by boot.inc after struct
 * g5p_boot. Case zero allocates nothing and keeps the r73 IRQ order.
 * Rows live in CPU RAM only; they are formatted after the epoch, never in the
 * IRQ service loop. The first 128 rows are retained, the last 128 roll.
 */
#include <linux/hrtimer.h>
#include <linux/log2.h>
#include <linux/smp.h>
#include <linux/utsname.h>
#include "r87b-attach.h"
#include "r74e112.h"
#include "heartbeat116.h"

#define G5P_R73B_ROWS 256
#define G5P_R73B_HEAD 128
#define G5P_R73B_IRQ_CAP 8192
#define G5P_R73B_PANIC_ROWS 32
#define G5P_R73B_TRACE_ROWS 128
#define G5P_R73B_TRACE_WORDS 9
#define G5P_R74_IRQ_CAP 262144
#define G5P_R74_IRQ_CAP_MAX 1048576
#define G5P_R74_EVENT_FLOOR (256 + 4 * 65536)
#define G5P_R74_STAGES 8
#define G5P_R74_MILESTONES 64
#define G5P_R74_MAX_TICKS 16
#define G5P_R74_RGX 9
#define G5P_R74_WORD1 8
#define G5P_R74F_KINDS 3
#define G5P_R74F_OFF_US 1000
#define G5P_R74F_ON_US 1800
#define G5P_R74F_BELL_MS 250
#define G5P_R74F_MMU_SLOTS 2
#define G5P_R74F_MMU_US 1000
#define G5P_R74G_GRACE_ALIVES 4
#define G5P_R74G_GRACE_US 20000
#define G5P_R74G_COMPANION 0x3980
#define G5P_R74G_RECOVERIES 0x3994
#define G5P_R74G_CLASS 0x3998
#define G5P_R74G_BAD_RING BIT(0)
#define G5P_R74G_BAD_RECORD BIT(1)
#define G5P_R74G_BAD_PA BIT(2)
#define G5P_R74G_BAD_HOST BIT(3)
#define G5P_R74G_BAD_CACHED BIT(4)
#define G5P_R74G_BAD_FAILED BIT(5)
#define G5P_R74G_BAD_COMPANION BIT(6)
#define G5P_R74G_BAD_MUTATION BIT(7)
#define G5P_R74H_TAIL 128
#define G5P_R74H_PHYS_MAX 1024
#define G5P_R74H_PTES 10

/* First-ALIVE submit: wait (reads only) for a GPC request in flight to finish. */
static unsigned int boot_alive_settle_us = 2000;
module_param(boot_alive_settle_us, uint, 0600);
MODULE_PARM_DESC(
	boot_alive_settle_us,
	"default0 (single read, as before); 1..5000: when the first-ALIVE submit reads "
	"R1+0x1000 with bit 4 set (a GPC performance-state request in flight), re-read "
	"it every 10 us for up to this long before the unchanged state==1 check, as "
	"Apple's setGPCPerformanceState waits for bit 4; R1 reads only; latched at map");

enum g5p_r73b_phase {
	G5P_R73B_RELEASE_ENTRY = 1,
	G5P_R73B_RELEASE_FIFO,
	G5P_R73B_RELEASE_GPC,
	G5P_R73B_RELEASE_EXIT,
	G5P_R73B_IRQ,
	G5P_R73B_DRAIN_BEFORE,
	G5P_R73B_DRAIN_AFTER,
	G5P_R73B_DRAIN_BUSY,
	G5P_R73B_CAP,
	G5P_R73B_EVENTS_EXIT,
	G5P_R73B_DC_PUBLISHED,
	G5P_R73B_DC_KICKED,
	G5P_R73B_WAIT_EXIT,
	G5P_R73B_EXPORT,
	G5P_R73B_PHASES,
};

static const char *const g5p_r73b_phase_names[G5P_R73B_PHASES] = {
	"unknown",	"release-entry", "release-fifo", "release-gpc",
	"release-exit", "irq",		 "drain-before", "drain-after",
	"drain-busy",	"cap",		 "events-exit",	 "dc-published",
	"dc-kicked",	"wait-exit",	 "export",
};

enum g5p_r73b_owner {
	G5P_R73B_NONE,
	G5P_R73B_OWNER_IRQ,
	G5P_R73B_OWNER_EVENTS,
	G5P_R73B_OWNER_PREPARE,
	G5P_R73B_OWNER_PUBLISH,
	G5P_R73B_OWNER_KICK,
	G5P_R73B_OWNER_WAIT,
};

/* Row columns: event, log lane 0, statistics, trace, device control. */
static const u8 g5p_r73b_row_slots[5] = { 0x70, 0x71, 0x72, 0x73, 0x6c };

struct g5p_r73b_row {
	u32 seq; /* sequence + 1, published last; zero means empty/torn */
	u32 phase;
	u64 t_ns;
	u32 irq, ack, released, owner, cap_seen;
	s32 result;
	u32 cp[5][2];
};

enum g5p_r74_step {
	G5P_R74_IRQ_TEST_REFUSED,
	G5P_R74_PREPARE_BEGIN,
	G5P_R74_PREPARE_END,
	G5P_R74_ATTACH_BEGIN,
	G5P_R74_ATTACH_PUBLISHED,
	G5P_R74_ATTACH_DOORBELL_BEGIN,
	G5P_R74_ATTACH_DOORBELL_END,
	G5P_R74_ATTACH_CONSUMED,
	G5P_R74_ATTACH_END,
	G5P_R74_TA_BEGIN,
	G5P_R74_TA_PUBLISH,
	G5P_R74_TA_DOORBELL_RETURNED,
	G5P_R74_OBSERVE,
	G5P_R74_WAIT_TICK,
	G5P_R74_WAIT_EXIT,
	G5P_R74_REPORT_END,
	G5P_R74_CMD8_CONSUMED,
	G5P_R74_TA_CONSUMED,
	G5P_R74_TA_RAM_COMPLETE,
	G5P_R74_TA_RAM_TIMEOUT,
	G5P_R74F_ATTACH_ARMED,
	G5P_R74F_ATTACH_REACTIVATED,
	G5P_R74F_ATTACH_BELL,
	G5P_R74F_MMU_CLOSED,
	G5P_R74F_TA_ARMED,
	G5P_R74F_TA_REACTIVATED,
	G5P_R74F_TA_BELL,
	G5P_R74F_STOP,
	G5P_R74G_FW_MMU,
	G5P_R74G_ACK_SKIPPED,
	G5P_R74G_TA_GRACE,
	G5P_R74G_TA_FALLBACK,
	G5P_R74_STEPS,
};

static const char *const g5p_r74_steps[G5P_R74_STEPS] = {
	"irq-test-refused",
	"prepare-begin",
	"prepare-end",
	"attach-begin",
	"attach-dc3-published",
	"attach-doorbell-begin",
	"attach-doorbell-end",
	"attach-consumed",
	"attach-end",
	"ta-begin",
	"ta-publish",
	"ta-doorbell-returned",
	"observe",
	"wait-tick",
	"wait-exit",
	"report-end",
	"cmd8-consumed",
	"ta-consumed",
	"ta-ram-complete",
	"ta-ram-timeout",
	"attach-bell-armed",
	"attach-reactivated",
	"attach-bell",
	"first-mmu-window-closed",
	"ta-bell-armed",
	"ta-reactivated",
	"ta-bell",
	"r74f-stop",
	"firmware-mmu-complete",
	"r31-ack-skipped",
	"ta-grace-end",
	"ta-fallback",
};

/* R74 read-only witnesses; every address is already read elsewhere in-tree. */

struct g5p_r74_milestone {
	u64 t_ns;
	u32 step, irqs, events;
	s32 ret;
	u32 dc[2], ta[2];
};

enum g5p_r74f_kind {
	G5P_R74F_DC3,
	G5P_R74F_TA,
	G5P_R74F_3D, /* r96: ring 0x64, doorbell 0x11, owned by the ALIVE decoder like the TA's */
};

/* idle -> published (gate closed) -> open (reactivated) -> firing ->
 * rang; published/open -> cancelled. FIRING lives under event_lock.
 */
enum g5p_r74f_state {
	G5P_R74F_IDLE,
	G5P_R74F_PUBLISHED,
	G5P_R74F_OPEN,
	G5P_R74F_FIRING,
	G5P_R74F_RANG,
	G5P_R74F_CANCELLED,
	G5P_R74F_STATES,
};

enum g5p_r74f_cancel {
	G5P_R74F_CANCEL_NONE,
	G5P_R74F_CANCEL_REACTIVATION,
	G5P_R74F_CANCEL_NO_ALIVE,
	G5P_R74F_CANCEL_CONSUMED,
	G5P_R74F_CANCEL_PROTOCOL,
	G5P_R74F_CANCEL_ALIVE_ADVANCED,
	G5P_R74F_CANCEL_SELF_DRAIN,
	G5P_R74F_CANCELS,
};

enum g5p_r74f_stop {
	G5P_R74F_STOP_NONE,
	G5P_R74F_STOP_ATTACH_REACTIVATION,
	G5P_R74F_STOP_ATTACH_NOT_KICKED,
	G5P_R74F_STOP_ATTACH_NOT_CONSUMED,
	G5P_R74F_STOP_MMU_CLOSED,
	G5P_R74F_STOP_TA_REACTIVATION,
	G5P_R74F_STOP_TA_NOT_KICKED,
	G5P_R74G_STOP_DC3_TAGS,
	G5P_R74G_STOP_DC3_RAM,
	G5P_R74G_STOP_COMPANION,
	G5P_R74G_STOP_TA_NOT_CONSUMED,
	G5P_R74G_STOP_TA_WINDOW_MISSED,
	G5P_R74F_STOPS,
};

enum g5p_r74f_path {
	G5P_R74F_PATH_NONE,
	G5P_R74F_PATH_IMMEDIATE,
	G5P_R74F_PATH_DEFERRED,
	G5P_R74F_PATHS,
};

/* One generation. Its owner writes the receipt, then releases state. */
struct g5p_r74f_bell {
	atomic_t state;
	u32 kind, generation, ring, start, target, path, cancel;
	u32 arm_alive, open_alive, bell_alive;
	u32 first_fence, first_state, fence, bell_state;
	s32 first_ret, bell_ret, power_ret;
	u32 reactivated, core_fence, gpc_fence, off_raw, on_raw;
	u32 attempted, kicked, complete;
	u32 consumer_before, consumer_after, producer_after;
	u32 alive_done;
	u64 arm_ns, power_start_ns, off_ns, on_ns, open_ns;
	u64 bell_start_ns, bell_ns, done_ns;
};

enum g5p_r74g_grace {
	G5P_R74G_GRACE_NONE,
	G5P_R74G_GRACE_CONSUMED,
	G5P_R74G_GRACE_ALIVES_END,
	G5P_R74G_GRACE_WALL,
	G5P_R74G_GRACE_FAULT,
	G5P_R74G_GRACES,
};

/* The RAM-visible results of the firmware DC3 handler: phase 0 at the DC3
 * consume, phase 1 again just before the TA producer moves.
 */
struct g5p_r74g_dc3 {
	u64 pa[2], expected_pa, t_ns;
	u32 slot[2], host[2], cached[2], rec[2][5];
	u32 start, target, consumer, producer, tag;
	u32 failed, event_failed, companion, recoveries, classification, bad;
};

struct g5p_r73b {
	struct device *dev;
	struct g5p_boot *boot;
	u32 case_id, watchdog_ms, dc_variant, meta6c8, r73e_case;
	bool ack_first, aux, fast_enabled, fast_window;
	bool early_release_ready, early_power_abort;
	atomic_t early_once, early_busy;
	u64 early_irq_ns, early_power_start_ns;
	u64 early_off_ns, early_on_ns;
	u32 early_state, early_after_state, early_power_raw;
	s32 early_ret, early_power_ret;
	/* wake112 (r74e112.h): the R74E proof deferred to the first-ALIVE owner. */
	atomic_t early_proof;
	u32 early_proof_gen, early_proof_gen_now, early_proof_why;
	s32 early_defer_ret;
	/* boot_alive_settle_us: limit, first in-flight value seen, re-reads, time spent. */
	u32 settle_us, settle_first, settle_samples;
	u64 settle_ns;
	u64 fast_release_ns, fast_alive_ns, fast_submit_ns;
	u32 fast_state;
	s32 fast_ret;
	/* nopatch: Apple's unmodified image; policy record +0/+4/+8/+c/+34 taken before the first bell. */
	bool nopatch;
	u32 np_pre[5];
	atomic_t seq, acks, cap_seen, busy, drains, drain_errors, exported,
		trace_saved;
	atomic_t watchdog_state; /* 0 off, 1 armed, 2 disarmed, 3 fired, 4 terminal */
	int watchdog_cpu;
	u32 owner;
	struct mutex drain_lock; /* single auxiliary-ring owner */
	struct hrtimer watchdog;
	struct g5p_r73b_row rows[G5P_R73B_ROWS];
	u32 trace_rows[G5P_R73B_TRACE_ROWS][G5P_R73B_TRACE_WORDS];
	u32 irq_cap, r74_case, r74_mask, r74_ticks, r74_word1_count;
	u32 r74_word1[G5P_R74_WORD1];
	atomic_t r74_seq, r74_irq_refused;
	struct g5p_r74_milestone r74_milestones[G5P_R74_MILESTONES];

	/* r74f_alive and FIRING are written under event_lock only. */
	u32 r74f_case, r74f_alive, r74f_active, r74f_generation, r74f_stop;

	u32 r87b_attach;
	s32 r74f_stop_value;
	atomic_t r74f_bells;
	struct g5p_r74f_bell r74f_bell[G5P_R74F_KINDS];

	/* R74g: no host MMU pass, publish-only TA grace, one fallback bell. */
	u32 r74g_case, r74g_ack_skipped, r74g_grace_end, r74g_opened;
	u32 r74g_grace_alive;
	u64 r74g_grace_ns;
	struct g5p_r74g_dc3 r74g_dc3[2];
	/* R74h: RAM-only trace tail and checkpoints; NULL when off. */
	u32 r74h_case;

	/* R80: firmware trace observer; NULL when off. */

	/* Heartbeat116 (heartbeat116.h): the R74 window detector; IRQ thread only. */
	bool hb_on;
	u32 hb_seed, hb_empty, hb_cross;
	atomic_t hb_late;
	struct g5p_hb_cfg hb_cfg;
	struct g5p_hb hb;
};

/* The ctrl+0 word installed before release: the r80 stage's, else the r78 control's. */
static u32 g5p_boot_ctrl0(struct g5p_boot *b)
{
	return b->r73b && false ? 0 : b->r78_ctrl0;
}

/* boot-r80.inc, included by boot.inc after boot-r74h.inc. */

/* render-r96.inc: the receipts at export (RAM only) */

/* render-r94.inc: the RAM state of the record and the code BO at export, read only */

/* boot-r80.inc: the two IRQ-owned plateau banks, read-only */

/* render-r84b.inc: the late raw dump and the CPU visibility step, RAM only */

static int g5p_boot_drain(struct g5p_diag *g);
static u32 g5p_transport_event_limit(void);
static int g5p_r73c_fast_control(struct g5p_diag *g);
static int g5p_r73e_prepare(struct g5p_diag *g);
static int g5p_r73e_irq_submit(struct g5p_diag *g);
static int g5p_r73e_alive_submit(struct g5p_diag *g, u32 owner);

static const char *g5p_r73b_name(u32 id)
{
	return id == 1 ? "meta1-bounded-all-rings" :
	       id == 2 ? "meta0-bounded-all-rings-control" :
	       id == 3 ? "meta1-event-only-reference" :
			 "off";
}

static bool g5p_r73b_sampled(u32 n)
{
	return n <= 16 || is_power_of_2(n);
}

static u32 *g5p_r73b_ptr(struct g5p_boot *b, u32 off)
{
	return (u32 *)(b->shared[off / G5P_GPU_PAGE].cpu + off % G5P_GPU_PAGE);
}

static u32 g5p_r73b_word(struct g5p_boot *b, u32 off)
{
	return READ_ONCE(*g5p_r73b_ptr(b, off));
}

static u32 g5p_r73b_slot(u32 seq)
{
	if (seq < G5P_R73B_HEAD)
		return seq;
	return G5P_R73B_HEAD +
	       (seq - G5P_R73B_HEAD) % (G5P_R73B_ROWS - G5P_R73B_HEAD);
}

/* Lockless: a writer owns its slot by sequence; seq is published last. */
static void g5p_r73b_crumb(struct g5p_boot *b, u32 phase, u32 owner, int result)
{
	struct g5p_r73b *r = b->r73b;
	struct g5p_r73b_row *row;
	u32 seq;

	if (!r)
		return;
	seq = atomic_inc_return(&r->seq) - 1;
	row = &r->rows[g5p_r73b_slot(seq)];
	WRITE_ONCE(row->seq, 0);
	/* Invalidate before rewriting so a reader never pairs old seq/new data. */
	smp_wmb();
	row->phase = phase;
	row->t_ns = ktime_get_mono_fast_ns();
	row->irq = atomic_read(&b->irqs);
	row->ack = atomic_read(&r->acks);
	row->released = READ_ONCE(b->released);
	row->owner = owner;
	row->result = result;
	row->cap_seen = atomic_read(&r->cap_seen);
	for (u32 i = 0; i < ARRAY_SIZE(g5p_r73b_row_slots); i++) {
		u32 ctl = b->layout.offset[g5p_r73b_row_slots[i]];

		row->cp[i][0] = g5p_r73b_word(b, ctl);
		row->cp[i][1] = g5p_r73b_word(b, ctl + 0x10);
	}
	/* Publish the row contents before its sequence. */
	smp_wmb();
	WRITE_ONCE(row->seq, seq + 1);
}

static const char *g5p_r74_name(u32 id)
{
	return id == 1 ? "forced-ready-meta1-natural-ta" :
	       id == 2 ? "meta0-unpatched-natural-ta-control" :
			 "off";
}

static const char *g5p_r74_step_name(u32 step)
{
	return step < G5P_R74_STEPS ? g5p_r74_steps[step] : "unknown";
}

static u32 g5p_r74_case(struct g5p_boot *b)
{
	return b && b->r73b ? 1 : 0;
}

/* R74b case 1: after cmd8 retirement, optional host MMIO must not race the
 * firmware policy loop power-gating RGX.  Functional setup/kick sites opt in
 * explicitly; this predicate gates observations only.
 */
static bool g5p_r74b_ram_only(struct g5p_diag *g)
{
	struct g5p_boot *b = g ? g->boot : NULL;

	return g5p_r74_case(b) == 1 && b && READ_ONCE(b->released) &&
	       READ_ONCE(b->r72_meta6c8);
}

/* Process context only, never per IRQ: one console line per r74 step, so a
 * whole-SoC freeze is localized by the last line netconsole delivered.
 */
static void g5p_r74_mark(struct g5p_diag *g, u32 step, int ret)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b ? b->r73b : NULL;
	struct g5p_r74_milestone m;
	u32 seq;

	if (!r || !1)
		return;
	m.t_ns = ktime_get_ns();
	m.step = step;
	m.ret = ret;
	m.irqs = atomic_read(&b->irqs);
	m.events = READ_ONCE(b->events);
	m.dc[0] = g5p_r73b_word(b, b->layout.offset[0x6c]);
	m.dc[1] = g5p_r73b_word(b, b->layout.offset[0x6c] + 0x10);
	m.ta[0] = g5p_r73b_word(b, b->layout.offset[0x60]);
	m.ta[1] = g5p_r73b_word(b, b->layout.offset[0x60] + 0x10);
	seq = atomic_inc_return(&r->r74_seq) - 1;
	if (seq < G5P_R74_MILESTONES)
		r->r74_milestones[seq] = m;
	dev_emerg(
		g->dev,
		"R74 milestone seq=%u step=%s ret=%d irqs=%u events=%u dc=%u/%u ta=%u/%u "
		"t_ns=%llu end=r74\n",
		seq, g5p_r74_step_name(step), ret, m.irqs, m.events, m.dc[0],
		m.dc[1], m.ta[0], m.ta[1], m.t_ns);
}

/* R74 keeps repeated ALIVEs quiet whatever their word1: each new value is
 * printed once (eight at most), so a moved loop cannot flood the console.
 */
static bool g5p_r74_word1(struct g5p_boot *b, u32 word1)
{
	struct g5p_r73b *r = b->r73b;

	;
	;
	for (u32 i = 0; i < r->r74_word1_count; i++)
		if (r->r74_word1[i] == word1)
			return false;
	if (r->r74_word1_count == G5P_R74_WORD1)
		return false;
	r->r74_word1[r->r74_word1_count++] = word1;
	return true;
}

static bool g5p_r73b_copy_row(struct g5p_r73b *r, u32 seq,
			      struct g5p_r73b_row *out)
{
	struct g5p_r73b_row *row = &r->rows[g5p_r73b_slot(seq)];

	if (READ_ONCE(row->seq) != seq + 1)
		return false;
	/* Pairs with the writer's publication barrier. */
	smp_rmb();
	*out = *row;
	/* Re-check that the slot was not recycled while copying. */
	smp_rmb();
	return READ_ONCE(row->seq) == seq + 1;
}

static int g5p_r73b_format(char *buf, size_t len,
			   const struct g5p_r73b_row *row)
{
	u32 phase = row->phase < G5P_R73B_PHASES ? row->phase : 0;

	return scnprintf(
		buf, len,
		"R73B crumb seq=%u t_ns=%llu phase=%s irq=%u ack=%u released=%u event=%u/%u "
		"log0=%u/%u other=%u/%u trace=%u/%u dc=%u/%u owner=%u result=%d cap=%u end=r73b\n",
		row->seq - 1, row->t_ns, g5p_r73b_phase_names[phase], row->irq,
		row->ack, row->released, row->cp[0][0], row->cp[0][1],
		row->cp[1][0], row->cp[1][1], row->cp[2][0], row->cp[2][1],
		row->cp[3][0], row->cp[3][1], row->cp[4][0], row->cp[4][1],
		row->owner, row->result, row->cap_seen);
}

/* Snapshot the producer once and consume at most that modular distance
 * (<= 255). One consumer store returns the whole batch; a producer that
 * advances meanwhile is left for the next pass, never chased.
 */
static int g5p_r73b_drain_ring(struct g5p_boot *b, u32 ctl_slot, u32 lane,
			       u32 data_slot, u32 stride, u32 ring,
			       u32 *consumed)
{
	u32 ctl = b->layout.offset[ctl_slot] + lane * 0x20;
	u32 data = b->layout.offset[data_slot] +
		   (ctl_slot == 0x71 ? lane * 0x2c00 : 0);
	u32 consumer = g5p_r73b_word(b, ctl);
	u32 producer = g5p_r73b_word(b, ctl + 0x10);
	u32 count, hash = b->r73_aux_hash[ring] ?: 2166136261U;

	if (consumer > 255 || producer > 255)
		return -EPROTO;
	count = (producer - consumer) & 255;
	if (!count)
		return 0;
	/* Producer index before the slot payloads it covers. */
	dma_rmb();
	for (u32 n = 0; n < count; n++) {
		u32 words[G5P_R73B_TRACE_WORDS] = { 0 };
		u32 off = data + consumer * stride;

		for (u32 i = 0; i < stride / 4; i++) {
			u32 value = g5p_r73b_word(b, off + i * 4);

			if (i < ARRAY_SIZE(words))
				words[i] = value;
			hash = (hash ^ value) * 16777619U;
		}
		if (ring == 5 &&
		    atomic_read(&b->r73b->trace_saved) < G5P_R73B_TRACE_ROWS) {
			u32 saved = atomic_read(&b->r73b->trace_saved);

			memcpy(b->r73b->trace_rows[saved], words,
			       sizeof(words));
			/* Publish this complete row before increasing the visible count. */
			smp_wmb();
			atomic_inc(&b->r73b->trace_saved);
		};
		;
		if (!b->r73_aux_entries[ring])
			memcpy(b->r73_aux_first[ring], words,
			       sizeof(b->r73_aux_first[ring]));
		memcpy(b->r73_aux_last[ring], words,
		       sizeof(b->r73_aux_last[ring]));
		b->r73_aux_entries[ring]++;
		consumer = (consumer + 1) & 255;
	}
	b->r73_aux_hash[ring] = hash;
	/* dma_wmb() ordering plus the slot loads above: Apple returns the
	 * consumer only after its reads, with a full barrier.
	 */
	dma_mb();
	WRITE_ONCE(*g5p_r73b_ptr(b, ctl), consumer);
	*consumed += count;
	return 0;
}

/* Apple order after the ACK: four log lanes, events, statistics, trace. */
static int g5p_r73b_aux_pass(struct g5p_diag *g, u32 *consumed)
{
	struct g5p_boot *b = g->boot;
	int ret = 0;

	for (u32 lane = 0; lane < 4 && !ret; lane++)
		ret = g5p_r73b_drain_ring(b, 0x71, lane, 0x75, 44, lane,
					  consumed);
	if (!ret)
		ret = g5p_boot_drain(g);
	/* R80: ring 4 has one consumer, the drain thread (retains every record). */
	if (!ret && !false)
		ret = g5p_r73b_drain_ring(b, 0x72, 0, 0x76, 44, 4, consumed);
	if (!ret)
		ret = g5p_r73b_drain_ring(b, 0x73, 0, 0x77, 36, 5, consumed);
	return ret;
}

static bool g5p_r73b_pending(struct g5p_boot *b, bool aux)
{
	static const u8 slots[] = { 0x70, 0x71, 0x72, 0x73 };

	for (u32 i = 0; i < (aux ? ARRAY_SIZE(slots) : 1); i++) {
		for (u32 lane = 0; lane < (slots[i] == 0x71 ? 4 : 1); lane++) {
			u32 ctl = b->layout.offset[slots[i]] + lane * 0x20;

			if (g5p_r73b_word(b, ctl) !=
			    g5p_r73b_word(b, ctl + 0x10))
				return true;
		}
	}
	return false;
}

/* The IRQ thread only trylocks the auxiliary rings; a miss still runs the
 * spinlock-protected event decoder. Process-context paths lock normally.
 * Drains with work, errors or sampled entries record before/after rows.
 */
static int g5p_r73b_service(struct g5p_diag *g, u32 owner, bool sampled)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	u32 consumed = 0, before;
	bool rows, aux;
	int ret;

	if (!r)
		return g5p_boot_drain(g);
	if (READ_ONCE(b->failed))
		return -EIO;
	/* The atomic r73c window services only the event ring.  Auxiliary
	 * traffic is retained until cmd8 is either published or refused. */
	aux = 1 && !READ_ONCE(r->fast_window);
	if (aux && owner == G5P_R73B_OWNER_IRQ &&
	    !mutex_trylock(&r->drain_lock)) {
		atomic_inc(&r->busy);
		g5p_r73b_crumb(b, G5P_R73B_DRAIN_BUSY, READ_ONCE(r->owner),
			       -EBUSY);
		ret = g5p_boot_drain(g);
		goto out;
	}
	if (aux && owner != G5P_R73B_OWNER_IRQ)
		mutex_lock(&r->drain_lock);
	if (aux)
		WRITE_ONCE(r->owner, owner);
	atomic_inc(&r->drains);
	rows = sampled || g5p_r73b_pending(b, aux);
	if (rows)
		g5p_r73b_crumb(b, G5P_R73B_DRAIN_BEFORE, owner, 0);
	before = READ_ONCE(b->events);
	ret = aux ? g5p_r73b_aux_pass(g, &consumed) : g5p_boot_drain(g);
	consumed += READ_ONCE(b->events) - before;
	if (rows || ret)
		g5p_r73b_crumb(b, G5P_R73B_DRAIN_AFTER, owner, ret ?: consumed);
	if (aux) {
		WRITE_ONCE(r->owner, G5P_R73B_NONE);
		mutex_unlock(&r->drain_lock);
	}
out:
	if (ret) {
		atomic_inc(&r->drain_errors);
		WRITE_ONCE(b->event_failed, true);
		WRITE_ONCE(b->failed, true);
	}
	return ret;
}

/* Heartbeat116: an R74 epoch bounds IRQ174 and its ring entries per window. */
static bool g5p_hb116_on(struct g5p_boot *b)
{
	return b->r73b && b->r73b->hb_on;
}

/* Auxiliary-ring entries consumed so far (RAM, monotonic, may wrap). */
static u32 g5p_hb116_aux(struct g5p_boot *b)
{
	u32 sum = 0;

	for (u32 i = 0; i < ARRAY_SIZE(b->r73_aux_entries); i++)
		sum += READ_ONCE(b->r73_aux_entries[i]);
	return sum;
}

static u32 g5p_hb116_progress(struct g5p_boot *b)
{
	return READ_ONCE(b->events) + g5p_hb116_aux(b);
}

/* The lifetime cap's terminal stop, once: final row, cap_seen, failed, one disable. */
static void g5p_hb116_trip(struct g5p_diag *g, int irq, u32 n)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	const struct g5p_hb *h = &r->hb;
	u32 prior = READ_ONCE(b->failed) || READ_ONCE(b->event_failed);

	g5p_r73b_crumb(b, G5P_R73B_CAP, G5P_R73B_OWNER_IRQ, n);
	if (atomic_cmpxchg(&r->cap_seen, 0, 1))
		return;
	WRITE_ONCE(b->event_failed, true);
	WRITE_ONCE(b->failed, true);
	disable_irq_nosync(irq);
	dev_err(g->dev,
		"HB116 trip why=%s prior_failed=%u windows=%u stalled=%u ms=%llu irqs=%u "
		"events=%u acks=%u empty=%u aux=%u total_irqs=%u total_events=%u t_ns=%llu "
		"stop=masked late=ack-only end=hb116\n",
		g5p_hb_why_name[h->why < G5P_HBW_N ? h->why : 0], prior,
		h->windows, h->stalled, h->trip.ns / NSEC_PER_MSEC,
		h->trip.irqs, h->trip.events, h->trip.acks, h->trip.empty,
		h->trip.aux, n, READ_ONCE(b->events), h->trip_ns);
}

/* Receipts only: the first window past each Wake112 lifetime bound; nothing trips here. */
static void g5p_hb116_cross(struct g5p_diag *g, u32 irqs, u32 events)
{
	struct g5p_r73b *r = g->boot->r73b;
	u32 limit = g5p_transport_event_limit();

	if (!(r->hb_cross & 1) && irqs > r->irq_cap) {
		r->hb_cross |= 1;
		dev_info(
			g->dev,
			"HB116 cross bound=irq-cap value=%u irqs=%u events=%u windows=%u "
			"verdict=continue end=hb116\n",
			r->irq_cap, irqs, events, r->hb.windows);
	}
	if (!(r->hb_cross & 2) && events >= limit) {
		r->hb_cross |= 2;
		dev_info(
			g->dev,
			"HB116 cross bound=event-limit value=%u irqs=%u events=%u windows=%u "
			"verdict=continue end=hb116\n",
			limit, irqs, events, r->hb.windows);
	}
}

/* After the entry's ACK and service: account it, close a window, or trip. */
static void g5p_hb116_irq(struct g5p_diag *g, int irq, u32 n, u32 before)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	const struct g5p_hb_win *w = &r->hb.last;
	u32 seed = r->hb_seed, events = READ_ONCE(b->events);
	struct g5p_hb_in in;

	if (g5p_hb116_progress(b) == before)
		r->hb_empty++;
	in.now_ns = ktime_get_mono_fast_ns();
	in.irqs = n + seed;
	in.events = events + seed;
	in.acks = atomic_read(&r->acks) + seed;
	in.empty = r->hb_empty + seed;
	in.aux = g5p_hb116_aux(b) + seed;
	switch (g5p_hb_step(&r->hb, &r->hb_cfg, &in)) {
	case G5P_HBR_OPEN:
	case G5P_HBR_CLOSED:
		break;
	case G5P_HBR_TRIP:
		g5p_hb116_trip(g, irq, n);
		return;
	}
	g5p_hb116_cross(g, n, events);
	if (g5p_hb_report(&r->hb))
		dev_info(
			g->dev,
			"HB116 window n=%u t_ns=%llu ms=%llu irqs=%u events=%u acks=%u empty=%u aux=%u "
			"stalled=%u peak_irqs=%u peak_events=%u total_irqs=%u total_events=%u "
			"total_acks=%d end=hb116\n",
			r->hb.windows, in.now_ns, w->ns / NSEC_PER_MSEC,
			w->irqs, w->events, w->acks, w->empty, w->aux,
			r->hb.stalled, r->hb.peak_irqs, r->hb.peak_events, n,
			events, atomic_read(&r->acks));
}

/* Every entry ACKs. Over the cap: ACK, final row, cap_seen, one disable;
 * the kernel stays alive so the runner can export the record.
 * Heartbeat116: an R74 epoch has no lifetime cap.  The window detector runs
 * after the ACK and the service; after its trip an entry only ACKs.
 */
static irqreturn_t g5p_r73b_irq(struct g5p_diag *g, int irq, u32 n)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	bool sampled = g5p_r73b_sampled(n);
	u32 before = 0;

	if (r->hb_on && atomic_read(&r->cap_seen)) {
		writel(0x4000, b->swirq + 0x94);
		atomic_inc(&r->acks);
		atomic_inc(&r->hb_late);
		return IRQ_HANDLED;
	}
	if (!r->hb_on && n > r->irq_cap) {
		writel(0x4000, b->swirq + 0x94);
		atomic_inc(&r->acks);
		if (!atomic_read(&r->cap_seen)) {
			g5p_r73b_crumb(b, G5P_R73B_CAP, G5P_R73B_OWNER_IRQ, n);
			if (!atomic_cmpxchg(&r->cap_seen, 0, 1)) {
				WRITE_ONCE(b->event_failed, true);
				WRITE_ONCE(b->failed, true);
				disable_irq_nosync(irq);
			}
		}
		return IRQ_HANDLED;
	}
	{
		/* KPF level6G: Apple ACKs the AIC meta interrupt before ring drains. */
		writel(0x4000, b->swirq + 0x94);
		atomic_inc(&r->acks);
	}
	if (sampled)
		g5p_r73b_crumb(b, G5P_R73B_IRQ, G5P_R73B_OWNER_IRQ, n);
	if (r->hb_on)
		before = g5p_hb116_progress(b);
	if (READ_ONCE(b->released))
		g5p_r73b_service(g, G5P_R73B_OWNER_IRQ, sampled);
	if (1 && READ_ONCE(b->alive))
		g5p_r73e_alive_submit(g, G5P_R73B_OWNER_IRQ);
	;
	;
	;
	;
	;
	if (r->hb_on)
		g5p_hb116_irq(g, irq, n, before);
	return IRQ_HANDLED;
}

static void g5p_r73b_disarm(struct g5p_r73b *r)
{
	if (atomic_cmpxchg(&r->watchdog_state, 1, 2) == 1)
		hrtimer_cancel(&r->watchdog);
}

static void g5p_r73b_stop(void *data)
{
	g5p_r73b_disarm(data);
}

/* Cached CPU RAM only: no MMIO, no locks, then a deliberate panic so a
 * silent hang becomes a pstore/ramoops record.
 */
static enum hrtimer_restart g5p_r73b_watchdog_fire(struct hrtimer *timer)
{
	struct g5p_r73b *r = container_of(timer, struct g5p_r73b, watchdog);
	struct g5p_boot *b = r->boot;
	struct g5p_r73b_row row;
	u32 total = atomic_read(&r->seq), first, last;
	char line[320];
	bool found = false;

	if (atomic_cmpxchg(&r->watchdog_state, 1, 3) != 1)
		return HRTIMER_NORESTART;
	/* A contained cap or a latched failure is terminal: the IRQ only ACKs,
	 * service returns -EIO and the runner sees failed=1. Keep that host
	 * alive for the export instead of turning it into a deliberate panic.
	 */
	if (atomic_read(&r->cap_seen) || READ_ONCE(b->failed)) {
		atomic_set(&r->watchdog_state, 4);
		dev_emerg(
			r->dev,
			"R73B watchdog terminal case=%u irqs=%d acks=%d cap_seen=%d failed=%u rows=%u "
			"no-panic end=r73b\n",
			r->case_id, atomic_read(&b->irqs),
			atomic_read(&r->acks), atomic_read(&r->cap_seen),
			READ_ONCE(b->failed), total);
		return HRTIMER_NORESTART;
	}
	dev_emerg(
		r->dev,
		"R73B receipt case=%u name=%s ack_first=%u aux=%u cap=%u watchdog_ms=%u "
		"dc_variant=%u meta6c8=%u release=%s end=r73b\n",
		r->case_id, g5p_r73b_name(r->case_id), r->ack_first, r->aux,
		r->irq_cap, r->watchdog_ms, r->dc_variant, r->meta6c8,
		init_utsname()->release);
	first = total > G5P_R73B_PANIC_ROWS ? total - G5P_R73B_PANIC_ROWS : 0;
	for (u32 seq = first; seq < total; seq++) {
		if (!g5p_r73b_copy_row(r, seq, &row))
			continue;
		g5p_r73b_format(line, sizeof(line), &row);
		dev_emerg(r->dev, "%s", line);
	}
	for (last = total; last > first && !found; last--)
		found = g5p_r73b_copy_row(r, last - 1, &row);
	if (!found)
		memset(&row, 0, sizeof(row));
	panic("G5P R73B watchdog: release epoch incomplete after %ums case=%u irqs=%d acks=%d "
	      "cap_seen=%d busy=%d rows=%u last seq=%u phase=%s irq=%u ack=%u released=%u "
	      "event=%u/%u log0=%u/%u other=%u/%u trace=%u/%u dc=%u/%u owner=%u result=%d "
	      "cap=%u end=r73b\n",
	      0, 1, atomic_read(&b->irqs), atomic_read(&r->acks),
	      atomic_read(&r->cap_seen), atomic_read(&r->busy), total,
	      row.seq ? row.seq - 1 : 0,
	      g5p_r73b_phase_names[row.phase < G5P_R73B_PHASES ? row.phase : 0],
	      row.irq, row.ack, row.released, row.cp[0][0], row.cp[0][1],
	      row.cp[1][0], row.cp[1][1], row.cp[2][0], row.cp[2][1],
	      row.cp[3][0], row.cp[3][1], row.cp[4][0], row.cp[4][1], row.owner,
	      row.result, row.cap_seen);
	return HRTIMER_NORESTART;
}

static void g5p_r73b_release_entry(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;

	if (!r)
		return;
	g5p_r73b_crumb(b, G5P_R73B_RELEASE_ENTRY, G5P_R73B_NONE, 0);
	{
		return;
	}
}

static void g5p_r73b_release_exit(struct g5p_diag *g, int ret)
{
	struct g5p_boot *b = g->boot;

	if (!b->r73b)
		return;
	g5p_r73b_crumb(b, G5P_R73B_RELEASE_EXIT, G5P_R73B_NONE, ret);
	if (ret)
		g5p_r73b_disarm(b->r73b);
}

/* The release epoch ends when the host has returned from the ALIVE wait. */
static void g5p_r73b_events_exit(struct g5p_diag *g, int ret)
{
	struct g5p_boot *b = g->boot;

	if (!b->r73b)
		return;
	g5p_r73b_crumb(b, G5P_R73B_EVENTS_EXIT, G5P_R73B_OWNER_EVENTS, ret);
	g5p_r73b_disarm(b->r73b);
}

static int g5p_r73b_latch(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	const u32 id = (1), ms = (0);
	const u32 fast = (1);
	const u32 early = (1);
	const u32 r74 = (1), cap = (1048576);
	const u32 r74f = (1);
	const u32 r74g = (1);
	const u32 r74h = (0);

	struct g5p_r73b *r;
	int ret;

	;
	;
	;
	;
	;
	;
	/* nopatch replaces r73f case 1 under the whole r74/r74f/r74g path, and nothing else. */
	/* nopatch's receipts and its r74 gate are the TYPED_V3 path's; any other path (r74 0) runs
	 * Apple's image anyway (only r73f patches it) and gets neither.
	 */
	if (1 != g5p_fw_unpatched_want((1), 0, 0, 0) ||
	    (1 && r74 == 1 && (0 || r74f != 1 || r74g != 1))) {
		dev_err(g->dev,
			"NOPATCH reject phase=map unpatched=%u/%u r73f_case=%u path=%u/%u/%u end=nopatch\n",
			1, (1), 0, r74, r74f, r74g);
		return -EINVAL;
	}
	/* R74 rides only on an exact r73f case 1 (or nopatch) or 4 with the raised limits. */
	if (r74 && (!id || r74 > 2 || 0 != (r74 == 1 ? (1 ? 0 : 1) : 4) ||
		    1 != (r74 == 1) || !(1) || cap < G5P_R74_IRQ_CAP ||
		    cap > G5P_R74_IRQ_CAP_MAX ||
		    g5p_transport_event_limit() < G5P_R74_EVENT_FLOOR)) {
		dev_err(g->dev,
			"R74 reject phase=map case=%u r73b_case=%u r73f_case=%u meta6c8=%u "
			"render_engine=%u irq_cap=%u event_limit=%u end=r74\n",
			r74, id, 0, b->r72_meta6c8, (1), cap,
			g5p_transport_event_limit());
		return -EINVAL;
	}

	/* R74f rides only on the exact r74 case 1 forced-ready epoch. */
	;

	/* R74g rides only on the exact r74f case 1 epoch. */
	;

	/* R74h observes only the exact r74g case 1 epoch. */
	;

	/* The bounded serialized drain supersedes r73's IRQ-context drain. */
	;
	;
	;
	;
	if (id > 3 || fast > 1 || (fast && id == 3) || early > 2 ||
	    (early && (!fast || id != early)) || b->r73_aux_drain || 7 != 7 ||
	    1 != (id == 2 ? 0 : 1) || (ms && (ms < 1000 || ms > 30000)))
		return -EINVAL;
	r = devm_kzalloc(g->dev, sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;
	r->dev = g->dev;
	r->boot = b;
	r->case_id = id;
	r->watchdog_ms = ms;
	r->dc_variant = 7;
	r->meta6c8 = 1;
	r->aux = id != 3;
	r->ack_first = true;
	r->fast_enabled = fast;
	r->r73e_case = early;
	r->r74_case = r74;
	r->r74f_case = r74f;
	r->r74g_case = r74g;
	r->r74h_case = r74h;
	r->settle_us = min_t(u32, READ_ONCE(boot_alive_settle_us), 5000);
	r->nopatch = 1 && r74 == 1;
	r->irq_cap = r74 ? cap : G5P_R73B_IRQ_CAP;
	/* Heartbeat116: in an R74 epoch the window detector replaces both lifetime bounds. */
	r->hb_on = r74 != 0;
	r->hb_seed = r74 ? (0) : 0;
	g5p_hb_cfg_default(&r->hb_cfg);
	atomic_set(&r->early_once, 0);
	atomic_set(&r->early_busy, 0);
	atomic_set(&r->early_proof, G5P_PX_NONE);
	r->watchdog_cpu = -1;
	mutex_init(&r->drain_lock);
	hrtimer_setup(&r->watchdog, g5p_r73b_watchdog_fire, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_PINNED_HARD);
	ret = devm_add_action_or_reset(g->dev, g5p_r73b_stop, r);
	if (ret)
		return ret;
	b->r73_aux_drain = 1;
	b->r73b = r;
	dev_info(
		g->dev,
		"R73B latched case=%u name=%s ack_first=%u aux=%u cap=%u watchdog_ms=%u "
		"dc_variant=%u meta6c8=%u rows=%u release=%s end=r73b\n",
		id, g5p_r73b_name(id), r->ack_first, r->aux, r->irq_cap, ms,
		r->dc_variant, r->meta6c8, G5P_R73B_ROWS,
		init_utsname()->release);
	g5p_blob_printf(
		g,
		"R73B latched case=%u name=%s ack_first=%u aux=%u cap=%u watchdog_ms=%u "
		"dc_variant=%u meta6c8=%u rows=%u release=%s end=r73b\n",
		id, g5p_r73b_name(id), 1, 1, r->irq_cap, ms, 7, 1,
		G5P_R73B_ROWS, init_utsname()->release);
	g5p_blob_printf(g, "R73C latched fast=%u case=%u meta6c8=%u end=r73c\n",
			1, id, 1);
	dev_info(g->dev, "R73E latched case=%u meta6c8=%u fast=%u end=r73e\n",
		 early, r->meta6c8, r->fast_enabled);
	;
	;
	if (r->nopatch)
		dev_info(
			g->dev,
			"NOPATCH latched fw=apple-original r73f_case=0 meta6c8=%u r73e_case=%u r74=%u "
			"r74f=%u r74g=%u force_policy_ready=0 end=nopatch\n",
			r->meta6c8, early, r74, r74f, r74g);
	{
		dev_info(
			g->dev,
			"R74 latched case=%u name=%s r73f_case=%u r73e_case=%u meta6c8=%u irq_cap=%u "
			"event_limit=%u render_engine=typed-v3 irq_test=refused render_irq_tested=0 "
			"end=r74\n",
			r74, g5p_r74_name(r74), 0, early, r->meta6c8,
			r->irq_cap, g5p_transport_event_limit());
		g5p_blob_printf(
			g,
			"R74 latched case=%u name=%s r73f_case=%u r73e_case=%u meta6c8=%u irq_cap=%u "
			"event_limit=%u render_engine=typed-v3 irq_test=refused render_irq_tested=0 "
			"end=r74\n",
			r74, g5p_r74_name(r74), 0, early, 1, r->irq_cap,
			g5p_transport_event_limit());
		dev_info(
			g->dev,
			"HB116 latched on=%u window_ms=%llu irq_ceiling=%u event_ceiling=%u ratio=%u "
			"slack=%u stall_windows=%u seed=%u legacy_irq_cap=%u legacy_event_limit=%u "
			"lifetime_bound=none end=hb116\n",
			r->hb_on, r->hb_cfg.window_ns / NSEC_PER_MSEC,
			r->hb_cfg.irq_ceiling, r->hb_cfg.event_ceiling,
			r->hb_cfg.ratio, r->hb_cfg.slack,
			r->hb_cfg.stall_windows, r->hb_seed, r->irq_cap,
			g5p_transport_event_limit());
		g5p_blob_printf(
			g,
			"HB116 latched on=%u window_ms=%llu irq_ceiling=%u event_ceiling=%u ratio=%u "
			"slack=%u stall_windows=%u seed=%u legacy_irq_cap=%u legacy_event_limit=%u "
			"lifetime_bound=none end=hb116\n",
			r->hb_on, r->hb_cfg.window_ns / NSEC_PER_MSEC,
			r->hb_cfg.irq_ceiling, r->hb_cfg.event_ceiling,
			r->hb_cfg.ratio, r->hb_cfg.slack,
			r->hb_cfg.stall_windows, r->hb_seed, r->irq_cap,
			g5p_transport_event_limit());
	}
	{
		dev_info(
			g->dev,
			"R74F latched case=%u off_us=%u on_us=%u bell_ms=%u kick=10 "
			"owner=first-type0-alive-after-reactivation retained_failed=0 end=r74f\n",
			r74f, G5P_R74F_OFF_US, G5P_R74F_ON_US,
			G5P_R74F_BELL_MS);
		g5p_blob_printf(
			g,
			"R74F latched case=%u off_us=%u on_us=%u bell_ms=%u kick=10 "
			"owner=first-type0-alive-after-reactivation retained_failed=0 end=r74f\n",
			r74f, G5P_R74F_OFF_US, G5P_R74F_ON_US,
			G5P_R74F_BELL_MS);
	}
	{
		dev_info(
			g->dev,
			"R74G latched case=%u grace_alives=%u grace_us=%u fallback_bells_max=1 "
			"host_mmu=firmware-dc3 ack_ac8=suppressed companion=root+3980 end=r74g\n",
			r74g, G5P_R74G_GRACE_ALIVES, G5P_R74G_GRACE_US);
		g5p_blob_printf(
			g,
			"R74G latched case=%u grace_alives=%u grace_us=%u fallback_bells_max=1 "
			"host_mmu=firmware-dc3 ack_ac8=suppressed companion=root+3980 end=r74g\n",
			r74g, G5P_R74G_GRACE_ALIVES, G5P_R74G_GRACE_US);
	};
	/* TYPED_V3 production path: the self-drain DC3 attach policy of r87b rides on r74f + r74g
	 * alone, with none of the r74h / r80 instrumentation.  r74g has the firmware own the first
	 * MMU pass, which is what makes self-drain safe (r87b-attach.h); r74h only observes.  Every
	 * other r8x / r9x knob is still refused below without boot_r80_case.
	 */
	{
		r->r87b_attach = G5P_R87B_ATTACH_SELF_DRAIN;
		dev_info(
			g->dev,
			"R87B latched policy=1 name=self-drain r74f=%u r74g=%u r74h=%u r80=0 "
			"production_path=1 end=r87b\n",
			r74f, r74g, r74h);
	}
	{
		{
			{
				{
					{
						if ((1) && !r->r87b_attach) {
							dev_err(g->dev,
								"R87B reject phase=map r87b knob without boot_r80_case policy=%u end=r87b\n",
								(1));
							return -EBADMSG;
						} else {
							{
								{
									;
									;
								};
							};
						};
					};
				};
			};
		};
	}
	return 0;
}

/* Process context, cached CPU RAM only; never waits for the command lock. */

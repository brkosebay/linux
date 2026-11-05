/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Bounded RAM-only records. No register accesses, default off. */
#include <linux/sched/cputime.h>
#include "cache-policy.h"
static bool render_user_cache_private_wc;
module_param(render_user_cache_private_wc, bool, 0600);
MODULE_PARM_DESC(render_user_cache_private_wc, "Candidate: omit repeated maintenance only for native private write-combined BOs; imports/exports unchanged; default false");
static bool render_user_cache_receipts;
module_param(render_user_cache_receipts, bool, 0600);
MODULE_PARM_DESC(render_user_cache_receipts, "Bounded RAM-only per-job CPU/cache receipts, default false");
struct g5p_cache_receipt {
	u64 job, start_cpu, start_wall, total_cpu, total_wall;
	u64 requested[2][3], maintained[2][3], cpu[2][3], wall[2][3];
	u32 pid, serial;
	bool enabled, policy;
};
#define G5P_CACHE_RECORDS 128
static struct g5p_cache_receipt g5p_cache_ring[G5P_CACHE_RECORDS];
static DEFINE_SPINLOCK(g5p_cache_ring_lock);
static u32 g5p_cache_head;
static int g5p_cache_show(struct seq_file *m, void *data)
{
	struct g5p_cache_receipt r;
	u32 head = READ_ONCE(g5p_cache_head), begin = head > G5P_CACHE_RECORDS ? head - G5P_CACHE_RECORDS : 0;
	unsigned long flags;
	seq_puts(m, "v=1 classes=private-wc,native-other,import time=task-runtime-ns wall=elapsed-ns\n");
	for (u32 n = begin; n < head; n++) {
		spin_lock_irqsave(&g5p_cache_ring_lock, flags);
		r = g5p_cache_ring[n % G5P_CACHE_RECORDS];
		spin_unlock_irqrestore(&g5p_cache_ring_lock, flags);
		if (r.serial != n + 1) continue; /* overwritten during this debugfs read */
		seq_printf(m, "job=%llu pid=%u policy=%u total_cpu_ns=%llu total_wall_ns=%llu", r.job, r.pid, r.policy, r.total_cpu, r.total_wall);
		for (u32 phase = 0; phase < 2; phase++)
			for (u32 c = 0; c < 3; c++)
				seq_printf(m, " p%uc%u=%llu,%llu,%llu,%llu", phase, c, r.requested[phase][c], r.maintained[phase][c], r.cpu[phase][c], r.wall[phase][c]);
		seq_putc(m, '\n');
	}
	return 0;
}

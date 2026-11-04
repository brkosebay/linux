/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Cached append-only text. Readers never take the command lock or touch MMIO.
 * Preserve earliest records on overflow, expose dropped bytes explicitly.
 */
#define G5P_BLOB_SIZE SZ_4M
#define G5P_MMU_HISTORY_SIZE SZ_32K

struct g5p_diagnostics {
	spinlock_t lock;
	u32 used, history_used;
	u64 dropped, history_dropped;
	char stage[40];
	u32 next_register;
	bool reading;
	char history[G5P_MMU_HISTORY_SIZE];
	char blob[G5P_BLOB_SIZE];
};

static void g5p_diagnostics_free(void *data)
{
	kvfree(data);
}

static int g5p_diagnostics_init(struct g5p_diag *g)
{
	g->diagnostics = kvzalloc(sizeof(*g->diagnostics), GFP_KERNEL);
	if (!g->diagnostics)
		return -ENOMEM;
	spin_lock_init(&g->diagnostics->lock);
	return devm_add_action_or_reset(g->dev, g5p_diagnostics_free,
					g->diagnostics);
}

static void g5p_diagnostic_append(struct g5p_diag *g, bool history,
				  const char *text, size_t len)
{
	struct g5p_diagnostics *d = g->diagnostics;
	unsigned long flags;
	u32 *used = history ? &d->history_used : &d->used;
	u64 *dropped = history ? &d->history_dropped : &d->dropped;
	u32 capacity = history ? G5P_MMU_HISTORY_SIZE : G5P_BLOB_SIZE;
	char *dst = history ? d->history : d->blob;

	spin_lock_irqsave(&d->lock, flags);
	/* Keep complete lines; once full, never replace earlier evidence. */
	if (*dropped || len > capacity - *used) {
		*dropped += len;
	} else {
		memcpy(dst + *used, text, len);
		*used += len;
	}
	spin_unlock_irqrestore(&d->lock, flags);
}

static __printf(2, 3) void g5p_blob_printf(struct g5p_diag *g, const char *fmt,
					   ...)
{
	char line[512];
	va_list args;
	size_t len;

	va_start(args, fmt);
	len = vscnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	g5p_diagnostic_append(g, false, line, len);
}

static void g5p_blob_hex(struct g5p_diag *g, const char *name, const u8 *data,
			 u32 len)
{
	for (u32 off = 0; off < len; off += 16) {
		char text[80];

		hex_dump_to_buffer(data + off, min_t(u32, len - off, 16), 16, 4,
				   text, sizeof(text), false);
		g5p_blob_printf(g, "%s: %04x: %s\n", name, off, text);
	}
}

/* R1+1000 uses synthetic address11000; other offsets denote R0. */
static void g5p_mmu_progress(struct g5p_diag *g, const char *stage, u32 next)
{
	struct g5p_diagnostics *d = g->diagnostics;
	unsigned long flags;

	spin_lock_irqsave(&d->lock, flags);
	strscpy(d->stage, stage, sizeof(d->stage));
	d->next_register = next;
	d->reading = true;
	spin_unlock_irqrestore(&d->lock, flags);
}

static __printf(2, 3) void g5p_mmu_record(struct g5p_diag *g, const char *fmt,
					  ...)
{
	struct g5p_diagnostics *d = g->diagnostics;
	char line[256];
	unsigned long flags;
	va_list args;
	size_t len;

	va_start(args, fmt);
	len = vscnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	g5p_diagnostic_append(g, true, line, len);
	spin_lock_irqsave(&d->lock, flags);
	d->reading = false;
	spin_unlock_irqrestore(&d->lock, flags);
	dev_info(g->dev, "%s", line);
}

/* The routine per-job and per-event lines (a job's publish, wake and success receipt, the firmware's
 * alive and stamp events).  At desktop job rates they are hundreds of lines a second, more than netconsole
 * can send: its queue grew to 700 MB of unreclaimable slab (release8, 55 min).  Faults, losses,
 * recoveries, terminal latches and every failed job keep their own levels and always print.
 *   job_log 0  dev_dbg (dynamic debug: off unless enabled in dynamic_debug/control)
 *           1  KERN_DEBUG: in dmesg for the parsers (RENDER-USER job=, RENDER publish), below the
 *              default console loglevel, so not on a console or netconsole
 *           2  KERN_INFO as before (short netconsole traces only: floods at desktop job rates)
 */
static unsigned int job_log = 0;
module_param(job_log, uint, 0600);
MODULE_PARM_DESC(
	job_log,
	"default0 routine per-job/per-event lines are dev_dbg; 1 KERN_DEBUG (dmesg, no "
	"console); 2 KERN_INFO as before (floods netconsole)");

#define g5p_job_printk(dev, fmt, ...)                                    \
	do {                                                             \
		switch (READ_ONCE(job_log)) {                            \
		case 0:                                                  \
			dev_dbg(dev, fmt, ##__VA_ARGS__);                \
			break;                                           \
		case 1:                                                  \
			dev_printk(KERN_DEBUG, dev, fmt, ##__VA_ARGS__); \
			break;                                           \
		default:                                                 \
			dev_info(dev, fmt, ##__VA_ARGS__);               \
			break;                                           \
		}                                                        \
	} while (0)

/* g5p_mmu_record for a routine per-job line: the blob keeps it, the console sees it only at job_log 2. */
static __printf(2, 3) __maybe_unused
	void g5p_mmu_record_job(struct g5p_diag *g, const char *fmt, ...)
{
	struct g5p_diagnostics *d = g->diagnostics;
	char line[256];
	unsigned long flags;
	va_list args;
	size_t len;

	va_start(args, fmt);
	len = vscnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	g5p_diagnostic_append(g, true, line, len);
	spin_lock_irqsave(&d->lock, flags);
	d->reading = false;
	spin_unlock_irqrestore(&d->lock, flags);
	g5p_job_printk(g->dev, "%s", line);
}

/* A routine event's raw words at the job_log level. */
static __maybe_unused void g5p_job_hex(const char *prefix, const void *buf,
				       size_t len)
{
	switch (READ_ONCE(job_log)) {
	case 0:
		print_hex_dump_debug(prefix, DUMP_PREFIX_NONE, 16, 4, buf, len,
				     false);
		break;
	case 1:
		print_hex_dump(KERN_DEBUG, prefix, DUMP_PREFIX_NONE, 16, 4, buf,
			       len, false);
		break;
	default:
		print_hex_dump(KERN_INFO, prefix, DUMP_PREFIX_NONE, 16, 4, buf,
			       len, false);
		break;
	}
}

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_HEARTBEAT116_H
#define G5P_HEARTBEAT116_H

/*
 * Heartbeat116: the R74 IRQ174 storm bound as a window rate, not a lifetime count.
 *
 * A healthy R74 epoch takes IRQ174 at about 575/s and consumes about 290 event-ring
 * entries/s, idle or not: one type-0 ALIVE per firmware scheduler iteration, and two IRQ
 * entries per ring entry (irqs = 2 * events - 18 over the Warm113 C4-prep R74 milestones;
 * r81 measured 577/289 per s).  Wake112 bounded both counts over the epoch's lifetime:
 * IRQ entries against boot_r74_irq_cap (1048576) and ring entries against the compute event
 * limit (524544).  Either bound latched failed/event_failed, so an idle, healthy GPU faulted
 * after about 30 minutes.
 *
 * Here both bounds are rates.  A window opens at the first IRQ entry and closes at the first
 * entry at least window_ns later, then re-bases on the totals of that entry.  Totals are u32
 * and every in-window count is a u32 difference, so a wrap of a total is harmless.
 *
 *   RATE    more than irq_ceiling IRQ entries in the open window (checked on every entry)
 *   EVENTS  more than event_ceiling ring entries in the open window (checked on every entry)
 *   STALL   stall_windows consecutive closed windows in which
 *           irqs > ratio * events + slack.  Healthy traffic is 2:1.  A latch that re-fires
 *           without ring progress, or a heartbeat after a fault stopped the drain, is not.
 *
 * A trip is absorbing.  Nothing here clears a failure; the glue adds the same terminal stop
 * as the lifetime cap had (cap_seen, failed, event_failed, one disable_irq_nosync).
 *
 * Pure policy: no register access, no lock, no kernel API.  The caller serializes (the R73B
 * IRQ thread is the only caller).
 */

#define G5P_HB_WINDOW_NS 1000000000ULL
#define G5P_HB_IRQ_CEILING 16384U
#define G5P_HB_EVENT_CEILING 8192U
#define G5P_HB_RATIO 8U
#define G5P_HB_SLACK 512U
#define G5P_HB_STALL_WINDOWS 3U

enum g5p_hb_why {
	G5P_HBW_NONE,
	G5P_HBW_RATE,
	G5P_HBW_EVENTS,
	G5P_HBW_STALL,
	G5P_HBW_N,
};

static const char *const g5p_hb_why_name[G5P_HBW_N] = {
	"none",
	"rate",
	"events",
	"stall",
};

enum g5p_hb_res {
	G5P_HBR_OPEN, /* counted in the open window */
	G5P_HBR_CLOSED, /* this entry closed a window (h->last) */
	G5P_HBR_TRIP, /* tripped now or earlier */
};

struct g5p_hb_cfg {
	u64 window_ns;
	u32 irq_ceiling, event_ceiling, ratio, slack, stall_windows;
};

/* The totals the IRQ thread samples after servicing one entry. */
struct g5p_hb_in {
	u64 now_ns;
	u32 irqs; /* IRQ174 entries (b->irqs) */
	u32 events; /* event-ring entries consumed (b->events) */
	u32 acks; /* AIC ACKs written by the R73B handler */
	u32 empty; /* entries whose service consumed no ring entry */
	u32 aux; /* auxiliary-ring entries consumed */
};

/* One window: its start and length and its in-window counts. */
struct g5p_hb_win {
	u64 start_ns, ns;
	u32 irqs, events, acks, empty, aux;
};

struct g5p_hb {
	u32 open; /* zero until the first entry (kzalloc) */
	u32 why; /* enum g5p_hb_why, written once */
	u32 windows; /* windows closed */
	u32 stalled; /* consecutive unproductive closed windows */
	u32 peak_irqs, peak_events;
	u64 start_ns;
	struct g5p_hb_in base; /* totals at the window's first entry */
	struct g5p_hb_win last; /* the last closed window */
	struct g5p_hb_win trip; /* the open window at the trip */
	u64 trip_ns;
};

static inline void g5p_hb_cfg_default(struct g5p_hb_cfg *c)
{
	c->window_ns = G5P_HB_WINDOW_NS;
	c->irq_ceiling = G5P_HB_IRQ_CEILING;
	c->event_ceiling = G5P_HB_EVENT_CEILING;
	c->ratio = G5P_HB_RATIO;
	c->slack = G5P_HB_SLACK;
	c->stall_windows = G5P_HB_STALL_WINDOWS;
}

static inline void g5p_hb_count(const struct g5p_hb *h,
				const struct g5p_hb_in *in,
				struct g5p_hb_win *w)
{
	w->start_ns = h->start_ns;
	w->ns = in->now_ns >= h->start_ns ? in->now_ns - h->start_ns : 0;
	w->irqs = in->irqs - h->base.irqs;
	w->events = in->events - h->base.events;
	w->acks = in->acks - h->base.acks;
	w->empty = in->empty - h->base.empty;
	w->aux = in->aux - h->base.aux;
}

static inline bool g5p_hb_unproductive(const struct g5p_hb_cfg *c,
				       const struct g5p_hb_win *w)
{
	return (u64)w->irqs > (u64)c->ratio * w->events + c->slack;
}

static inline enum g5p_hb_res
g5p_hb_trip(struct g5p_hb *h, u32 why, const struct g5p_hb_win *w, u64 now_ns)
{
	h->why = why;
	h->trip = *w;
	h->trip_ns = now_ns;
	return G5P_HBR_TRIP;
}

/* One serviced IRQ entry. */
static inline enum g5p_hb_res g5p_hb_step(struct g5p_hb *h,
					  const struct g5p_hb_cfg *c,
					  const struct g5p_hb_in *in)
{
	struct g5p_hb_win w;

	if (h->why)
		return G5P_HBR_TRIP;
	if (!h->open) {
		h->open = 1;
		h->start_ns = in->now_ns;
		h->base = *in;
		return G5P_HBR_OPEN;
	}
	g5p_hb_count(h, in, &w);
	if (w.irqs > c->irq_ceiling)
		return g5p_hb_trip(h, G5P_HBW_RATE, &w, in->now_ns);
	if (w.events > c->event_ceiling)
		return g5p_hb_trip(h, G5P_HBW_EVENTS, &w, in->now_ns);
	if (w.ns < c->window_ns)
		return G5P_HBR_OPEN;
	h->last = w;
	h->windows++;
	if (w.irqs > h->peak_irqs)
		h->peak_irqs = w.irqs;
	if (w.events > h->peak_events)
		h->peak_events = w.events;
	h->stalled = g5p_hb_unproductive(c, &w) ? h->stalled + 1 : 0;
	h->start_ns = in->now_ns;
	h->base = *in;
	if (h->stalled >= c->stall_windows)
		return g5p_hb_trip(h, G5P_HBW_STALL, &w, in->now_ns);
	return G5P_HBR_CLOSED;
}

/* Receipts: the first four closed windows, then every 64th. */
static inline bool g5p_hb_report(const struct g5p_hb *h)
{
	return h->windows <= 4 || !(h->windows % 64);
}

#endif /* G5P_HEARTBEAT116_H */

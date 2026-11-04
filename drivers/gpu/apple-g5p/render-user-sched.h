/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Render job scheduling across contexts (docs/gpu/multi-client-design.md, stage 2), shaped
 * like drm_sched so that stage 4 can swap drm_sched in without a UAPI or structural change:
 *
 *   entity  = one render context (drm_sched_entity): a FIFO of its accepted jobs, and at
 *             most one close item, which runs after its last job (drm_sched_entity_destroy());
 *   run list = the entities with work, served round-robin: each pick takes the first entity
 *             whose head job is ready and moves it to the tail (drm_sched's RR policy);
 *   ready   = every dependency of the head job has signalled, it failed before the hardware,
 *             its input deadline passed, or admission closed for good (the job then fails
 *             without the hardware).  Readiness is the caller's predicate; the kernel's adds
 *             dma-fence callbacks (drm_sched_job_add_dependency()) so that an unsignalled
 *             input of one client never blocks another client's jobs.
 *
 * One credit: the single shared TA/3D channel and PM record run one job at a time, so the
 * caller runs each picked item to completion before the next pick.  Only the head of an
 * entity is looked at, so each context's fences signal in submission order.
 *
 * The lists are plain list_heads; the caller holds its scheduler lock around every call.
 * Host-tested (scripts/gpu/multiclient2-host/test_render_sched.c).
 */
#ifndef G5P_RENDER_USER_SCHED_H
#define G5P_RENDER_USER_SCHED_H

struct g5p_rs_job {
	struct list_head link;
};

struct g5p_rs_entity {
	struct list_head link; /* in g5p_rs.run while it has work */
	struct list_head jobs; /* FIFO of g5p_rs_job */
	bool in_run, close_pending;
};

struct g5p_rs {
	struct list_head run;
};

/* g5p_rs_pick() result */
enum g5p_rs_pick {
	G5P_RS_IDLE, /* nothing ready; *wake holds the earliest deadline if any job waits */
	G5P_RS_JOB, /* *job was popped from *entity */
	G5P_RS_CLOSE, /* *entity has no job left and its close item is due */
};

/* Head-job predicate: true when it may run now.  *deadline is the time at which it becomes
 * ready on its own (its input deadline); the predicate leaves it alone if there is none.
 */
typedef bool (*g5p_rs_ready_fn)(struct g5p_rs_job *job, void *cookie,
				unsigned long *deadline);

static inline void g5p_rs_init(struct g5p_rs *s)
{
	INIT_LIST_HEAD(&s->run);
}

static inline void g5p_rs_entity_init(struct g5p_rs_entity *e)
{
	INIT_LIST_HEAD(&e->link);
	INIT_LIST_HEAD(&e->jobs);
	e->in_run = false;
	e->close_pending = false;
}

static inline void g5p_rs_enlist(struct g5p_rs *s, struct g5p_rs_entity *e)
{
	if (!e->in_run) {
		list_add_tail(&e->link, &s->run);
		e->in_run = true;
	}
}

/* An accepted job joins the end of its entity's FIFO. */
static inline void g5p_rs_push(struct g5p_rs *s, struct g5p_rs_entity *e,
			       struct g5p_rs_job *job)
{
	list_add_tail(&job->link, &e->jobs);
	g5p_rs_enlist(s, e);
}

/* The entity's close item: due once its FIFO is empty.  False if one is already queued. */
static inline bool g5p_rs_close(struct g5p_rs *s, struct g5p_rs_entity *e)
{
	if (e->close_pending)
		return false;
	e->close_pending = true;
	g5p_rs_enlist(s, e);
	return true;
}

/* After the close item ran: the entity leaves the scheduler for good. */
static inline void g5p_rs_closed(struct g5p_rs *s, struct g5p_rs_entity *e)
{
	(void)s;
	e->close_pending = false;
	if (e->in_run) {
		list_del_init(&e->link);
		e->in_run = false;
	}
}

/*
 * The next item to run, round-robin over the entities with work.  A picked entity moves to
 * the tail; one left without a job and without a close item leaves the run list.  Entities
 * whose head is not ready keep their place.  `has_wake` reports whether *wake is set.
 */
static inline int g5p_rs_pick(struct g5p_rs *s, g5p_rs_ready_fn ready,
			      void *cookie, struct g5p_rs_entity **entity,
			      struct g5p_rs_job **job, unsigned long *wake,
			      bool *has_wake)
{
	struct g5p_rs_entity *e;

	*has_wake = false;
	list_for_each_entry(e, &s->run, link) {
		struct g5p_rs_job *head;
		unsigned long deadline;
		bool timed = false;

		if (list_empty(&e->jobs)) {
			/* Without a close item it left the run list at its last pick. */
			if (!e->close_pending)
				continue;
			*entity = e;
			*job = NULL;
			list_move_tail(&e->link, &s->run);
			return G5P_RS_CLOSE;
		}
		head = list_first_entry(&e->jobs, struct g5p_rs_job, link);
		deadline = 0;
		if (!ready(head, cookie, &deadline)) {
			timed = deadline != 0;
			if (timed &&
			    (!*has_wake || (long)(deadline - *wake) < 0)) {
				*wake = deadline;
				*has_wake = true;
			}
			continue;
		}
		list_del_init(&head->link);
		*entity = e;
		*job = head;
		if (list_empty(&e->jobs) && !e->close_pending) {
			list_del_init(&e->link);
			e->in_run = false;
		} else {
			list_move_tail(&e->link, &s->run);
		}
		return G5P_RS_JOB;
	}
	return G5P_RS_IDLE;
}

#endif

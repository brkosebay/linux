/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Firmware slot pairs of render contexts (docs/gpu/multi-client-design.md, stage 1).
 *
 * Each render context owns a TA and a 3D firmware slot in 7..255, both pointing at its own
 * page catalogue.  The pair is bound once by two DC3 records {3, slot, 0, 0, 0}, 3D first,
 * then TA, with the catalogue PA in 0x86[slot] and the host tag in 0x84[slot].  The firmware's
 * DC3 handler installs CBASE and copies the host tag to 0x85[slot].
 *
 * These helpers only read and write those RAM words, so they cannot reach a register and are
 * safe after a job has retired.  Pure functions, host-tested
 * (scripts/gpu/multiclient1-host/test_render_bind.c).  The caller provides g5p_le32(),
 * g5p_put32() and g5p_put64().
 */
#ifndef G5P_RENDER_USER_BIND_H
#define G5P_RENDER_USER_BIND_H

#define G5P_RU_SLOT_MIN 7U
#define G5P_RU_SLOT_MAX 255U
/* RAM-only wait for the firmware to consume a published DC3 batch. */
#define G5P_RU_BIND_MS 3000U
/* Host tag of a pre-bound spare pair (render_user_spare_pairs): never zero. */
#define G5P_RU_SPARE_TAG 0x00400000U
/* Host tag of a parked pair (render_user_close_park): never zero, never a spare's. */
#define G5P_RU_PARK_TAG 0x00500000U
#define G5P_RU_MAX_SPARES 32U

/* The words one pair's binding touches.  Pointers, because the DC records may straddle the
 * shared pages.  Index [e]: 0 TA slot, 1 3D slot.
 */
struct g5p_ru_bind_mem {
	u8 *rec[2][5]; /* the records at start and start + 1 */
	u8 *pa[2]; /* 0x86[slot], 64 bit */
	u8 *host[2]; /* 0x84[slot] */
	u8 *cached[2]; /* 0x85[slot] */
};

/* One RAM sample of those words and of the DC ring control. */
struct g5p_ru_bind_sample {
	u32 cons, prod;
	u32 rec[2][5];
	u64 pa[2];
	u32 host[2], cached[2];
};

/* g5p_ru_bind_busy() */
#define G5P_RU_BIND_BUSY_RING 0x1U /* the DC ring is not idle */
#define G5P_RU_BIND_BUSY_SLOT 0x2U /* a slot has a PA or a host tag */

/* g5p_ru_bind_check(), the r74g criteria */
#define G5P_RU_BIND_BAD_RING 0x1U
#define G5P_RU_BIND_BAD_RECORD 0x2U
#define G5P_RU_BIND_BAD_PA 0x4U
#define G5P_RU_BIND_BAD_HOST 0x8U
#define G5P_RU_BIND_BAD_CACHED 0x10U

enum g5p_ru_bind_wait {
	G5P_RU_BIND_PENDING,
	G5P_RU_BIND_CONSUMED,
	G5P_RU_BIND_PROTO,
	G5P_RU_BIND_TIMEOUT,
};

static inline bool g5p_ru_slot_used(const unsigned long *map, u32 slot)
{
	return (map[slot / BITS_PER_LONG] >> (slot % BITS_PER_LONG)) & 1;
}

/* The slot pair of a new render context: the two lowest free slots in 7..255.  The lower one
 * is the 3D slot, or the TA slot with ta_first (render_user_slot_variant 1).  False when fewer
 * than two are free.  slot[0] is TA, slot[1] is 3D.
 */
static inline bool g5p_ru_slot_pair(const unsigned long *map, bool ta_first,
				    u32 *slot)
{
	u32 pick[2], n = 0;

	for (u32 s = G5P_RU_SLOT_MIN; s <= G5P_RU_SLOT_MAX && n < 2; s++)
		if (!g5p_ru_slot_used(map, s))
			pick[n++] = s;
	if (n < 2)
		return false;
	slot[0] = pick[ta_first ? 0 : 1];
	slot[1] = pick[ta_first ? 1 : 0];
	return true;
}

/* Word j of record i of a pair: {3, slot, 0, 0, 0}, the 3D slot's record first. */
static inline u32 g5p_ru_bind_record(const u32 *slot, u32 i, u32 j)
{
	return j == 0 ? 3 : j == 1 ? slot[1 - i] : 0;
}

static inline void g5p_ru_bind_read(const struct g5p_ru_bind_mem *m, u32 cons,
				    u32 prod, struct g5p_ru_bind_sample *s)
{
	s->cons = cons;
	s->prod = prod;
	for (u32 i = 0; i < 2; i++)
		for (u32 j = 0; j < 5; j++)
			s->rec[i][j] = g5p_le32(m->rec[i][j]);
	for (u32 e = 0; e < 2; e++) {
		s->pa[e] = (u64)g5p_le32(m->pa[e] + 4) << 32 |
			   g5p_le32(m->pa[e]);
		s->host[e] = g5p_le32(m->host[e]);
		s->cached[e] = g5p_le32(m->cached[e]);
	}
}

/* Before a publish: the ring must be idle and both slots unused (zero PA and host tag). */
static inline u32 g5p_ru_bind_busy(const struct g5p_ru_bind_sample *s)
{
	u32 busy = 0;

	if (s->prod > 255 || s->cons != s->prod)
		busy |= G5P_RU_BIND_BUSY_RING;
	for (u32 e = 0; e < 2; e++)
		if (s->pa[e] || s->host[e])
			busy |= G5P_RU_BIND_BUSY_SLOT;
	return busy;
}

/*
 * A parked pair (render_user_close_park, release8 quiet2): a closed context's slots, bound to the
 * kernel's park catalogue (it maps only the firmware window) with the park tag, so that the PM's
 * late write-back of that context's free list resolves instead of faulting on an empty PC level.
 * Both slots on `park`, both host tags the park tag.  park 0: nothing is parked.
 */
static inline bool g5p_ru_bind_parked(const struct g5p_ru_bind_sample *s,
				      u64 park)
{
	if (!park)
		return false;
	for (u32 e = 0; e < 2; e++)
		if (s->pa[e] != park || s->host[e] != G5P_RU_PARK_TAG)
			return false;
	return true;
}

/* PA and tag of both slots, then the two records.  The caller cleans the catalogue first and
 * stores the producer afterwards, each behind a barrier.
 */
static inline void g5p_ru_bind_publish(const struct g5p_ru_bind_mem *m,
				       const u32 *slot, u64 root, u32 tag)
{
	for (u32 i = 0; i < 2; i++) {
		u32 e = 1 - i;

		g5p_put64(m->pa[e], root);
		g5p_put32(m->host[e], tag);
		for (u32 j = 0; j < 5; j++)
			g5p_put32(m->rec[i][j], g5p_ru_bind_record(slot, i, j));
	}
}

/* After the ring reached `target`: the records intact, both slots on `root`, and the firmware
 * tag equal to the host tag (the DC3 handler ran for both).
 */
static inline u32 g5p_ru_bind_check(const struct g5p_ru_bind_sample *s,
				    const u32 *slot, u64 root, u32 tag,
				    u32 target)
{
	u32 bad = 0;

	if (s->cons != target || s->prod != target)
		bad |= G5P_RU_BIND_BAD_RING;
	for (u32 i = 0; i < 2; i++)
		for (u32 j = 0; j < 5; j++)
			if (s->rec[i][j] != g5p_ru_bind_record(slot, i, j))
				bad |= G5P_RU_BIND_BAD_RECORD;
	for (u32 e = 0; e < 2; e++) {
		if (s->pa[e] != root)
			bad |= G5P_RU_BIND_BAD_PA;
		if (s->host[e] != tag)
			bad |= G5P_RU_BIND_BAD_HOST;
		if (s->cached[e] != tag)
			bad |= G5P_RU_BIND_BAD_CACHED;
	}
	return bad;
}

/* One sample of a published batch.  The caller takes `expired` before reading the ring, so
 * the sample taken at the deadline still counts.
 */
static inline int g5p_ru_bind_step(u32 cons, u32 prod, u32 target, bool expired)
{
	if (prod != target || cons > 255)
		return G5P_RU_BIND_PROTO;
	if (cons == target)
		return G5P_RU_BIND_CONSUMED;
	return expired ? G5P_RU_BIND_TIMEOUT : G5P_RU_BIND_PENDING;
}

/*
 * Unbind (iOS AGXGart_detachSlotsAndDestroyAddressSpace, firmware-re README round 15 §1): PA
 * and host tag of a slot go to zero and one DC3 record {3, slot, 0, 0, 0} follows.  The DC3
 * handler then installs CBASE 0, copies the zero tag to 0x85[slot] and invalidates the slot's
 * PC level.  The slots of one batch take consecutive ring entries, one record each, behind a
 * single producer store.  The caller must have retired every use of the slots first.
 */
#define G5P_RU_USER_SLOTS (G5P_RU_SLOT_MAX - G5P_RU_SLOT_MIN + 1)

struct g5p_ru_slot_mem {
	u8 *rec[5]; /* the slot's record in the batch */
	u8 *pa, *host,
		*cached; /* 0x86[slot] (64 bit), 0x84[slot], 0x85[slot] */
};

struct g5p_ru_slot_sample {
	u32 cons, prod;
	u32 rec[5];
	u64 pa;
	u32 host, cached;
};

/* The user slots the firmware may hold: those with a PA or a host tag.  At most `max`. */
static inline u32 g5p_ru_held_slots(const u8 *pa, const u8 *host, u32 *slots,
				    u32 max)
{
	u32 n = 0;

	for (u32 s = G5P_RU_SLOT_MIN; s <= G5P_RU_SLOT_MAX && n < max; s++)
		if (g5p_le32(pa + s * 8) || g5p_le32(pa + s * 8 + 4) ||
		    g5p_le32(host + s * 4))
			slots[n++] = s;
	return n;
}

/* A batch of n records fits the 256-entry ring behind an idle consumer. */
static inline bool g5p_ru_unbind_fits(u32 n)
{
	return n && n <= G5P_RU_USER_SLOTS;
}

static inline u32 g5p_ru_unbind_record(u32 slot, u32 j)
{
	return j == 0 ? 3 : j == 1 ? slot : 0;
}

static inline void g5p_ru_slot_read(const struct g5p_ru_slot_mem *m, u32 cons,
				    u32 prod, struct g5p_ru_slot_sample *s)
{
	s->cons = cons;
	s->prod = prod;
	for (u32 j = 0; j < 5; j++)
		s->rec[j] = g5p_le32(m->rec[j]);
	s->pa = (u64)g5p_le32(m->pa + 4) << 32 | g5p_le32(m->pa);
	s->host = g5p_le32(m->host);
	s->cached = g5p_le32(m->cached);
}

/* PA and host tag to zero, then the record.  The caller stores the producer behind a barrier. */
static inline void g5p_ru_unbind_publish(const struct g5p_ru_slot_mem *m,
					 u32 slot)
{
	g5p_put64(m->pa, 0);
	g5p_put32(m->host, 0);
	for (u32 j = 0; j < 5; j++)
		g5p_put32(m->rec[j], g5p_ru_unbind_record(slot, j));
}

/* After the ring reached `target`: the record intact, PA and host tag still zero, and the
 * firmware's tag zero too, which only its DC3 handler writes (the slot was bound with a
 * non-zero tag).  The G5P_RU_BIND_BAD_* bits.
 */
static inline u32 g5p_ru_unbind_check(const struct g5p_ru_slot_sample *s,
				      u32 slot, u32 target)
{
	u32 bad = 0;

	if (s->cons != target || s->prod != target)
		bad |= G5P_RU_BIND_BAD_RING;
	for (u32 j = 0; j < 5; j++)
		if (s->rec[j] != g5p_ru_unbind_record(slot, j))
			bad |= G5P_RU_BIND_BAD_RECORD;
	if (s->pa)
		bad |= G5P_RU_BIND_BAD_PA;
	if (s->host)
		bad |= G5P_RU_BIND_BAD_HOST;
	if (s->cached)
		bad |= G5P_RU_BIND_BAD_CACHED;
	return bad;
}

#endif

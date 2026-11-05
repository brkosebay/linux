/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * CDM channel and per-job firmware objects. Layout helpers perform no I/O.
 * The kernel owns the channel, control watermarks, pointer ring, work record,
 * completion microsequence, timestamp queue, stamps and dependency object.
 * GPU program instructions are provided by userspace, never embedded here.
 *
 * Compute uses the context's TA MMU slot. The scheduler serializes TA and CDM
 * work across contexts, and a lost CDM job blocks the channel until retirement
 * or recovery. Stamp identity zero is safe because the TA event latch is armed
 * only while executing a TA phase; completion also requires coherent stamps.
 */
#ifndef G5P_COMPUTE_USER_LAYOUT_H
#define G5P_COMPUTE_USER_LAYOUT_H

#define G5P_CU_ENGINE 2U
#define G5P_CU_OFFSET (G5P_R22_3D + 0x3000U) /* in the shared region */
#define G5P_CU_BYTES 0x1000U
#define G5P_CU_W 0x400U
#define G5P_CU_W_BYTES 0x290U /* through the timestamps at W+0x280..0x288 */
#define G5P_CU_SKU 0x800U
#define G5P_CU_SKU_BYTES 0xc0U
#define G5P_CU_TQ 0x900U
#define G5P_CU_START 0x940U /* W258: the retirement mirror */
#define G5P_CU_END 0x944U /* W25c: finish33's stamp */
#define G5P_CU_DEP 0x980U
#define G5P_CU_DEP_BYTES 0x280U
#define G5P_CU_IDENTITY 0U
#define G5P_CU_SEQ_MAX 0x7fffffffU

/* Firmware configuration W+0x244 and compute record type. */
#define G5P_CU_W244 0x00a00004U
#define G5P_CU_TAG 3U

/* The validated register images of one job (compute-user-validate.h fills it). */
struct g5p_cu_regs {
	u64 stream; /* W+0x70 -> R0+0x480 */
	u64 terminate; /* the stream's terminate word: W+0x214, W+0x23c */
	u64 state[4]; /* W+0x1ec..0x204 */
	u64 tail; /* W+0x60 -> R0+0x498, W+0x20c */
	u32 config[2]; /* W+0x10, W+0x14 -> R0+0x4c0, 0x4c8 */
	/* {code | data << 32}: W+0x21c, W+0x22c, W+0x68 -> R0+0x4b8 */
	u64 store, resume, term;
	u32 store1, resume1; /* W+0x224 -> R0+0x4b0, W+0x234 */
};

/* Job `seq` (1-based) occupies pointer-ring entry (seq - 1) % 16 and publishes seq % 16. */
static inline u32 g5p_cu_ring_index(u32 seq)
{
	return (seq - 1) % G5P_WORK_SLOTS;
}

static inline u32 g5p_cu_endpoint(u32 seq)
{
	return seq % G5P_WORK_SLOTS;
}

/* The channel at its epoch start: header, ring capacity, timestamp queue, dependency object
 * (g5p_compute_dependency()); watermarks, ring and stamps zero.  `p` is the channel's
 * G5P_CU_BYTES, `meta` its META address.
 */
static inline void g5p_cu_channel_init(u8 *p, u32 meta)
{
	u8 *d = p + G5P_CU_DEP;

	memset(p, 0, G5P_CU_BYTES);
	g5p_put32(p, meta + G5P_WORK_CTL);
	g5p_put32(p + 4, meta + G5P_WORK_RING);
	g5p_put32(p + 8, meta + G5P_CU_TQ);
	g5p_put32(p + 0xc, 4);
	g5p_put32(p + 0x18, ~0U);
	g5p_put32(p + G5P_WORK_CTL + 0x50, G5P_WORK_SLOTS);
	g5p_put32(p + G5P_CU_TQ + 4, meta + G5P_CU_TQ);
	g5p_put64(d + 0x30, 1);
	g5p_put64(d + 0x48, ~0ULL);
	g5p_put32(d + 0x50, ~0U);
	for (u32 off = 0x70; off < 0x270; off += 0x10)
		g5p_put32(d + off + 8, 0xf);
}

/* The SKU microsequence of job `seq`: start32, timestamp, wait COMPUTE_FINISHED (event 4),
 * wait (R0+0x4a0 & 1) == 1, timestamp, finish33 (W25c = W260), the backlink, end.
 * Each sequence reference uses the current job number.
 */
static inline u32 g5p_cu_sku(u8 *sku, u32 meta, u32 global, u32 slot, u32 seq)
{
	u32 w = meta + G5P_CU_W, end = meta + G5P_CU_END;
	const u32 words[] = {
		0x32,
		w + 0x10,
		w + 0x60,
		global + 0x720,
		meta,
		slot,
		0,
		0,
		0,
		w + 0x1e8,
		0,
		0,
		seq,
		0x80000018,
		w + 0x278,
		w + 0x280,
		w + 0x280,
		seq,
		0x101,
		0x004a000a,
		1,
		1,
		0x18,
		w + 0x278,
		w + 0x280,
		w + 0x284,
		seq,
		0x33,
		global + 0x720,
		meta,
		slot,
		w + 0x1e8,
		0,
		0,
		seq,
		end,
		seq,
		(u32)-0x6c,
		0x40000017,
	};

	memset(sku, 0, G5P_CU_SKU_BYTES);
	for (u32 i = 0; i < ARRAY_SIZE(words); i++)
		g5p_put32(sku + i * 4, words[i]);
	return ARRAY_SIZE(words) * 4;
}

/* Every per-job object of job `seq` on slot `slot`: W, SKU and its pointer-ring entry.  The
 * channel header, the dependency object and the timestamp queue stay as initialised.
 */
static inline void g5p_cu_job(u8 *p, u32 meta, u32 global, u32 slot, u32 seq,
			      const struct g5p_cu_regs *k)
{
	u8 *w = p + G5P_CU_W;

	memset(w, 0, G5P_CU_W_BYTES);
	g5p_put32(w, G5P_CU_TAG);
	g5p_put32(w + 8, slot);
	g5p_put32(w + 0xc, meta + G5P_CU_DEP);
	g5p_put32(w + 0x10, k->config[0]);
	g5p_put32(w + 0x14, k->config[1]);
	g5p_put64(w + 0x60, k->tail);
	g5p_put64(w + 0x68, k->term);
	g5p_put64(w + 0x70, k->stream);
	/* The six selected-resource entries at W+0x78..0xa7 stay zero (disabled). */
	g5p_put32(w + 0x1e0, meta + G5P_CU_SKU);
	g5p_put32(w + 0x1e4, G5P_CU_SKU_BYTES);
	for (u32 i = 0; i < 4; i++)
		g5p_put64(w + 0x1ec + i * 8, k->state[i]);
	g5p_put64(w + 0x20c, k->tail);
	g5p_put64(w + 0x214, k->terminate);
	g5p_put64(w + 0x21c, k->store);
	g5p_put32(w + 0x224, k->store1);
	g5p_put64(w + 0x22c, k->resume);
	g5p_put32(w + 0x234, k->resume1);
	g5p_put64(w + 0x23c, k->terminate);
	g5p_put32(w + 0x244, G5P_CU_W244);
	g5p_put32(w + 0x258, meta + G5P_CU_START);
	g5p_put32(w + 0x25c, meta + G5P_CU_END);
	g5p_put32(w + 0x260, seq);
	g5p_put32(w + 0x264, G5P_CU_IDENTITY);
	g5p_put32(w + 0x270, seq);
	g5p_cu_sku(p + G5P_CU_SKU, meta, global, slot, seq);
	g5p_put32(p + G5P_WORK_RING + g5p_cu_ring_index(seq) * 4,
		  meta + G5P_CU_W);
}

/* Retirement of job `seq` (the r19d predicate, RAM only): W still names our stamps and
 * sequence, finish33's stamp and the retirement mirror both at seq, all five channel
 * watermarks at its endpoint.
 */
static inline bool g5p_cu_retired(const u8 *p, u32 meta, u32 seq)
{
	const u8 *w = p + G5P_CU_W;
	u32 endpoint = g5p_cu_endpoint(seq);

	if (g5p_le32(w + 0x258) != meta + G5P_CU_START ||
	    g5p_le32(w + 0x25c) != meta + G5P_CU_END ||
	    g5p_le32(w + 0x260) != seq || g5p_le32(p + G5P_CU_START) != seq ||
	    g5p_le32(p + G5P_CU_END) != seq)
		return false;
	for (u32 off = 0; off <= 0x40; off += 0x10)
		if (g5p_le32(p + G5P_WORK_CTL + off) != endpoint)
			return false;
	return true;
}

/* The stamps are the previous job's or ours, and no watermark runs past our endpoint's
 * predecessor: anything else is a protocol error (another writer, or the wrong channel).
 */
static inline bool g5p_cu_stamp_sane(const u8 *p, u32 seq)
{
	u32 start = g5p_le32(p + G5P_CU_START), end = g5p_le32(p + G5P_CU_END);

	return (start == seq - 1 || start == seq) &&
	       (end == seq - 1 || end == seq);
}

#endif

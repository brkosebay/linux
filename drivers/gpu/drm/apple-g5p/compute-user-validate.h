/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_COMPUTE_USER_VALIDATE_H
#define G5P_COMPUTE_USER_VALIDATE_H

/*
 * TYPED_COMPUTE pass checks over already-resolved resource
 * ranges, as render-user-validate.h does for TYPED_V3.  Pure, host-tested
 * (scripts/gpu/compute-host/test_compute.c).  Nothing here is a pointer the kernel
 * dereferences; the stream's last word is read by the caller through the BO's own pages
 * (g5p_cu_content()).  Like TYPED_V3 the kernel does not decode the CDM records, the PDS
 * programs or the USC code they name; it bounds every address the firmware or the CDM
 * starts from and leaves the rest to the per-context VM and fault isolation.
 *   -EINVAL  flags, alignment, sizes, a non-zero terminate control word, no terminator
 *   -ERANGE  a PDS reference outside [pds_base, pds_base + code_window), or a range past
 *            hw_va_limit
 *   -ENOENT  no listed range wholly covers the stream, a state word or a PDS reference
 *   -EACCES  the range lacks READ (stream, PDS data), WRITE (state) or EXEC (PDS code)
 */
#define G5P_CU_MAX_STREAM 0x10000U
#define G5P_CU_STREAM_MIN 4U
#define G5P_CU_TERMINATE 0x80000000U

/* The pass as the UAPI carries it, kernel copy. */
struct g5p_cu_pass {
	u64 cdm_va;
	u32 cdm_size, cdm_end;
	u64 state[4];
	u64 tail;
	u32 config[2];
	u32 store[3], resume[3], term[3]; /* code, data, control */
};

/* Where the content rule reads: the range holding the stream. */
struct g5p_cu_loc {
	u32 stream_range;
};

/* One {code, data} PDS pair: code in a READ|EXEC range, data in a READ range (g5p_ru_v3_ref()). */
static inline int g5p_cu_pds(const struct g5p_ru_v3_limits *lim,
			     const struct g5p_ru_range *r, u32 n,
			     const u32 *pds)
{
	int ret = g5p_ru_v3_ref(lim, r, n, pds[0], true);

	return ret ?: g5p_ru_v3_ref(lim, r, n, pds[1], false);
}

static inline int g5p_cu_pass_check(const struct g5p_cu_pass *p,
				    const struct g5p_ru_v3_limits *lim,
				    const struct g5p_ru_range *r, u32 n,
				    struct g5p_cu_loc *loc)
{
	const u32 rw = G5P_RU_ACCESS_READ | G5P_RU_ACCESS_WRITE;
	u32 si;
	int ret;

	if (p->cdm_size < G5P_CU_STREAM_MIN ||
	    p->cdm_size > G5P_CU_MAX_STREAM ||
	    ((p->cdm_va | p->cdm_size | p->cdm_end) & 3) ||
	    p->cdm_end < G5P_CU_STREAM_MIN || p->cdm_end > p->cdm_size)
		return -EINVAL;
	if ((p->tail & 0xf) || p->term[2])
		return -EINVAL;
	for (u32 i = 0; i < 4; i++)
		if (p->state[i] & 3)
			return -EINVAL;
	/* Every listed range, as TYPED_V3: the CDM and its tasks read and write past them. */
	for (u32 i = 0; i < n; i++)
		if (!g5p_ru_below(lim->hw_va_limit, r[i].va, r[i].size))
			return -ERANGE;
	if (!g5p_ru_cover(r, n, p->cdm_va, p->cdm_size, &si))
		return -ENOENT;
	if (!(r[si].access & G5P_RU_ACCESS_READ))
		return -EACCES;
	for (u32 i = 0; i < 5; i++) {
		u64 va = i < 4 ? p->state[i] : p->tail;
		u32 k;

		if (!g5p_ru_cover(r, n, va, i < 4 ? 4 : 16, &k))
			return -ENOENT;
		if ((r[k].access & rw) != rw)
			return -EACCES;
	}
	ret = g5p_cu_pds(lim, r, n, p->store);
	if (!ret)
		ret = g5p_cu_pds(lim, r, n, p->resume);
	if (!ret)
		ret = g5p_cu_pds(lim, r, n, p->term);
	if (ret)
		return ret;
	loc->stream_range = si;
	return 0;
}

/* The last stream dword, at cdm_va + cdm_end - 4, is the CDM terminate word. */
static inline int g5p_cu_content(const u8 *tail)
{
	u32 word = tail[0] | tail[1] << 8 | tail[2] << 16 | (u32)tail[3] << 24;

	return word == G5P_CU_TERMINATE ? 0 : -EINVAL;
}

/* The firmware's register images of a checked pass. */
static inline void g5p_cu_regs_of(struct g5p_cu_regs *k,
				  const struct g5p_cu_pass *p)
{
	k->stream = p->cdm_va;
	k->terminate = p->cdm_va + p->cdm_end - 4;
	for (u32 i = 0; i < 4; i++)
		k->state[i] = p->state[i];
	k->tail = p->tail;
	k->config[0] = p->config[0];
	k->config[1] = p->config[1];
	k->store = p->store[0] | (u64)p->store[1] << 32;
	k->resume = p->resume[0] | (u64)p->resume[1] << 32;
	k->term = p->term[0] | (u64)p->term[1] << 32;
	k->store1 = p->store[2];
	k->resume1 = p->resume[2];
}

/*
 * Submit-time admission of a compute job, on top of g5p_ru_v3_admit(): the CDM rides an epoch
 * a render job brought up (the attach rings the TA bell; a compute job never does), and its
 * own sequence is bounded like the render one.
 */
enum g5p_cu_refusal {
	G5P_CU_OK,
	G5P_CU_NO_EPOCH, /* no render job has brought the device up in this epoch */
	G5P_CU_OVERFLOW, /* the CDM sequence is exhausted */
};

static inline int g5p_cu_admit(bool device_up, u32 cu_sequence)
{
	if (!device_up)
		return G5P_CU_NO_EPOCH;
	if (cu_sequence >= G5P_CU_SEQ_MAX)
		return G5P_CU_OVERFLOW;
	return G5P_CU_OK;
}

#endif

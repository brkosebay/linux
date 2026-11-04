/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RENDER_USER_GEOMETRY_H
#define G5P_RENDER_USER_GEOMETRY_H

/* Kernel-derived render-target geometry: AGXRenderTargetMemoryG5's arithmetic
 * leaf 6ca6d5c (firmware round16 section 3) for one sample at the pixel
 * centre (8,8) or four samples (docs/gpu/experiments/msaa.md), in integer
 * arithmetic only. It is Rogue's non-simple
 * parameter format: 32x32 tiles, 4x4 macrotiles of
 * ALIGN(DIV_ROUND_UP(tiles, 4), 4) tiles per axis, five-byte region headers
 * per padded tile, a tail pointer cache of next_pow2(max(padded tiles))^2
 * eight-byte entries. The scale pairs are the leaf's IEEE singles
 * 0x3e89309c / n, 0x3fddb3d9 / n (fdiv, round to nearest even) and n * 16.
 * tools/gpu/test_rt_oracle.c compares every output with the unmodified leaf
 * for every width and height in 1..G5P_RU_MAX_DIM.
 *
 * G5P_RU_MAX_DIM is rogue_get_render_size_max() for this tiling class; it
 * is a memory bound (16 MiB private window, 8 MiB main ring), not a field
 * limit: the 9-bit TE macrotile and 10-bit padded-size fields would hold
 * twice as much.
 */
#define G5P_RU_MAX_DIM 8192U
#define G5P_RU_SCALE_A 0x3e89309cU
#define G5P_RU_SCALE_B 0x3fddb3d9U

static inline u32 g5p_ru_fls(u32 v)
{
	return v ? 32 - __builtin_clz(v) : 0;
}

/* ucvtf of 1 <= n < 2^24: exact. */
static inline u32 g5p_ru_f32(u32 n)
{
	u32 bits = g5p_ru_fls(n);

	return ((126 + bits) << 23) | ((n << (24 - bits)) & 0x7fffff);
}

/* fdiv of a normal positive single by 1 <= n < 2^24, round to nearest even.
 * The quotient q of the 24-bit significand shifted by fls(n) + 3 has 27 or
 * 28 bits; s of them are rounding bits, the remainder is the sticky bit.
 * Results stay normal for the two leaf constants and n <= G5P_RU_MAX_DIM.
 */
static inline u32 g5p_ru_fdiv(u32 a, u32 n)
{
	u32 shift = g5p_ru_fls(n) + 3;
	u64 num = (u64)((a & 0x7fffff) | 0x800000) << shift;
	u32 q = num / n, sticky = num % n != 0;
	u32 s = g5p_ru_fls(q) - 24, m = q >> s, low = q & ((1U << s) - 1);

	if (low > 1U << (s - 1) ||
	    (low == 1U << (s - 1) && (sticky || (m & 1))))
		m++;
	if (m >> 24) {
		m >>= 1;
		s++;
	}
	return ((((a >> 23) & 0xff) - shift + s) << 23) | (m & 0x7fffff);
}

/* Sample positions, one byte per sample: x in [3:0], y in [7:4], 1/16 px.
 * One sample sits at the pixel centre; four use the standard Vulkan/D3D
 * pattern, which is also Metal's (record+0x69c..0x6b8 of the iOS
 * msaa4-resolve/-store captures: (6,2) (14,6) (2,10) (10,14)).
 */
#define G5P_RU_POSITIONS_1X 0x88ULL
#define G5P_RU_POSITIONS_4X 0xeaa26e26ULL

/* The leaf for one or four samples (6ca6d5c's 1, 2, 4, 8 table, of which
 * Linux uses 1 and 4). Four samples are 2x2 per pixel: the region headers
 * and the tail pointer cache are sized for twice the tiles in x and y, the
 * ISP macrotile size (Rbc) doubles in y, Rb8 is log2(samples) (5940f40)
 * and the TE word Rf0 is 5. Rb0 and Rf8 are the positions word, packed by
 * the leaf as x0 | y0 << 4 | x1 << 8 | ... | y3 << 28. The scales, the TE
 * tiling words and the tile count do not depend on the samples.
 */
static inline int g5p_ru_rt_derive_samples(u32 width, u32 height, u32 samples,
					   u64 positions, struct g5p_ru_rt *rt)
{
	u32 tx = (width + 31) / 32, ty = (height + 31) / 32;
	u32 mx = (((tx + 3) >> 2) + 3) & ~3U, my = (((ty + 3) >> 2) + 3) & ~3U;
	u32 s = samples == 4 ? 2 : 1;
	u32 px = mx * s * 4, py = my * s * 4, tpc;

	if (!width || !height || width > G5P_RU_MAX_DIM ||
	    height > G5P_RU_MAX_DIM || (samples != 1 && samples != 4) ||
	    positions >> (samples * 8))
		return -EINVAL;
	tpc = 1U << g5p_ru_fls((px > py ? px : py) - 1);
	*rt = (struct g5p_ru_rt) {
		.header = px * py * 5,
		.region = mx * my * s * s * 5,
		.packed = positions,
		.state = s - 1 ? 2 : 0,
		.padded = (mx & 0x3ff) << 16 | ((my * s) & 0x3ff),
		.scale = {
			g5p_ru_fdiv(G5P_RU_SCALE_A, width) |
				(u64)g5p_ru_fdiv(G5P_RU_SCALE_A, height) << 32,
			g5p_ru_fdiv(G5P_RU_SCALE_B, width) |
				(u64)g5p_ru_fdiv(G5P_RU_SCALE_B, height) << 32,
			(g5p_ru_f32(width) + (4U << 23)) |
				(u64)(g5p_ru_f32(height) + (4U << 23)) << 32,
		},
		.ta_state = { s - 1 ? 5 : 0, positions },
		.ta_tiling = {
			(u64)((ty - 1) << 12 | (tx - 1)) << 32 |
				((height - 1) << 16 | (width - 1)),
			(u64)(my << 18 | my << 10 | 3 * my) << 32 |
				(mx << 18 | mx << 10 | 3 * mx),
		},
		.ta_tiling_word = mx * my,
		.tiles = tx * ty,
		.secondary = (u64)tpc * tpc * 8,
		.secondary_align = 0x40,
	};
	return 0;
}

/* g5p_ru_rt64 is this function's result for 64x64. */

/* PM pool for a kernel-derived target. Linux policy, not hardware: Rogue's
 * minimum free list (50 4KiB pages, non-XE cores) plus one 4KiB PM page per
 * tile, in 128KiB blocks, never below the latched pool and at most one F
 * page. 64x64 (4 tiles) needs 2, so the latched 32 stays. There is no PM
 * growth during a job: a frame that exhausts the pool fails like any OOM.
 */
static inline u32 g5p_ru_pm_blocks_for(u32 tiles, u32 latched)
{
	u64 need = ((50ULL + tiles) * 0x1000 + G5P_RU_BLOCK_BYTES - 1) /
		   G5P_RU_BLOCK_BYTES;

	if (need < latched)
		need = latched;
	return need > G5P_RU_MAX_BLOCKS ? G5P_RU_MAX_BLOCKS : need;
}



static inline int g5p_ru_rt_derive(u32 width, u32 height, struct g5p_ru_rt *rt)
{
	return g5p_ru_rt_derive_samples(width, height, 1, G5P_RU_POSITIONS_1X, rt);
}

#endif

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RENDER_VARIANTS_R25_H
#define G5P_RENDER_VARIANTS_R25_H

/* Finite, owned-input experiments. No raw W, register or firmware-patch API. */
#define G5P_R25_MAX_BLOCKS 128U
#define G5P_R25_HEAD 0x3000U

struct g5p_r25_variants {
	u32 pm, blocks, context, host, sku, memory, rt, slots;
};

static inline bool g5p_r25_valid(const struct g5p_r25_variants *v)
{
	return v->pm <= 3 &&
	       (v->blocks == 0 || v->blocks == 64 || v->blocks == 128) &&
	       v->context <= 3 && v->host <= 2 && v->sku <= 3 &&
	       v->memory <= 4 && v->rt <= 2 && v->slots <= 1;
}

/* Event synthesis, removed context tasks, candidate clocks and the low-address
 * dialect test cannot qualify rendering, even if firmware stamps retire.
 */
static inline bool g5p_r25_diagnostic(const struct g5p_r25_variants *v)
{
	return v->sku || v->context || v->host || v->rt == 2;
}

/* PM payload frames are physical PPNs and have no user PTE attributes. This
 * changes only PM auxiliary mappings (and, in mode4, other private mappings).
 * Firmware/shared META mappings and the borrowed sink are never rewritten.
 */
static inline u32 g5p_r25_attributes(const struct g5p_r25_variants *v, u64 va)
{
	if (!v->memory)
		return 0xc;
	if (v->memory != 4 &&
	    (va < G5P_R22_AUX || va >= G5P_R22_AUX + 5 * G5P_GPU_PAGE))
		return 0xc;
	return v->memory == 2 ? 4 : v->memory == 3 ? 8 : 0;
}

/* Preserve the original constructor exactly for the default pool. The extra
 * blocks fit the existing 16KiB list and free allocations, including F's last
 * dword at128 blocks. Validate every PA before mutating either allocation.
 */

#endif

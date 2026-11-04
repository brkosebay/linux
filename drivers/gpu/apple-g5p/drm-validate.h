/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_DRM_VALIDATE_H
#define G5P_DRM_VALIDATE_H

#include "bits-compat.h"

#define G5P_USER_VA_START 0x10000ULL
#define G5P_USER_VA_END 0x10000000000ULL
#define G5P_USER_RESERVED_START G5P_IMAGE_VA
#define G5P_USER_RESERVED_END (G5P_PIO_VA + G5P_PIO_SIZE)
#define G5P_MAX_BO_SIZE (64ULL << 20)
/* VA bookkeeping: the sum of mapping sizes in one VM.  A GEM bound twice
 * (GBM alloc, then the EGL import of its own dma-buf) counts twice here.
 */
#define G5P_MAX_VM_BYTES BIT_ULL(30)
/* Memory behind one VM's mappings, counted once per GEM object. */
#define G5P_MAX_PINNED_BYTES (768ULL << 20)

static inline bool g5p_user_pa_valid(u64 pa)
{
	return pa < 0x10000000000ULL && !(pa & (G5P_GPU_PAGE - 1));
}

static inline bool g5p_user_range_valid(u64 va, u64 size)
{
	if (!size || ((va | size) & (G5P_GPU_PAGE - 1)) ||
	    va < G5P_USER_VA_START || va >= G5P_USER_VA_END ||
	    size > G5P_USER_VA_END - va)
		return false;
	return va >= G5P_USER_RESERVED_END ||
	       va + size <= G5P_USER_RESERVED_START;
}

static inline bool g5p_bo_range_valid(u64 offset, u64 size, u64 bo_size)
{
	return size && !((offset | size) & (G5P_GPU_PAGE - 1)) &&
	       offset < bo_size && size <= bo_size - offset;
}

#endif

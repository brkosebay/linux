/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * The firmware window of a render VM (docs/gpu/multi-client-design.md §6 R4, stage 3).
 *
 * The firmware's own GPU memory lives in one GiB, catalogue entry G5P_RU_FIRMWARE_CI: its
 * image, the root and shared pages (rings, the PA/tag arrays, W), the MMU table alias and the
 * PIO pages (Fender, AIC).  Stage 1 copied that catalogue entry into every render VM, so any
 * render job could address all of it.  A render VM now gets, under that entry, a directory
 * the kernel built that maps only the pages the GPU's own requesters reach through the
 * context's slot: the PM control page (S) and the PM list and free list (F).  Their leaf
 * entries are copied from the firmware's tables, so their translation is unchanged.
 *
 *   render_user_fw_window = 0 (G5P_RU_FW_FULL)    the stage 1 mapping: the whole firmware GiB;
 *                                                  render contexts need CAP_SYS_RAWIO
 *                          1 (G5P_RU_FW_WINDOW)  the window above (default)
 *                          2 (G5P_RU_FW_NONE)    no firmware page at all (an experiment)
 *
 * Pure helpers, host-tested (scripts/gpu/multiclient2-host/test_render_window.c).  The
 * caller provides g5p_le32() and the boot layout constants.
 */
#ifndef G5P_RENDER_USER_WINDOW_H
#define G5P_RENDER_USER_WINDOW_H

#define G5P_RU_FW_FULL 0U
#define G5P_RU_FW_WINDOW 1U
#define G5P_RU_FW_NONE 2U
#define G5P_RU_FW_MODES 3U

#define G5P_RU_FW_PAGES 3U
/* The pages of the window, all in the firmware GiB. */
static const u64 g5p_ru_fw_window_va[G5P_RU_FW_PAGES] = {
	G5P_SHARED_VA + G5P_R22_PM, /* PM control and S */
	G5P_SHARED_VA + G5P_SHARED_SIZE, /* PM list */
	G5P_SHARED_VA + G5P_SHARED_SIZE + G5P_GPU_PAGE, /* PM free list F */
};

static inline u32 g5p_ru_fw_di(u64 va)
{
	return (va >> 21) & 511;
}

static inline u32 g5p_ru_fw_ti(u64 va)
{
	return (va >> 14) & 127;
}

/* The firmware's 64-bit leaf entry for `va`, from its leaf pages and their directory
 * indices; 0 when the firmware does not map it.
 */
static inline u64 g5p_ru_fw_leaf(u8 *const *leaf, const u32 *leaf_di,
				 u32 leaves, u64 va)
{
	for (u32 i = 0; i < leaves; i++) {
		const u8 *e = leaf[i] + g5p_ru_fw_ti(va) * 8;

		if (leaf_di[i] == g5p_ru_fw_di(va))
			return (u64)g5p_le32(e + 4) << 32 | g5p_le32(e);
	}
	return 0;
}

/* The window's leaf page index for `va` among `n` distinct directory indices so far, or n
 * (a new one) when none has its directory index.
 */
static inline u32 g5p_ru_fw_slot(const u32 *di, u32 n, u64 va)
{
	u32 i;

	for (i = 0; i < n && di[i] != g5p_ru_fw_di(va); i++)
		;
	return i;
}

#endif

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Latency114: one RAM dword, meta+0x71c, in the bootstrap image.
 *
 * meta (object 0x4d) +0x718 is the firmware's pstate frequency table in MHz.
 * Its init copies the table to private+0x26c at boot (MINIM 0x1000122c..127c),
 * and the scheduler loop reloads its timer from entry 1 on both branches
 * (0x8000a1a4 via private+0x2c4 at pstate 1, 0x8000a338 via private+0x270).
 * Linux leaves the table zero; Apple has { 0, 340, 474, 550, 723 }.  The A/B
 * installs entry 1 only: 0 is the control (nothing is stored, variant 7 leaves
 * the word zero), 340 the factor.  Everything else is refused at map, before
 * any store.  The store goes into the staging buffer, before g5p_boot_copy()
 * publishes the image and long before the firmware is released.  There is no
 * register access here.
 */
#ifndef APPLE_G5P_META71C114_H
#define APPLE_G5P_META71C114_H

#define G5P_M71C_SLOT 0x4dU
#define G5P_M71C_OFFSET 0x71cU
#define G5P_M71C_CONTROL 0U
#define G5P_M71C_FACTOR 340U
#define G5P_M71C_VARIANT 7U

enum {
	G5P_M71C_OK,
	G5P_M71C_BAD_VALUE,
	G5P_M71C_BAD_VARIANT,
	G5P_M71C_WHYS,
};

static const char *const g5p_m71c_whys[G5P_M71C_WHYS] = {
	"none",
	"value",
	"variant",
};

/* Latched at map: only the two pre-registered arms, the factor on dc7 only. */
static inline u32 g5p_m71c_check(u32 requested, u32 variant)
{
	if (requested != G5P_M71C_CONTROL && requested != G5P_M71C_FACTOR)
		return G5P_M71C_BAD_VALUE;
	if (requested == G5P_M71C_FACTOR && variant != G5P_M71C_VARIANT)
		return G5P_M71C_BAD_VARIANT;
	return G5P_M71C_OK;
}

/*
 * One store into the staging image (the factor only) and its read-back.  The
 * caller fails the map unless the read-back equals the request.
 */
static inline u32 g5p_m71c_install(const struct g5p_boot_layout *p, u8 *root,
				   u8 *shared, u32 requested, bool *wrote)
{
	u8 *word = g5p_object(p, root, shared, G5P_M71C_SLOT) + G5P_M71C_OFFSET;

	*wrote = false;
	if (requested == G5P_M71C_FACTOR) {
		g5p_put32(word, requested);
		*wrote = true;
	}
	return g5p_le32(word);
}

#endif

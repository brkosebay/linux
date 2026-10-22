/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_G5P_PACK_H
#define APPLE_G5P_PACK_H

#include "ldr.h"
#ifndef G5P_LDR_USER_TEST
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#define G5P_IMAGE_SIZE 0x30000
#define G5P_CONFIG_WRITES 27

struct g5p_ldr_image {
	u8 *data;
	u32 config[G5P_CONFIG_WRITES][2];
	u32 config_count;
	u32 t1_pc;
	u32 t1_sp;
};

/* Exact C0 ranges derived from AGXMetaFirmware_layoutAndLoadLDR ...6c9f1bc.
 * GPU virtual mappings and the boot config at +0x200 are intentionally separate.
 */
static inline int g5p_image_offset(u32 addr, u32 size, u32 *offset)
{
	static const struct {
		u32 base, size, packed;
	} segments[] = {
		{ 0x10000000, 0x18000, 0x04000 },
		{ 0x20000000, 0x04000, 0x1c000 },
		{ 0xc0000000, 0x04000, 0x00000 },
		{ 0xe0001000, 0x04000, 0x20000 },
		{ 0x80000080, 0x055dc, 0x24000 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(segments); i++) {
		if (addr >= segments[i].base && size <= segments[i].size &&
		    addr - segments[i].base <= segments[i].size - size) {
			*offset = segments[i].packed + addr - segments[i].base;
			return 0;
		}
	}
	return -EINVAL;
}

static inline int g5p_pack_record(void *ctx, enum g5p_ldr_op op, u32 addr,
				  u32 size, const u8 *data)
{
	struct g5p_ldr_image *image = ctx;
	u32 i, offset;
	int ret;

	switch (op) {
	case G5P_LOAD:
		ret = g5p_image_offset(addr, size, &offset);
		if (ret)
			return ret;
		memcpy(image->data + offset, data, size);
		break;
	case G5P_ZERO:
		/* Apple does not apply the local/core-memory ZeroMem records here. */
		if (addr >= 0x80000000 && addr < 0x88000000)
			break;
		ret = g5p_image_offset(addr, size, &offset);
		if (ret)
			return ret;
		memset(image->data + offset, 0, size);
		break;
	case G5P_CONFIG:
		for (i = 0; i < size; i += 12) {
			if (image->config_count == G5P_CONFIG_WRITES)
				return -EINVAL;
			image->config[image->config_count][0] = g5p_le32(data + i + 4);
			image->config[image->config_count++][1] = g5p_le32(data + i + 8);
		}
		break;
	case G5P_START:
		image->t1_sp = g5p_le32(data + 4);
		image->t1_pc = g5p_le32(data + 8);
		break;
	}
	return 0;
}
#endif

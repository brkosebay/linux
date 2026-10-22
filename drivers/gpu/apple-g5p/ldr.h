/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Deliberately small Apple G5P META LDR reader. No allocation or MMIO here. */
#ifndef APPLE_G5P_LDR_H
#define APPLE_G5P_LDR_H

#ifndef G5P_LDR_USER_TEST
#include <linux/errno.h>
#include <linux/types.h>
#endif

#define G5P_LDR_MAX_RECORDS 128
#define G5P_LDR_MAX_BYTES (256 * 1024)

enum g5p_ldr_op { G5P_LOAD, G5P_ZERO, G5P_CONFIG, G5P_START };

typedef int (*g5p_ldr_emit)(void *ctx, enum g5p_ldr_op op, u32 address,
			   u32 size, const u8 *data);

static inline u32 g5p_le32(const u8 *p)
{
	return (u32)p[0] | (u32)p[1] << 8 | (u32)p[2] << 16 | (u32)p[3] << 24;
}

static inline u16 g5p_le16(const u8 *p)
{
	return (u16)p[0] | (u16)p[1] << 8;
}

static inline bool g5p_ldr_bounds(size_t len, u32 off, u32 size)
{
	return off <= len && size <= len - off;
}

static inline int g5p_ldr_l2(const u8 *fw, size_t len, u32 off, u16 tag,
			     const u8 **data, u32 *size)
{
	u16 bytes;

	if (!g5p_ldr_bounds(len, off, 4))
		return -EINVAL;
	bytes = g5p_le16(fw + off + 2);
	if (bytes < 6 || !g5p_ldr_bounds(len, off, bytes) ||
	    g5p_le16(fw + off) != tag)
		return -EINVAL;
	*data = fw + off + 4;
	*size = bytes - 6;
	return 0;
}

/* Validate in a first pass (emit=NULL); only then use a hardware callback. */
static inline int g5p_ldr_walk(const u8 *fw, size_t len,
			       g5p_ldr_emit emit, void *ctx)
{
	u32 pos, seen[G5P_LDR_MAX_RECORDS], total = 0;
	unsigned int nr = 0, starts = 0;

	if (len != 119112 || g5p_le32(fw) != 0x01aa5500 ||
	    !g5p_ldr_bounds(len, g5p_le32(fw + 4), 1))
		return -EINVAL;
	pos = g5p_le32(fw + 8);
	while (pos != 0xffffffff) {
		const u8 *p, *data;
		u32 command, bytes, addr = 0, size = 0, l2, next;
		enum g5p_ldr_op op;
		unsigned int i;
		int ret;

		if (nr == G5P_LDR_MAX_RECORDS || !g5p_ldr_bounds(len, pos, 12))
			return -EINVAL;
		for (i = 0; i < nr; i++)
			if (seen[i] == pos)
				return -EINVAL;
		seen[nr++] = pos;
		p = fw + pos;
		command = g5p_le16(p);
		bytes = g5p_le16(p + 2);
		next = g5p_le32(p + 4);
		if (bytes < 12 || (bytes & 3) || !g5p_ldr_bounds(len, pos, bytes))
			return -EINVAL;
		if (command & 0x10) {
			if (command != 0x10 || bytes != 16)
				return -EINVAL;
			ret = g5p_ldr_l2(fw, len, g5p_le32(p + 8), 0x8010, &data, &size);
			if (ret)
				return ret;
			goto next_record;
		}
		switch (command) {
		case 0:
			if (bytes != 20)
				return -EINVAL;
			op = G5P_LOAD;
			addr = g5p_le32(p + 8);
			l2 = g5p_le32(p + 12);
			ret = g5p_ldr_l2(fw, len, l2, 0x8000, &data, &size);
			if (ret)
				return ret;
			break;
		case 4:
			if (bytes != 24)
				return -EINVAL;
			op = G5P_ZERO;
			addr = g5p_le32(p + 8);
			size = g5p_le32(p + 12);
			ret = g5p_ldr_l2(fw, len, g5p_le32(p + 16), 0x8004, &data, &l2);
			if (ret || l2)
				return -EINVAL;
			break;
		case 5:
		case 3:
			if (bytes != 16)
				return -EINVAL;
			op = command == 5 ? G5P_CONFIG : G5P_START;
			ret = g5p_ldr_l2(fw, len, g5p_le32(p + 8),
					 0x8000 | command, &data, &size);
			if (ret || !size || size % (command == 5 ? 12 : 16))
				return -EINVAL;
			if (command == 5) {
				for (i = 0; i < size; i += 12)
					if (g5p_le32(data + i) != 2 ||
					    (g5p_le32(data + i + 4) & 3))
						return -EINVAL;
			} else {
				if (++starts != 1 || size != 32 || next != 0xffffffff ||
				    g5p_le32(data) != 1 || g5p_le32(data + 16) != 0)
					return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		if (op == G5P_LOAD || op == G5P_ZERO) {
			if ((addr & 3) || (size & 3) || !size ||
			    size > G5P_LDR_MAX_BYTES || addr > (u32)~0U - size ||
			    total > G5P_LDR_MAX_BYTES - size)
				return -EINVAL;
			total += size;
		}
		if (emit) {
			ret = emit(ctx, op, addr, size, data);
			if (ret)
				return ret;
		}
next_record:
		pos = next;
	}
	return nr == 94 && starts == 1 ? 0 : -EINVAL;
}
#endif

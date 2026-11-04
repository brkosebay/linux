/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_BITS_COMPAT_H
#define G5P_BITS_COMPAT_H

#ifdef __KERNEL__
#include <linux/bits.h>
#include <linux/kernel.h>
#else
#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif
#ifndef BIT_ULL
#define BIT_ULL(nr) (1ULL << (nr))
#endif
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif
#endif

#endif

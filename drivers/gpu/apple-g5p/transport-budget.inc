/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#include "vm-mutation.h"
static u32 g5p_transport_event_limit(void)
{
	return 256 + 131072 * 4;
}

/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_R87B_ATTACH_H
#define G5P_R87B_ATTACH_H
/* R87b DIAGNOSTIC ONLY, never production-safe: the DC3 attach policy selector.
 * (docs/gpu/experiments/87b-build-report.md)
 *
 * Two campaigns (r85 stage 2, r87 stage 1) never reached GPU work: the DC3 attach pair was
 * published, the immediate gate (R1+0x1000 & 0x1f == 1) read 0x10/0x11, the host GPC off/on
 * repair timed out (-110) and attach stopped with -115 before the RAM-only consume wait ran,
 * after which the machine froze.  Every retained instance of that failure shows the firmware
 * consuming the pair on its own (dc=3/3 at export, no bell rung).  boot_r87b_attach selects:
 *   0  the original attach policy, exactly r87: a closed gate is repaired by the host (R1+0x20000
 *      clear, GPC off/on, ALIVE-owned doorbell).  The matched control.
 *   1  self-drain (the r82b design, docs/gpu/experiments/82-attach-reactivation-triage.md): a
 *      closed DC3 gate is only a phase sample of the firmware's GPC window.  The host does no
 *      repair and rings no bell; it keeps the RAM-only 3 s consume wait of render-r74f.inc and
 *      continues through the firmware-MMU attach path (r74g) when the firmware consumed the pair.
 *      An active gate still rings the bell at once (the unchanged 80 % path).  The TA kind is
 *      never affected.
 * The selector is latched at map and read only through the latched copy.  A non-zero value needs
 * the r80 epoch (which implies r74h, r74g and r74f case 1): self-drain is only safe because r74g
 * has the firmware own the first MMU pass, so no host R0 access is licensed by a rung bell.
 * The TYPED_V3 production path latches policy 1 with boot_r74f_case = boot_r74g_case = 1 and NO
 * r80 / r74h (boot-r73b.inc, "production path"); g5p_r87b_attach_valid() describes the r80
 * epoch only.
 */
#include <linux/errno.h>
#include <linux/types.h>

#define G5P_R87B_ATTACH_ORIGINAL 0U
#define G5P_R87B_ATTACH_SELF_DRAIN 1U
#define G5P_R87B_ATTACH_MAX 1U

#endif

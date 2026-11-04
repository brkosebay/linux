/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_WORK_LAYOUT_H
#define G5P_WORK_LAYOUT_H

/* Experimental first CL queue probe: c89420/c88840/c8a808/c980a0.
 * Tag 4 is emitted for dependencies by c8a808. Treating an already satisfied
 * dependency as a NOP is inferred and must be validated after G2.
 */
#define G5P_WORK_OFFSET 0x54000U
#define G5P_WORK_BYTES 0x400U
#define G5P_WORK_CTL 0x100U
#define G5P_WORK_RING 0x200U
#define G5P_WORK_CMD 0x300U
#define G5P_WORK_STAMP 0x340U
#define G5P_WORK_SLOTS 16U

/* Isolated visibility hypothesis, not the recovered normal host alias. */

/* Firmware10004e94 copies notification word2 into ctl+30. */

#endif

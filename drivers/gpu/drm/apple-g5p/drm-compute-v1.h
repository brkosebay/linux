/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* TYPED_COMPUTE (drm-compute-v1.inc): what the TYPED_V3 executor, the scheduler's drain and the
 * render device's bring-up call before the file is included.
 */
#ifndef G5P_DRM_COMPUTE_V1_H
#define G5P_DRM_COMPUTE_V1_H

struct g5p_drm;
struct g5p_render;
struct g5p_rjob;

static int g5p_cu_execute(struct g5p_drm *d, struct g5p_render *r,
			  struct g5p_rjob *job);
static bool g5p_cu_drain(struct g5p_drm *d);
static void g5p_cu_device_init(struct g5p_drm *d);

#endif

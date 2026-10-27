// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM/KMS driver for the Apple A9 (S8000/S8003) display pipe, disp0.
 *
 * iBoot leaves the eDP link, the panel and the timing generator running.
 * This driver adopts that 1536x2048 mode and never programs timings,
 * clocks, power or the link. It offers one XRGB8888 primary plane; a new
 * framebuffer reaches the pipe through the ADP parameter FIFO, which holds
 * the packet until the next frame boundary and applies it there. Interrupt
 * 0 carries both events used here: VBI (vblank) and 0x400 (packet applied),
 * which completes the page flip, so a flip event never arrives before the
 * new buffer is being scanned out. The packet's token is the next frame
 * number, because the pipe loads it into its frame counter.
 * Disabling the CRTC scans out a black buffer (the former boot framebuffer)
 * instead of clearing the layer enable or touching the link, which the
 * reverse engineering has not verified.
 *
 * Built in, it replaces simpledrm during boot; apple_adbe_drm.enable=0 on
 * the kernel command line keeps simpledrm instead.
 *
 * The pipe bus is 32 bits wide with DRAM at bus 0 (bus = CPU - offset, the
 * offset taken from the live scanout). Buffers are ordinary DMA allocations,
 * so probe requires the display DART to pass the scanout through without
 * translation; it checks this by reading the DART, never by writing it.
 *
 * The driver holds no framebuffer or GEM references of its own: the atomic
 * helpers own every buffer's lifetime. debugfs (adbe_state, adbe_bos) still
 * tracks each GEM object, its CPU mappings and whether it sits in CMA, so
 * that a soak shows a buffer leak, and who holds it, directly.
 */
#include <linux/aperture.h>
#include <linux/cma.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/pid.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/vmstat.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_auth.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_dumb_buffers.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_file.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "adbe_hw.h"

#define ADBE_WIDTH		1536
#define ADBE_HEIGHT		2048
#define ADBE_PITCH		(ADBE_WIDTH * 4)
#define ADBE_KICK_WAIT_MS	20	/* pre-kick gate; iOS allows 1000 */
#define ADBE_FLIP_VBLANKS	3	/* complete on VBI if 0x400 never comes */
#define ADBE_FLIP_TIMEOUT_MS	100	/* complete anyway if no interrupt */
#define ADBE_PARK_WAIT_MS	100
#define ADBE_LAT_BUCKETS	4	/* flip latency 0, 1, 2, 3+ frames */
/* Interrupt sources while vblank is on (added to iBoot's mask). */
#define ADBE_INTEN_VBLANK	(ADP_ST_VBI | ADP_ST_APPLIED)
/* The CMA area accessors are not exported: only a built-in driver can tell. */
#define ADBE_CMA_KNOWN		(IS_BUILTIN(CONFIG_DRM_APPLE_ADBE) && IS_ENABLED(CONFIG_DMA_CMA))

/* Kill switch: apple_adbe_drm.enable=0 leaves the display to simpledrm. */
static bool enable = true;
module_param(enable, bool, 0400);
MODULE_PARM_DESC(enable, "Drive disp0 (default); 0 keeps simpledrm on the boot framebuffer");

/* Writable; read once per bind, so a rebind picks up new values. */
static char *commit = "base";
module_param(commit, charp, 0600);
MODULE_PARM_DESC(commit, "Plane commit: base (packet with base[/stride]), full (whole layer set), sync (direct store + empty packet)");

static char *token = "next";
module_param(token, charp, 0600);
MODULE_PARM_DESC(token, "Preamble token (loaded into the frame counter): next (counter + 1), same, serial");

enum adbe_bo_kind {
	ADBE_BO_DUMB,		/* allocated here: scanout or fbdev buffer */
	ADBE_BO_IMPORT,		/* PRIME import, another device's memory */
	ADBE_BO_KINDS,
};

static const char * const adbe_bo_kind_names[ADBE_BO_KINDS] = {
	[ADBE_BO_DUMB] = "dumb",
	[ADBE_BO_IMPORT] = "import",
};

/* A GEM object with debugfs bookkeeping. */
struct adbe_bo {
	struct drm_gem_dma_object dma;	/* first: the helpers kfree() it */
	struct list_head node;		/* adbe->bos once accounted, else empty */
	enum adbe_bo_kind kind;
	bool in_cma;
	atomic_t mmaps;			/* live CPU mappings (VMAs) */
	ktime_t born;
	pid_t pid;			/* creator */
	char comm[TASK_COMM_LEN];
};

static_assert(offsetof(struct adbe_bo, dma) == 0);

struct adbe {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	void __iomem *adp;
	void __iomem *boot_va;
	struct resource boot;
	struct device_node *boot_np;	/* the simple-framebuffer node */
	u64 bus_offset;
	u32 boot_base;
	u32 boot_stride;
	u32 saved_inten;
	bool vbl_enabled;
	enum adbe_commit commit;
	enum adbe_token token;
	int irq;
	/* Interrupt handler only. */
	u16 last_frame;
	bool frame_valid;
	/* Staged by the plane update, pushed by the CRTC flush. */
	u32 next_base;
	u32 next_stride;
	bool next_dirty;
	/* Commit-tail only (serialized by the atomic helpers). */
	u32 cur_base;
	bool boot_black;
	u16 serial;
	/* Protected by drm.event_lock. */
	struct drm_pending_vblank_event *event;
	unsigned int event_vbls;
	bool event_applied;	/* 0x400 seen for the pending flip */
	u32 pending_base;
	u64 kick_seq;		/* vblank count when the flip was pushed */
	ktime_t kick_time;
	int kick_done;		/* completions counted before the push */
	struct timer_list flip_timer;
	/*
	 * Statistics for debugfs. Every status read that shows a completion
	 * bit counts once, wherever it happens, so that kicks equals
	 * completions plus the packet still in the FIFO.
	 */
	atomic_t kicks;
	atomic_t completions;
	atomic_t applied;	/* reads with 0x400 */
	atomic_t synced;	/* reads with 0x8 (iOS's bit, never seen yet) */
	atomic_t busy;
	atomic_t vbl_fallbacks;
	atomic_t timeouts;
	atomic_t latch_mismatch;	/* base readback != pushed at completion */
	atomic_t frame_jumps;		/* counter moved by != 1 between VBIs */
	atomic_t errors;
	atomic_t lat_frames[ADBE_LAT_BUCKETS];	/* push to completion */
	u32 lat_us_max;
	u64 lat_us_sum;
	u32 lat_count;
	atomic_t vbis;		/* VBI interrupts handled while vblank is on */
	atomic_t vbl_events;	/* flip events armed with nothing to push */
	/* Live GEM objects, protected by bo_lock. */
	struct mutex bo_lock;
	struct list_head bos;
	unsigned int bo_count[ADBE_BO_KINDS];
	u64 bo_bytes[ADBE_BO_KINDS];
	u64 bo_cma_bytes[ADBE_BO_KINDS];
	u64 bo_dumb_peak;
};

static struct adbe *to_adbe(struct drm_device *drm)
{
	return container_of(drm, struct adbe, drm);
}

static struct adbe_bo *to_adbe_bo(struct drm_gem_object *obj)
{
	return container_of(obj, struct adbe_bo, dma.base);
}

static const struct drm_display_mode adbe_mode = {
	DRM_MODE_INIT(60, ADBE_WIDTH, ADBE_HEIGHT, 148, 197),
};

/* Caller holds event_lock. Returns the event to complete, if any. */
static struct drm_pending_vblank_event *adbe_take_event(struct adbe *adbe)
{
	struct drm_pending_vblank_event *event = adbe->event;

	adbe->event = NULL;
	return event;
}

static void adbe_send_event(struct adbe *adbe, struct drm_pending_vblank_event *event)
{
	unsigned long flags;

	if (!event)
		return;
	spin_lock_irqsave(&adbe->drm.event_lock, flags);
	drm_crtc_send_vblank_event(&adbe->crtc, event);
	spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	drm_crtc_vblank_put(&adbe->crtc);
}

/* Account one status read's completion bits (any context). */
static void adbe_count_done(struct adbe *adbe, u32 status)
{
	if (!(status & ADP_ST_DONE))
		return;
	atomic_inc(&adbe->completions);
	if (status & ADP_ST_APPLIED)
		atomic_inc(&adbe->applied);
	if (status & ADP_ST_SYNCED)
		atomic_inc(&adbe->synced);
}

/* Push-to-completion latency of one flip, after the vblank was counted. */
static void adbe_count_latency(struct adbe *adbe, u64 seq, ktime_t start)
{
	u64 frames = drm_crtc_vblank_count(&adbe->crtc) - seq;
	u32 us = ktime_us_delta(ktime_get(), start);

	atomic_inc(&adbe->lat_frames[min_t(u64, frames, ADBE_LAT_BUCKETS - 1)]);
	WRITE_ONCE(adbe->lat_us_max, max(adbe->lat_us_max, us));
	WRITE_ONCE(adbe->lat_us_sum, adbe->lat_us_sum + us);
	WRITE_ONCE(adbe->lat_count, adbe->lat_count + 1);
}

static irqreturn_t adbe_irq(int irq, void *data)
{
	struct adbe *adbe = data;
	struct drm_pending_vblank_event *done = NULL;
	bool applied = false;
	ktime_t start = 0;
	u64 seq = 0;
	u32 status;

	/*
	 * Read and acknowledge under event_lock so a flip armed by
	 * adbe_kick() never completes on a status read before its push.
	 */
	spin_lock(&adbe->drm.event_lock);
	status = readl(adbe->adp + ADP_STATUS);
	if (status)
		writel(status, adbe->adp + ADP_STATUS);	/* W1C, level line */
	adbe_count_done(adbe, status);
	if (adbe->event) {
		if (status & ADP_ST_DONE)
			adbe->event_applied = true;
		/*
		 * Complete once the boundary's vblank is counted (VBI here, or
		 * already handled), so the event carries that frame's sequence
		 * and timestamp even if 0x400 interrupts a moment before VBI.
		 */
		if (adbe->event_applied &&
		    ((status & ADP_ST_VBI) ||
		     drm_crtc_vblank_count(&adbe->crtc) != adbe->kick_seq)) {
			done = adbe_take_event(adbe);
			applied = true;
			seq = adbe->kick_seq;
			start = adbe->kick_time;
			/* Camera-free cross-check: the plane holds the pushed base. */
			if (readl(adbe->adp + ADP_BASE) != adbe->pending_base)
				atomic_inc(&adbe->latch_mismatch);
		} else if (!adbe->event_applied && (status & ADP_ST_VBI) &&
			   ++adbe->event_vbls >= ADBE_FLIP_VBLANKS) {
			done = adbe_take_event(adbe);
			atomic_inc(&adbe->vbl_fallbacks);
		}
	}
	spin_unlock(&adbe->drm.event_lock);
	if (!status)
		return IRQ_NONE;
	if (status & ADP_ST_ERRORS) {
		atomic_or(status & ADP_ST_ERRORS, &adbe->errors);
		dev_warn_ratelimited(adbe->drm.dev, "pipe error bits %#x\n",
				     status & ADP_ST_ERRORS);
	}
	/* VBI is sticky: only count it while it is enabled (not on OutUnder). */
	if ((status & ADP_ST_VBI) && READ_ONCE(adbe->vbl_enabled)) {
		u16 frame = readl(adbe->adp + ADP_FRAME);

		/* A reloaded counter (or a missed VBI) shows up here. */
		if (adbe->frame_valid && adbe_frame_jump(adbe->last_frame, frame))
			atomic_inc(&adbe->frame_jumps);
		adbe->last_frame = frame;
		adbe->frame_valid = true;
		atomic_inc(&adbe->vbis);
		drm_crtc_handle_vblank(&adbe->crtc);
	}
	/* The event carries this boundary's vblank sequence and timestamp. */
	if (applied)
		adbe_count_latency(adbe, seq, start);
	adbe_send_event(adbe, done);
	return IRQ_HANDLED;
}

/* Safety net: never leave a flip pending if no interrupt completes it. */
static void adbe_flip_timeout(struct timer_list *t)
{
	struct adbe *adbe = timer_container_of(adbe, t, flip_timer);
	struct drm_pending_vblank_event *done;
	unsigned long flags;

	spin_lock_irqsave(&adbe->drm.event_lock, flags);
	done = adbe_take_event(adbe);
	spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	if (done) {
		atomic_inc(&adbe->timeouts);
		dev_warn_ratelimited(adbe->drm.dev, "flip not acknowledged in %u ms\n",
				     ADBE_FLIP_TIMEOUT_MS);
	}
	adbe_send_event(adbe, done);
}

/*
 * Push one plane update. Sleeps for at most ADBE_KICK_WAIT_MS on the
 * pre-kick gate; a busy FIFO skips the update instead of spinning. If
 * @event is given and vblank is on, it completes when the pipe applies
 * the packet. Returns 0 if the packet was pushed.
 */
static int adbe_kick(struct adbe *adbe, u32 base, u32 stride,
		     struct drm_pending_vblank_event **event)
{
	struct adbe_pkt pkt;
	u32 parmst, ptr, status;
	unsigned long flags;
	bool armed = false;
	int ret;

	ret = adbe_pkt_build(&pkt, adbe->adp, adbe->commit, base, stride);
	if (ret)
		return ret;
	ret = adbe_fifo_wait_ready(adbe->adp, ADBE_KICK_WAIT_MS, &parmst, &ptr);
	if (ret) {
		atomic_inc(&adbe->busy);
		dev_warn_ratelimited(adbe->drm.dev,
				     "FIFO busy (0x101c=%#x 0x1040=%#x), update skipped\n",
				     parmst, ptr);
		return ret;
	}
	if (event && *event && !drm_crtc_vblank_get(&adbe->crtc))
		armed = true;
	spin_lock_irqsave(&adbe->drm.event_lock, flags);
	/* Completions before this push belong to an earlier packet. */
	status = readl(adbe->adp + ADP_STATUS) & ADP_ST_DONE;
	if (status) {
		adbe_count_done(adbe, status);
		writel(status, adbe->adp + ADP_STATUS);
	}
	if (adbe->commit == ADBE_COMMIT_SYNC) {
		if (readl(adbe->adp + ADP_STRIDE) != stride)
			writel(stride, adbe->adp + ADP_STRIDE);
		writel(base, adbe->adp + ADP_BASE);
	}
	adbe->pending_base = base;
	/* From here on, a new completion can only be this packet's. */
	adbe->kick_done = atomic_read(&adbe->completions);
	adbe_pkt_push(adbe->adp, &pkt, adbe->token, ++adbe->serial, true);
	atomic_inc(&adbe->kicks);
	if (armed) {
		adbe->event = *event;
		adbe->event_vbls = 0;
		adbe->event_applied = false;
		adbe->kick_seq = drm_crtc_vblank_count(&adbe->crtc);
		adbe->kick_time = ktime_get();
		*event = NULL;
	}
	spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	if (armed)
		mod_timer(&adbe->flip_timer,
			  jiffies + msecs_to_jiffies(ADBE_FLIP_TIMEOUT_MS));
	adbe->cur_base = base;
	return 0;
}

/*
 * Scan out the boot framebuffer, cleared to black: "off" for the CRTC and
 * the state left behind on unbind, shutdown and kexec. Polls for the
 * completion (the vblank interrupt is off by now, and the completion is
 * counted like any other); if the packet is not confirmed, falls back to
 * a direct store plus an empty packet.
 */
static void adbe_park(struct adbe *adbe)
{
	struct adbe_pkt empty = { .count = 0 };
	u32 parmst, ptr;
	unsigned int ms;

	if (!adbe->boot_black) {
		memset_io(adbe->boot_va, 0, ADBE_PITCH * ADBE_HEIGHT);
		wmb();	/* publish write-combined stores before the flip */
		adbe->boot_black = true;
	}
	if (adbe->cur_base == adbe->boot_base &&
	    readl(adbe->adp + ADP_BASE) == adbe->boot_base)
		return;
	if (!adbe_kick(adbe, adbe->boot_base, adbe->boot_stride, NULL)) {
		for (ms = 0; ms < ADBE_PARK_WAIT_MS; ms++) {
			unsigned long flags;
			u32 status;

			/* Same lock as the interrupt handler: count it once. */
			spin_lock_irqsave(&adbe->drm.event_lock, flags);
			status = readl(adbe->adp + ADP_STATUS) & ADP_ST_DONE;
			if (status) {
				adbe_count_done(adbe, status);
				writel(status, adbe->adp + ADP_STATUS);
			}
			spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
			if (atomic_read(&adbe->completions) != adbe->kick_done) {
				if (readl(adbe->adp + ADP_BASE) == adbe->boot_base)
					return;
				break;
			}
			usleep_range(900, 1100);
		}
	}
	dev_warn(adbe->drm.dev, "park packet not confirmed; storing the boot base directly\n");
	writel(adbe->boot_base, adbe->adp + ADP_BASE);
	if (!adbe_fifo_wait_ready(adbe->adp, ADBE_KICK_WAIT_MS, &parmst, &ptr))
		adbe_pkt_push(adbe->adp, &empty, adbe->token, ++adbe->serial, true);
	adbe->cur_base = adbe->boot_base;
	msleep(50);
	if (readl(adbe->adp + ADP_BASE) != adbe->boot_base)
		dev_err(adbe->drm.dev, "boot base not restored (readback %#x)\n",
			readl(adbe->adp + ADP_BASE));
}

static int adbe_plane_atomic_check(struct drm_plane *plane,
				   struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct adbe *adbe = to_adbe(plane->dev);
	struct drm_crtc_state *crtc_state = NULL;
	struct drm_framebuffer *fb = new_state->fb;
	dma_addr_t addr;
	u64 len;
	u32 bus;
	int ret;

	if (new_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING, false, false);
	if (ret || !new_state->visible)
		return ret;
	if (fb->pitches[0] % ADBE_ALIGN || fb->pitches[0] < ADBE_PITCH ||
	    fb->pitches[0] > SZ_32K) {
		drm_dbg_kms(plane->dev, "pitch %u unsupported\n", fb->pitches[0]);
		return -EINVAL;
	}
	addr = drm_fb_dma_get_gem_addr(fb, new_state, 0);
	len = (u64)fb->pitches[0] * (ADBE_HEIGHT - 1) + ADBE_PITCH;
	if (!adbe_cpu_to_bus(addr, adbe->bus_offset, len, &bus)) {
		drm_dbg_kms(plane->dev, "buffer %pad not reachable on the pipe bus\n", &addr);
		return -EINVAL;
	}
	return 0;
}

static void adbe_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct adbe *adbe = to_adbe(plane->dev);
	struct drm_framebuffer *fb = new_state->fb;
	dma_addr_t addr;
	u32 bus;

	if (!new_state->visible)
		return;
	addr = drm_fb_dma_get_gem_addr(fb, new_state, 0);
	/* Checked in atomic_check; the length only matters there. */
	if (!adbe_cpu_to_bus(addr, adbe->bus_offset, ADBE_ALIGN, &bus))
		return;
	adbe->next_base = adbe_tag_word(bus, adbe->boot_base);
	adbe->next_stride = adbe_tag_word(fb->pitches[0], adbe->boot_stride);
	adbe->next_dirty = true;
}

static const struct drm_plane_helper_funcs adbe_plane_helper_funcs = {
	.atomic_check = adbe_plane_atomic_check,
	.atomic_update = adbe_plane_atomic_update,
};

static const struct drm_plane_funcs adbe_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const u32 adbe_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const u64 adbe_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int adbe_crtc_atomic_check(struct drm_crtc *crtc,
				  struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;
	if (crtc_state->mode.hdisplay != ADBE_WIDTH ||
	    crtc_state->mode.vdisplay != ADBE_HEIGHT)
		return -EINVAL;
	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

static void adbe_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_atomic_commit *state)
{
	struct adbe *adbe = to_adbe(crtc->dev);
	struct drm_pending_vblank_event *event = crtc->state->event;
	unsigned long flags;

	crtc->state->event = NULL;
	if (adbe->next_dirty) {
		adbe->next_dirty = false;
		/* On success the event moves into adbe->event. */
		adbe_kick(adbe, adbe->next_base, adbe->next_stride, &event);
	}
	if (!event)
		return;
	/* Nothing pushed: complete at the next vblank, or now if it is off. */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	atomic_inc(&adbe->vbl_events);
	if (drm_crtc_vblank_get(crtc) == 0)
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void adbe_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_atomic_commit *state)
{
	drm_crtc_vblank_on(crtc);
}

static void adbe_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_atomic_commit *state)
{
	struct adbe *adbe = to_adbe(crtc->dev);
	struct drm_pending_vblank_event *done;
	unsigned long flags;

	timer_delete_sync(&adbe->flip_timer);
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	done = adbe_take_event(adbe);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	adbe_send_event(adbe, done);
	drm_crtc_vblank_off(crtc);
	adbe->next_dirty = false;
	adbe_park(adbe);
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int adbe_enable_vblank(struct drm_crtc *crtc)
{
	struct adbe *adbe = to_adbe(crtc->dev);

	adbe->frame_valid = false;	/* no jump across a vblank-off gap */
	/*
	 * VBI stays latched while vblank is off; unmasking it would report a
	 * stale boundary as a vblank now. Clear it first (round 2b: 64 VBI
	 * interrupts for 63 frames).
	 */
	writel(ADP_ST_VBI, adbe->adp + ADP_STATUS);
	WRITE_ONCE(adbe->vbl_enabled, true);
	writel(adbe->saved_inten | ADBE_INTEN_VBLANK, adbe->adp + ADP_INTEN);
	return 0;
}

static void adbe_disable_vblank(struct drm_crtc *crtc)
{
	struct adbe *adbe = to_adbe(crtc->dev);

	/* The exact word iBoot left, not a bit clear. */
	writel(adbe->saved_inten, adbe->adp + ADP_INTEN);
	WRITE_ONCE(adbe->vbl_enabled, false);
}

static const struct drm_crtc_helper_funcs adbe_crtc_helper_funcs = {
	.atomic_check = adbe_crtc_atomic_check,
	.atomic_flush = adbe_crtc_atomic_flush,
	.atomic_enable = adbe_crtc_atomic_enable,
	.atomic_disable = adbe_crtc_atomic_disable,
};

static const struct drm_crtc_funcs adbe_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = adbe_enable_vblank,
	.disable_vblank = adbe_disable_vblank,
};

static int adbe_connector_get_modes(struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &adbe_mode);
}

static const struct drm_connector_helper_funcs adbe_connector_helper_funcs = {
	.get_modes = adbe_connector_get_modes,
};

static const struct drm_connector_funcs adbe_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs adbe_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* Enable the CRTC before planes, so a flip always has vblank available. */
static const struct drm_mode_config_helper_funcs adbe_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

/*
 * Whether a buffer sits in the default CMA area. When CMA is full, the DMA
 * allocator falls back to one high-order page allocation per buffer (order
 * 10 for a 12 MiB scanout buffer), and that is where reclaim and compaction
 * pressure comes from.
 */
static bool adbe_in_cma(struct device *dev, dma_addr_t addr, size_t size)
{
#if ADBE_CMA_KNOWN
	struct cma *cma = dev_get_cma_area(dev);
	phys_addr_t pa = dma_to_phys(dev, addr), base;

	if (!cma)
		return false;
	base = cma_get_base(cma);
	return pa >= base && pa + size <= base + cma_get_size(cma);
#else
	return false;
#endif
}

/*
 * Start tracking a new object; the caller holds a reference to it. A dumb
 * handle closed and reused by another thread before the lookup can name an
 * object that is already tracked: leave that one alone.
 */
static void adbe_bo_account(struct adbe *adbe, struct drm_gem_object *obj,
			    enum adbe_bo_kind kind)
{
	struct adbe_bo *bo = to_adbe_bo(obj);

	mutex_lock(&adbe->bo_lock);
	if (!list_empty(&bo->node)) {
		mutex_unlock(&adbe->bo_lock);
		return;
	}
	bo->kind = kind;
	bo->in_cma = adbe_in_cma(adbe->drm.dev, bo->dma.dma_addr, obj->size);
	bo->born = ktime_get();
	bo->pid = task_tgid_nr(current);
	get_task_comm(bo->comm, current);
	list_add_tail(&bo->node, &adbe->bos);
	adbe->bo_count[kind]++;
	adbe->bo_bytes[kind] += obj->size;
	if (bo->in_cma)
		adbe->bo_cma_bytes[kind] += obj->size;
	adbe->bo_dumb_peak = max(adbe->bo_dumb_peak, adbe->bo_bytes[ADBE_BO_DUMB]);
	mutex_unlock(&adbe->bo_lock);
}

static void adbe_gem_free(struct drm_gem_object *obj)
{
	struct adbe *adbe = to_adbe(obj->dev);
	struct adbe_bo *bo = to_adbe_bo(obj);

	mutex_lock(&adbe->bo_lock);
	if (!list_empty(&bo->node)) {
		list_del(&bo->node);
		adbe->bo_count[bo->kind]--;
		adbe->bo_bytes[bo->kind] -= obj->size;
		if (bo->in_cma)
			adbe->bo_cma_bytes[bo->kind] -= obj->size;
	}
	mutex_unlock(&adbe->bo_lock);
	drm_gem_dma_free(&bo->dma);
}

/* A mapping holds an object reference: count them, for orphaned mmaps. */
static void adbe_gem_vm_open(struct vm_area_struct *vma)
{
	atomic_inc(&to_adbe_bo(vma->vm_private_data)->mmaps);
	drm_gem_vm_open(vma);
}

static void adbe_gem_vm_close(struct vm_area_struct *vma)
{
	atomic_dec(&to_adbe_bo(vma->vm_private_data)->mmaps);
	drm_gem_vm_close(vma);
}

static const struct vm_operations_struct adbe_gem_vm_ops = {
	.open = adbe_gem_vm_open,
	.close = adbe_gem_vm_close,
};

static int adbe_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	int ret = drm_gem_dma_mmap(to_drm_gem_dma_obj(obj), vma);

	if (!ret)
		atomic_inc(&to_adbe_bo(obj)->mmaps);
	return ret;
}

/* drm_gem_dma_default_funcs, plus the bookkeeping. */
static const struct drm_gem_object_funcs adbe_gem_funcs = {
	.free = adbe_gem_free,
	.print_info = drm_gem_dma_object_print_info,
	.get_sg_table = drm_gem_dma_object_get_sg_table,
	.vmap = drm_gem_dma_object_vmap,
	.mmap = adbe_gem_mmap,
	.vm_ops = &adbe_gem_vm_ops,
};

static struct drm_gem_object *adbe_gem_create_object(struct drm_device *drm,
						     size_t size)
{
	struct adbe_bo *bo = kzalloc(sizeof(*bo), GFP_KERNEL);

	if (!bo)
		return ERR_PTR(-ENOMEM);
	INIT_LIST_HEAD(&bo->node);
	bo->dma.base.funcs = &adbe_gem_funcs;
	return &bo->dma.base;
}

static int adbe_dumb_create(struct drm_file *file, struct drm_device *drm,
			    struct drm_mode_create_dumb *args)
{
	struct drm_gem_object *obj;
	int ret = drm_mode_size_dumb(drm, args, ADBE_ALIGN, 0);

	if (!ret)
		ret = drm_gem_dma_dumb_create_internal(file, drm, args);
	if (ret)
		return ret;
	/* Only the handle holds it, and another thread may close it already. */
	obj = drm_gem_object_lookup(file, args->handle);
	if (obj) {
		adbe_bo_account(to_adbe(drm), obj, ADBE_BO_DUMB);
		drm_gem_object_put(obj);
	}
	return 0;
}

static struct drm_gem_object *
adbe_gem_prime_import_sg_table(struct drm_device *drm,
			       struct dma_buf_attachment *attach,
			       struct sg_table *sgt)
{
	struct drm_gem_object *obj;

	obj = drm_gem_dma_prime_import_sg_table(drm, attach, sgt);
	if (!IS_ERR(obj))
		adbe_bo_account(to_adbe(drm), obj, ADBE_BO_IMPORT);
	return obj;
}

/* Buffers per client: GEM handles and the framebuffers made from them. */
static void adbe_show_clients(struct seq_file *m, struct adbe *adbe)
{
	struct drm_device *drm = &adbe->drm;
	struct drm_file *file;

	mutex_lock(&drm->filelist_mutex);
	list_for_each_entry_reverse(file, &drm->filelist, lhead) {
		unsigned int handles[ADBE_BO_KINDS] = {}, fbs = 0;
		u64 bytes[ADBE_BO_KINDS] = {};
		bool master = drm_is_current_master(file);
		struct drm_framebuffer *fb;
		struct drm_gem_object *obj;
		struct task_struct *task;
		struct pid *pid;
		int id;

		spin_lock(&file->table_lock);
		idr_for_each_entry(&file->object_idr, obj, id) {
			enum adbe_bo_kind kind = to_adbe_bo(obj)->kind;

			handles[kind]++;
			bytes[kind] += obj->size;
		}
		spin_unlock(&file->table_lock);
		mutex_lock(&file->fbs_lock);
		list_for_each_entry(fb, &file->fbs, filp_head)
			fbs++;
		mutex_unlock(&file->fbs_lock);

		rcu_read_lock();	/* file->pid and the task's comm */
		pid = rcu_dereference(file->pid);
		task = pid_task(pid, PIDTYPE_TGID);
		seq_printf(m, "client %s[%d] master=%c fbs=%u dumb=%u/%lluK import=%u/%lluK\n",
			   task ? task->comm : "?", pid_vnr(pid), master ? 'y' : 'n',
			   fbs, handles[ADBE_BO_DUMB], bytes[ADBE_BO_DUMB] >> 10,
			   handles[ADBE_BO_IMPORT], bytes[ADBE_BO_IMPORT] >> 10);
		rcu_read_unlock();
	}
	mutex_unlock(&drm->filelist_mutex);
}

/*
 * Memory summary. Orphans have no handle left: only a framebuffer, a CPU
 * mapping, a dma-buf export or the kernel (fbdev) keeps them alive.
 */
static void adbe_show_memory(struct seq_file *m, struct adbe *adbe)
{
	struct drm_vblank_crtc *vblank = drm_crtc_vblank_crtc(&adbe->crtc);
	unsigned int orphans = 0, mmaps = 0;
	u64 orphan_bytes = 0;
	struct adbe_bo *bo;
	unsigned long flags;
	bool pending;
	int fbs;

	mutex_lock(&adbe->bo_lock);
	list_for_each_entry(bo, &adbe->bos, node) {
		mmaps += atomic_read(&bo->mmaps);
		if (!READ_ONCE(bo->dma.base.handle_count)) {
			orphans++;
			orphan_bytes += bo->dma.base.size;
		}
	}
	seq_printf(m, "gem dumb=%u/%lluK import=%u/%lluK dumb-peak=%lluK orphans=%u/%lluK mmaps=%u",
		   adbe->bo_count[ADBE_BO_DUMB], adbe->bo_bytes[ADBE_BO_DUMB] >> 10,
		   adbe->bo_count[ADBE_BO_IMPORT], adbe->bo_bytes[ADBE_BO_IMPORT] >> 10,
		   adbe->bo_dumb_peak >> 10, orphans, orphan_bytes >> 10, mmaps);
	if (ADBE_CMA_KNOWN)
		seq_printf(m, " dumb-in-cma=%lluK import-in-cma=%lluK",
			   adbe->bo_cma_bytes[ADBE_BO_DUMB] >> 10,
			   adbe->bo_cma_bytes[ADBE_BO_IMPORT] >> 10);
	else
		seq_puts(m, " dumb-in-cma=? import-in-cma=?");
	mutex_unlock(&adbe->bo_lock);

	mutex_lock(&adbe->drm.mode_config.fb_lock);
	fbs = adbe->drm.mode_config.num_fb;
	mutex_unlock(&adbe->drm.mode_config.fb_lock);
	seq_printf(m, " fbs=%d cma-free=%luK\n", fbs,
		   global_zone_page_state(NR_FREE_CMA_PAGES) << (PAGE_SHIFT - 10));

	spin_lock_irqsave(&adbe->drm.event_lock, flags);
	pending = adbe->event;
	spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	seq_printf(m, "vblank refcount=%d enabled=%d vbi=%d vbl-events=%d flip-pending=%d timer=%d\n",
		   atomic_read(&vblank->refcount), READ_ONCE(vblank->enabled),
		   atomic_read(&adbe->vbis), atomic_read(&adbe->vbl_events),
		   pending, timer_pending(&adbe->flip_timer));
	adbe_show_clients(m, adbe);
}

/* One line per live GEM object, oldest first. */
static int adbe_bos_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct adbe *adbe = to_adbe(entry->dev);
	ktime_t now = ktime_get();
	struct adbe_bo *bo;

	mutex_lock(&adbe->bo_lock);
	list_for_each_entry(bo, &adbe->bos, node) {
		struct drm_gem_object *obj = &bo->dma.base;

		seq_printf(m, "%-6s %6zuK dma=%pad cma=%c handles=%u mmaps=%d refs=%u exported=%c age=%llds by %s[%d]\n",
			   adbe_bo_kind_names[bo->kind], obj->size >> 10,
			   &bo->dma.dma_addr,
			   ADBE_CMA_KNOWN ? (bo->in_cma ? 'y' : 'n') : '?',
			   READ_ONCE(obj->handle_count), atomic_read(&bo->mmaps),
			   kref_read(&obj->refcount), READ_ONCE(obj->dma_buf) ? 'y' : 'n',
			   ktime_ms_delta(now, bo->born) / MSEC_PER_SEC,
			   bo->comm, bo->pid);
	}
	mutex_unlock(&adbe->bo_lock);
	return 0;
}

static int adbe_state_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct adbe *adbe = to_adbe(entry->dev);
	u32 count = READ_ONCE(adbe->lat_count);
	/* A packet not yet applied: the FIFO still holds it. */
	bool in_flight = readl(adbe->adp + ADP_FIFO_PTR) & ADP_FIFO_PTR_BUSY;

	seq_printf(m, "commit=%s token=%s kicks=%d completions=%d in-flight=%u applied-0x400=%d synced-0x8=%d busy=%d vbl-fallbacks=%d timeouts=%d latch-mismatch=%d frame-jumps=%d errors=%#x\n",
		   adbe_commit_names[adbe->commit], adbe_token_names[adbe->token],
		   atomic_read(&adbe->kicks), atomic_read(&adbe->completions),
		   in_flight, atomic_read(&adbe->applied),
		   atomic_read(&adbe->synced), atomic_read(&adbe->busy),
		   atomic_read(&adbe->vbl_fallbacks), atomic_read(&adbe->timeouts),
		   atomic_read(&adbe->latch_mismatch),
		   atomic_read(&adbe->frame_jumps), atomic_read(&adbe->errors));
	seq_printf(m, "flip-latency frames 0:%d 1:%d 2:%d 3+:%d; us avg=%llu max=%u (n=%u)\n",
		   atomic_read(&adbe->lat_frames[0]), atomic_read(&adbe->lat_frames[1]),
		   atomic_read(&adbe->lat_frames[2]), atomic_read(&adbe->lat_frames[3]),
		   count ? READ_ONCE(adbe->lat_us_sum) / count : 0,
		   READ_ONCE(adbe->lat_us_max), count);
	seq_printf(m, "base=%#x stride=%#x boot-base=%#x bus-offset=%#llx inten=%#x saved-inten=%#x\n",
		   readl(adbe->adp + ADP_BASE), readl(adbe->adp + ADP_STRIDE),
		   adbe->boot_base, adbe->bus_offset,
		   readl(adbe->adp + ADP_INTEN), adbe->saved_inten);
	seq_printf(m, "status=%#x frame=%u 0x101c=%#x 0x1040=%#x\n",
		   readl(adbe->adp + ADP_STATUS),
		   readl(adbe->adp + ADP_FRAME) & 0xffff,
		   readl(adbe->adp + ADP_PARMST), readl(adbe->adp + ADP_FIFO_PTR));
	adbe_show_memory(m, adbe);
	return 0;
}

DEFINE_DRM_GEM_DMA_FOPS(adbe_fops);

static const struct drm_driver adbe_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &adbe_fops,
	.gem_create_object = adbe_gem_create_object,
	.dumb_create = adbe_dumb_create,
	.gem_prime_import_sg_table = adbe_gem_prime_import_sg_table,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "adbe",
	.desc = "Apple A9 display pipe",
	.major = 1,
	.minor = 0,
};

static void adbe_put_node(void *np)
{
	of_node_put(np);
}

/* The simple-framebuffer iBoot set up: CPU address and geometry. */
static int adbe_boot_framebuffer(struct adbe *adbe)
{
	struct device_node *np;
	const char *format;
	u32 width, height, stride;
	int ret = -ENODEV;

	for_each_compatible_node(np, NULL, "simple-framebuffer") {
		if (!of_device_is_available(np))
			continue;
		ret = of_reserved_mem_region_to_resource(np, 0, &adbe->boot);
		if (ret)
			ret = of_address_to_resource(np, 0, &adbe->boot);
		if (ret)
			break;
		ret = of_property_read_u32(np, "width", &width);
		ret |= of_property_read_u32(np, "height", &height);
		ret |= of_property_read_u32(np, "stride", &stride);
		ret |= of_property_read_string(np, "format", &format);
		if (ret || width != ADBE_WIDTH || height != ADBE_HEIGHT ||
		    stride != ADBE_PITCH || strcmp(format, "x8r8g8b8") ||
		    resource_size(&adbe->boot) < ADBE_PITCH * ADBE_HEIGHT)
			ret = -EINVAL;
		break;
	}
	if (!ret)
		adbe->boot_np = of_node_get(np);
	of_node_put(np);
	if (ret)
		return ret;
	return devm_add_action_or_reset(adbe->drm.dev, adbe_put_node, adbe->boot_np);
}

/*
 * Retire the firmware framebuffer device: the aperture helpers unplug
 * simpledrm if it is bound, and a device simpledrm has not bound yet (a
 * built-in probe order, or a deferred probe) is unregistered so that it
 * cannot take the boot framebuffer back later.
 */
static int adbe_take_boot_fb(struct adbe *adbe)
{
	struct platform_device *fb;
	int ret;

	ret = aperture_remove_conflicting_devices(adbe->boot.start,
						  resource_size(&adbe->boot),
						  adbe_drm_driver.name);
	if (ret)
		return ret;
	fb = of_find_device_by_node(adbe->boot_np);
	if (fb) {
		dev_info(adbe->drm.dev, "retiring %s before simpledrm binds it\n",
			 dev_name(&fb->dev));
		platform_device_unregister(fb);
		put_device(&fb->dev);
	}
	return 0;
}

/* Everything is read-only until the aperture is taken over. */
static int adbe_adopt(struct adbe *adbe)
{
	struct device *dev = adbe->drm.dev;
	struct adbe_dart dart = {};
	u32 base, stride;
	u16 frame;
	int ret;

	if ((readl(adbe->adp + ADP_SPDS) & 0xffff) != ADP_SPDS_VERSION ||
	    !adbe_format_known(readl(adbe->adp + ADP_FORMAT)) ||
	    !(readl(adbe->adp + ADP_VALID) & 1) ||
	    (readl(adbe->adp + ADP_LAYERS) & 0xf) != 1 ||
	    (readl(adbe->adp + ADP_STATUS) & ADP_ST_ERRORS))
		return dev_err_probe(dev, -ENODEV, "pipe is not running as iBoot leaves it\n");
	frame = readl(adbe->adp + ADP_FRAME);
	msleep(50);
	if ((u16)readl(adbe->adp + ADP_FRAME) == frame)
		return dev_err_probe(dev, -ENODEV, "frame counter is not advancing\n");
	ret = adbe_boot_framebuffer(adbe);
	if (ret)
		return dev_err_probe(dev, ret, "need the live 1536x2048 simple-framebuffer\n");
	base = readl(adbe->adp + ADP_BASE);
	stride = readl(adbe->adp + ADP_STRIDE);
	if ((stride & ~(ADBE_ALIGN - 1)) != ADBE_PITCH ||
	    (base & ~(ADBE_ALIGN - 1)) != lower_32_bits(adbe->boot.start))
		return dev_err_probe(dev, -ENODEV,
				     "plane %#x/%#x is not the boot framebuffer %pa\n",
				     base, stride, &adbe->boot.start);
	adbe->boot_base = base;
	adbe->boot_stride = stride;
	adbe->cur_base = base;
	adbe->bus_offset = adbe->boot.start - (base & ~(ADBE_ALIGN - 1));
	adbe->saved_inten = readl(adbe->adp + ADP_INTEN);

	ret = adbe_dart_read(dev, &dart);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the display DART\n");
	ret = adbe_dart_scanout_streams(&dart, base & ~(ADBE_ALIGN - 1), adbe->boot.start);
	if (ret)
		return dev_err_probe(dev, ret < 0 ? ret : -ENODEV,
				     "display DART %s; DMA buffers would not be reachable\n",
				     ret < 0 ? "state unknown" : "translates the scanout");
	dev_info(dev, "adopting %ux%u at %pa (plane %#x, stride %#x, bus offset %#llx, DART bypass, inten %#x)\n",
		 ADBE_WIDTH, ADBE_HEIGHT, &adbe->boot.start, base, stride,
		 adbe->bus_offset, adbe->saved_inten);
	return 0;
}

static int adbe_modeset_init(struct adbe *adbe)
{
	struct drm_device *drm = &adbe->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = 32;
	drm->mode_config.min_height = 32;
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.preferred_depth = 24;
	/* Write-combined buffers: CPU readback is slow, render elsewhere. */
	drm->mode_config.prefer_shadow = 1;
	drm->mode_config.funcs = &adbe_mode_config_funcs;
	drm->mode_config.helper_private = &adbe_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &adbe->plane, BIT(0), &adbe_plane_funcs,
				       adbe_formats, ARRAY_SIZE(adbe_formats),
				       adbe_modifiers, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&adbe->plane, &adbe_plane_helper_funcs);

	ret = drmm_crtc_init_with_planes(drm, &adbe->crtc, &adbe->plane, NULL,
					 &adbe_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&adbe->crtc, &adbe_crtc_helper_funcs);

	ret = drmm_encoder_init(drm, &adbe->encoder, NULL, DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	adbe->encoder.possible_crtcs = drm_crtc_mask(&adbe->crtc);

	ret = drmm_connector_init(drm, &adbe->connector, &adbe_connector_funcs,
				  DRM_MODE_CONNECTOR_eDP, NULL);
	if (ret)
		return ret;
	drm_connector_helper_add(&adbe->connector, &adbe_connector_helper_funcs);
	ret = drm_connector_attach_encoder(&adbe->connector, &adbe->encoder);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	return 0;
}

static void adbe_teardown(struct adbe *adbe)
{
	drm_atomic_helper_shutdown(&adbe->drm);
	timer_delete_sync(&adbe->flip_timer);
	/* Even if the CRTC was never enabled: boot buffer, iBoot's mask. */
	adbe_park(adbe);
	writel(adbe->saved_inten, adbe->adp + ADP_INTEN);
}

static int adbe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adbe *adbe;
	int ret;

	if (!enable) {
		dev_info(dev, "disabled by apple_adbe_drm.enable=0; simpledrm keeps the display\n");
		return -ENODEV;
	}
	adbe = devm_drm_dev_alloc(dev, &adbe_drm_driver, struct adbe, drm);
	if (IS_ERR(adbe))
		return PTR_ERR(adbe);
	platform_set_drvdata(pdev, adbe);
	kernel_param_lock(THIS_MODULE);
	ret = sysfs_match_string(adbe_commit_names, commit);
	if (ret >= 0) {
		adbe->commit = ret;
		ret = sysfs_match_string(adbe_token_names, token);
		adbe->token = ret;
	}
	kernel_param_unlock(THIS_MODULE);
	if (ret < 0)
		return dev_err_probe(dev, ret, "commit: base|full|sync, token: next|same|serial\n");
	timer_setup(&adbe->flip_timer, adbe_flip_timeout, 0);
	INIT_LIST_HEAD(&adbe->bos);
	ret = drmm_mutex_init(&adbe->drm, &adbe->bo_lock);
	if (ret)
		return ret;

	adbe->adp = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adbe->adp))
		return PTR_ERR(adbe->adp);
	ret = adbe_adopt(adbe);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;
	adbe->boot_va = devm_ioremap_wc(dev, adbe->boot.start, ADBE_PITCH * ADBE_HEIGHT);
	if (!adbe->boot_va)
		return -ENOMEM;
	ret = adbe_modeset_init(adbe);
	if (ret)
		return ret;
	adbe->irq = platform_get_irq(pdev, 0);
	if (adbe->irq < 0)
		return adbe->irq;
	/* No packet of ours yet: a latched completion is not one to count. */
	writel(ADP_ST_DONE, adbe->adp + ADP_STATUS);
	/* After vblank init; iBoot's mask (OutUnder only) until vblank is on. */
	ret = devm_request_irq(dev, adbe->irq, adbe_irq, 0, dev_name(dev), adbe);
	if (ret)
		return ret;

	/* Past this point simpledrm is gone; the pipe still shows its buffer. */
	ret = adbe_take_boot_fb(adbe);
	if (ret)
		return dev_err_probe(dev, ret, "cannot take over the boot framebuffer\n");
	ret = drm_dev_register(&adbe->drm, 0);
	if (ret) {
		dev_err(dev, "registration failed (%d); the panel keeps the last boot image\n",
			ret);
		return ret;
	}
	drm_debugfs_add_file(&adbe->drm, "adbe_state", adbe_state_show, NULL);
	drm_debugfs_add_file(&adbe->drm, "adbe_bos", adbe_bos_show, NULL);
	drm_client_setup(&adbe->drm, NULL);
	return 0;
}

static void adbe_remove(struct platform_device *pdev)
{
	struct adbe *adbe = platform_get_drvdata(pdev);

	drm_dev_unplug(&adbe->drm);
	adbe_teardown(adbe);
}

static void adbe_shutdown(struct platform_device *pdev)
{
	adbe_teardown(platform_get_drvdata(pdev));
}

static int adbe_suspend(struct device *dev)
{
	struct adbe *adbe = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&adbe->drm);
}

static int adbe_resume(struct device *dev)
{
	struct adbe *adbe = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&adbe->drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(adbe_pm_ops, adbe_suspend, adbe_resume);

static const struct of_device_id adbe_of_match[] = {
	{ .compatible = "adbe0,s8000" },
	{}
};
MODULE_DEVICE_TABLE(of, adbe_of_match);

static struct platform_driver adbe_platform_driver = {
	.probe = adbe_probe,
	.remove = adbe_remove,
	.shutdown = adbe_shutdown,
	.driver = {
		.name = "apple-adbe",
		.of_match_table = adbe_of_match,
		.pm = pm_sleep_ptr(&adbe_pm_ops),
	},
};
module_platform_driver(adbe_platform_driver);

MODULE_DESCRIPTION("DRM driver for the Apple A9 display pipe");
MODULE_AUTHOR("Berkay Kösebay <brkosebay@users.noreply.github.com>");
MODULE_LICENSE("GPL");

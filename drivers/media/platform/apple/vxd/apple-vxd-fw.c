// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - boot the resident DEVA firmware on the MTX core (R1)
 *
 * The DEVA image (REL_812.0_14_Deva, 97,944 bytes, built 08/11/2016) ships
 * inside the iOS AppleD5500 kext; it is not committed to this tree.  Obtain
 * it as omarchy-ipad5 docs/periph/ds-video-decoder.md describes and stage
 * it as /lib/firmware/apple/vxd-deva.bin (sha256
 * bf5167652e81540471aaca141943fd33d71c3990c137f145471305f5f5ade83b).
 *
 * Everything below is transcribed from the kext with the annotation tooling
 * (tools/ds-video/) and follows AppleD5500/CPVDECCore's exact call and
 * write order:
 *
 *   PrepareCore:
 *     known state v1, core discovery (InitializeCoreConfig), firmware
 *     buffer allocation (dma + GART map + copy)
 *   LoadFirmwareAndStartCore:
 *     known state v1 (again), the clock save/quiesce/restore dance,
 *     known state v2, ProcessorInitialise (MMU + comms rings), the MTX
 *     boot block, 0x418 = 0x100000, post-boot-init registers
 *   poll 0x2014 for 0xa5a5a5a5, then read 0x2024 / 0x2018
 *
 * Every step announces itself with dev_emerg + console flush (vxd_step)
 * so the last line before a crash names it.
 */
#include <crypto/sha2.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/iopoll.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "apple-vxd.h"

static const u8 apple_vxd_deva_sha256[SHA256_DIGEST_SIZE] = {
	0xbf, 0x51, 0x67, 0x65, 0x2e, 0x81, 0x54, 0x04,
	0x71, 0xaa, 0xca, 0x14, 0x19, 0x43, 0xfd, 0x33,
	0xd7, 0x1c, 0x39, 0x90, 0xc1, 0x37, 0xf1, 0x45,
	0x47, 0x13, 0x05, 0xf5, 0xf5, 0xad, 0xe8, 0x3b,
};

/*
 * PollMSVDX [E]: poll until (val & mask) == match, 1 ms steps, until the
 * millisecond timeout.
 */
static int vxd_poll_msvdx(struct apple_vxd_dev *vxd, u32 off, u32 mask,
			  u32 match, unsigned long timeout_ms)
{
	u32 val;

	return readl_poll_timeout(vxd->regs + off, val, (val & mask) == match,
				  1000, timeout_ms * 1000);
}

/*
 * SetClocks [E]: write default, verify the read-back, then write the
 * value.  The kext's read-back check is an assert-and-continue (only the
 * register polls below decide), so a mismatch is a warning here, not a
 * boot abort.
 */
static void vxd_set_clocks(struct apple_vxd_dev *vxd, u32 val)
{
	u32 readback;

	vxd_write(vxd, VXD_REG_CR_PVDEC_MAN_CLK_ENABLE, VXD_CLK_ENABLE_DEFAULT);
	readback = vxd_read(vxd, VXD_REG_CR_PVDEC_MAN_CLK_ENABLE);
	if (readback != VXD_CLK_ENABLE_DEFAULT)
		dev_warn(vxd->dev,
			 "SetClocks - core register write differs from read (wrote %#x, read %#x): core not powered/clocked?\n",
			 VXD_CLK_ENABLE_DEFAULT, readback);

	vxd_write(vxd, VXD_REG_CR_PVDEC_MAN_CLK_ENABLE, val);
}

/*
 * Known state v1 [E] (CPVDECCore fffffff006297f94) - called by both
 * PrepareCore and LoadFirmwareAndStartCore; two attempts, then the
 * epilogue writes.  Every attempt: clocks on, pipe resets, CMD_CTRL
 * pulse 0xffff0000/0/1 with a busy-clear poll, then the MTX clock select
 * poll.
 */
static int vxd_known_state_v1(struct apple_vxd_dev *vxd, const char *which)
{
	unsigned int attempt, pipe;
	int ret;

	for (attempt = 0; attempt < 2; attempt++) {
		vxd_step(vxd->dev, "known state v1: clocks");
		vxd_set_clocks(vxd, VXD_CLK_ENABLE_ALL);

		vxd_step(vxd->dev, "known state v1: pipe resets");
		for (pipe = 1; pipe <= vxd->num_pixel_pipes; pipe++) {
			vxd_write(vxd, VXD_REG_CMD_KICK, pipe);
			vxd_write(vxd, VXD_REG_PIPE_CTRL, 0xffffffff);
		}

		for (pipe = 1; pipe <= vxd->num_entropy_pipes; pipe++) {
			vxd_write(vxd, VXD_REG_CMD_KICK, pipe);
			vxd_write(vxd, VXD_REG_PIPE_SET_0, 0xffffffff);
		}

		vxd_step(vxd->dev, "known state v1: CMD_CTRL pulse");
		vxd_write(vxd, VXD_REG_CMD_CTRL, VXD_CMD_CTRL_RESET);
		vxd_read(vxd, VXD_REG_CMD_CTRL);	/* APB flush read [E] */
		vxd_write(vxd, VXD_REG_CMD_CTRL, 0);
		vxd_write(vxd, VXD_REG_CMD_CTRL, 1);

		ret = vxd_poll_msvdx(vxd, VXD_REG_CMD_CTRL, 1, 0, 100);
		if (ret) {
			dev_err(vxd->dev,
				"known state v1 (%s): 0x400 busy poll attempt %u failed: %d\n",
				which, attempt, ret);
			return ret;
		}

		vxd_step(vxd->dev, "known state v1: MTX clock select");
		vxd_write(vxd, VXD_REG_MTX_MAN_CLK_ENABLE, VXD_MTX_CLK_SEL);
		ret = vxd_poll_msvdx(vxd, VXD_REG_MTX_MAN_CLK_ENABLE,
				     VXD_MTX_CLK_SEL, 0, 100);
		if (ret) {
			dev_err(vxd->dev,
				"known state v1 (%s): 0x1008 poll attempt %u failed: %d\n",
				which, attempt, ret);
			return ret;
		}
	}

	/* Epilogue [E]: host interrupt disabled, pending causes cleared. */
	vxd_write(vxd, VXD_REG_IRQ_ENABLE, 0);
	vxd_write(vxd, VXD_REG_IRQ_CLEAR, 0xffff0000);
	return 0;
}

/*
 * Known state v2 [E] (fffffff0062980fc) - immediately before
 * ProcessorInitialise; same reset value, but the MTX clock poll runs on
 * the first attempt only and the epilogue differs.
 */
static int vxd_known_state_v2(struct apple_vxd_dev *vxd)
{
	unsigned int attempt, pipe;
	int ret;

	for (attempt = 0; attempt < 2; attempt++) {
		vxd_set_clocks(vxd, VXD_CLK_ENABLE_ALL);

		for (pipe = 1; pipe <= vxd->num_pixel_pipes; pipe++) {
			vxd_write(vxd, VXD_REG_CMD_KICK, pipe);
			vxd_write(vxd, VXD_REG_PIPE_CTRL, 0xffffffff);
		}

		for (pipe = 1; pipe <= vxd->num_entropy_pipes; pipe++) {
			vxd_write(vxd, VXD_REG_CMD_KICK, pipe);
			vxd_write(vxd, VXD_REG_PIPE_SET_0, 0xffffffff);
		}

		vxd_step(vxd->dev, "known state v2: CMD_CTRL pulse");
		vxd_write(vxd, VXD_REG_CMD_CTRL, VXD_CMD_CTRL_RESET);
		vxd_read(vxd, VXD_REG_CMD_CTRL);	/* APB flush read [E] */
		vxd_write(vxd, VXD_REG_CMD_CTRL, 0);

		if (!attempt) {
			vxd_write(vxd, VXD_REG_MTX_MAN_CLK_ENABLE,
				  VXD_MTX_CLK_SEL);
			ret = vxd_poll_msvdx(vxd, VXD_REG_MTX_MAN_CLK_ENABLE,
					     VXD_MTX_CLK_SEL, 0, 100);
			if (ret)
				return ret;
		}
	}

	vxd_read(vxd, VXD_REG_COMMS_STATE);
	vxd_write(vxd, VXD_REG_IRQ_CLEAR, 0xffffffff);
	return 0;
}

/*
 * Core discovery [E] (InitializeCoreConfig fffffff006299330, head only):
 * the entropy-pipe feature flag, a clock save-and-set to the firmware
 * value, and the per-pixel-pipe config write with the pipe status reads.
 * The kext's per-pipe register recompute (a few hundred lines of bitfield
 * algebra on the statuses) is still TODO; the boot works with the
 * constructor defaults until the R2 dumps say otherwise.
 */
static void vxd_discover_core(struct apple_vxd_dev *vxd)
{
	u32 pipe, status = 0, maxa = 0;

	vxd->cfg_flags = ((vxd->num_entropy_pipes != 0) << 21);

	/*
	 * save 0x440, set the firmware clock value [E].  The saved value is
	 * not restored in this prologue (round 2 already left it unused), so
	 * only the read is kept, in the kext's order.
	 */
	vxd_read(vxd, VXD_REG_CR_PVDEC_MAN_CLK_ENABLE);
	vxd_set_clocks(vxd, VXD_CLK_ENABLE_FW);

	for (pipe = 1; pipe <= vxd->num_pixel_pipes; pipe++) {
		vxd_write(vxd, VXD_REG_CMD_KICK, pipe);
		vxd_write(vxd, VXD_REG_PIPE_CTRL, 0x20000);
		status |= vxd_read(vxd, VXD_REG_PIPE_STATUS);
		maxa = max(maxa, vxd_read(vxd, VXD_REG_PIPE_STATUS_2) & 0x3);
	}
	vxd->pipe_status_cfg = status;
	vxd->pipe_status_2_max = maxa;
}

/*
 * The quiesce dance [E] (sub_fffffff0062995cc, between the second known
 * state v1 and the boot): save 0x440, set the firmware value, touch the
 * pixel pipes with 0x20000, then restore the saved value.
 */
static void vxd_quiesce_clocks(struct apple_vxd_dev *vxd)
{
	u32 saved, pipe;

	saved = vxd_read(vxd, VXD_REG_CR_PVDEC_MAN_CLK_ENABLE);
	vxd_set_clocks(vxd, VXD_CLK_ENABLE_FW);

	for (pipe = 1; pipe <= vxd->num_pixel_pipes; pipe++) {
		vxd_write(vxd, VXD_REG_CMD_KICK, pipe);
		vxd_write(vxd, VXD_REG_PIPE_CTRL, 0x20000);
	}

	vxd_set_clocks(vxd, saved);
}

/*
 * ProcessorInitialise [E]: message ring setup and MMU programming.  The
 * 0x202c flags take the !core[0xc0] branch (0x1000000); the 0x1003000
 * branch and what core[0xc0] means are [U], pinned down by the R2 dump.
 */
static void vxd_processor_initialise(struct apple_vxd_dev *vxd)
{
	/* This core always programs its GART before MTX boot: there is no
	 * DART and no other address path [E].
	 */
	vxd_step(vxd->dev, "ProcessorInitialise: MMU programming");
	vxd_read(vxd, VXD_REG_MTX_MMU_CFG0);
	vxd_write(vxd, VXD_REG_MTX_MMU_0, 0);
	vxd_write(vxd, VXD_REG_MTX_MMU_DIR_ADR,
		  (u32)((u64)vxd->gart.dir_dma >> 8));
	vxd_write(vxd, VXD_REG_MTX_MMU_MODE,
		  vxd->mmu_40bit ? VXD_MTX_MMU_MODE_40BIT : 0);

	vxd_step(vxd->dev, "ProcessorInitialise: comms rings");
	vxd_write(vxd, VXD_REG_FW_READY_MAGIC, 0);
	vxd_write(vxd, VXD_REG_COMMS_MSG_COUNT, 200);
	vxd_write(vxd, VXD_REG_FW_PC, 0);
	vxd_write(vxd, VXD_REG_TO_HOST_RD_INDEX, 0);
	vxd_write(vxd, VXD_REG_TO_HOST_WR_INDEX, 0);
	vxd_write(vxd, VXD_REG_TO_MTX_RD_INDEX, 0);
	vxd_write(vxd, VXD_REG_TO_MTX_WR_INDEX, 0);
	vxd_write(vxd, VXD_REG_COMMS_2008, 0);
	vxd_write(vxd, VXD_REG_COMMS_FLAGS, 0x1000000 | 0x60);

	vxd_read(vxd, VXD_REG_COMMS_STATE);
}

/* MTX boot block programming [E] (BootFirmware). */
static void vxd_boot_mtx(struct apple_vxd_dev *vxd, u32 fw_words)
{
	vxd_step(vxd->dev, "MTX boot: clocks + pipe config");
	vxd_write(vxd, VXD_REG_CMD_KICK, 1);
	vxd_set_clocks(vxd, VXD_CLK_ENABLE_FW);

	vxd_write(vxd, VXD_REG_PIPE_CTRL, 0x30000);
	vxd_write(vxd, VXD_REG_ENTDEC_0_CFG, 0);
	vxd_write(vxd, VXD_REG_ENTDEC_0_SIZE,
		  (((fw_words + 15) & ~7) & ~0x10000) | 0x10000);
	vxd_write(vxd, VXD_REG_DMAC_STATUS, VXD_DMAC_STATUS_FLUSH);
	vxd_write(vxd, VXD_REG_PIPE_4004, 0);

	/* Boot block; BOOT_5 first (MTX clock enable; order is [E]). */
	vxd_step(vxd->dev, "MTX boot: boot block writes");
	vxd_write(vxd, VXD_REG_MTX_BOOT_5, VXD_MTX_BOOT_5_ENABLE);
	vxd_write(vxd, VXD_REG_MTX_BOOT_3, 0);
	vxd_write(vxd, VXD_REG_MTX_BOOT_FW_ADR, vxd->fw_dev_va);
	vxd_write(vxd, VXD_REG_MTX_BOOT_4, VXD_MTX_BOOT_4_VALUE);
	vxd_write(vxd, VXD_REG_MTX_BOOT_2, VXD_MTX_BOOT_2_VALUE);
	vxd_write(vxd, VXD_REG_MTX_BOOT_1,
		  VXD_MTX_BOOT_1_STAGE0 | fw_words);
	vxd_write(vxd, VXD_REG_MTX_BOOT_1,
		  VXD_MTX_BOOT_1_STAGE1 | fw_words);

	vxd_read(vxd, VXD_REG_COMMS_STATE);
}

/*
 * Post-boot-init [E]: comms flags, MTX_CTRL read-modify-write (bits 0 and
 * 16 come from core flags, zero on a fresh core) and the per-pipe
 * registers.  The pipe configuration values are the CPVDECCore constructor
 * defaults recovered from __TEXT.__const (the per-pipe recompute in
 * InitializeCoreConfig stays a TODO for R2).
 */
static void vxd_post_boot_init(struct apple_vxd_dev *vxd)
{
	u32 val;

	vxd_write(vxd, VXD_REG_COMMS_FLAGS, 0x1000000 | 0x60);

	val = vxd_read(vxd, VXD_REG_MTX_CTRL);
	val &= VXD_MTX_CTRL_KEEP_MASK;
	vxd_write(vxd, VXD_REG_MTX_CTRL, val);

	vxd_write(vxd, VXD_REG_CMD_KICK, 1);
	vxd_write(vxd, VXD_REG_PIPE_CR0, 0);
	vxd_write(vxd, VXD_REG_PIPE_CR1, 0x4ce);
	vxd_write(vxd, VXD_REG_PIPE_CR2, 0);
	vxd_write(vxd, VXD_REG_PIPE_CR3, 0x2008);
	vxd_write(vxd, VXD_REG_PIPE_SET_1, 0);
	vxd_write(vxd, VXD_REG_PIPE_SET_2, 0x334);
}

int apple_vxd_boot_firmware(struct apple_vxd_dev *vxd)
{
	const struct firmware *fw;
	u8 digest[SHA256_DIGEST_SIZE];
	u32 fw_words;
	int ret;

	mutex_lock(&vxd->power_lock);
	if (vxd->fw_loaded) {
		/* Up, but a mailbox that never took the init message stays an error. */
		ret = vxd->hw_ready ? 0 : -EIO;
		if (ret)
			dev_err(vxd->dev,
				"firmware is running but its mailbox did not start; unbind/bind to retry\n");
		mutex_unlock(&vxd->power_lock);
		return ret;
	}

	if (vxd->selftest_failed) {
		dev_err(vxd->dev, "mailbox self-test failed at probe: not booting the firmware\n");
		mutex_unlock(&vxd->power_lock);
		return -EINVAL;
	}

	ret = pm_runtime_resume_and_get(vxd->dev);
	if (ret) {
		mutex_unlock(&vxd->power_lock);
		return ret;
	}

	vxd_step(vxd->dev, "boot: power domain + PMGR pwrstate");
	apple_vxd_log_pwrstate(vxd, "boot");

	/*
	 * AllocateFirmwareBuffer [E]: a 4-byte-aligned coherent buffer of
	 * blob_size, GART-mapped (the MTX fetches through its own MMU -
	 * with the round-1 dummy-PTE slots this fetch address was
	 * unbacked), and the firmware must fit under 0x10000 MTX words.
	 */
	vxd_step(vxd->dev, "boot: request DEVA firmware + sha256");
	ret = request_firmware(&fw, APPLE_VXD_FW_NAME, vxd->dev);
	if (ret) {
		dev_err(vxd->dev,
			"DEVA firmware %s not available (%d); extract it from the iPadOS kernelcache (AppleD5500) and install it\n",
			APPLE_VXD_FW_NAME, ret);
		goto out_put;
	}

	if (!fw->size || (fw->size & 3) ||
	    fw->size / sizeof(u32) >= APPLE_VXD_FW_MAX_WORDS) {
		dev_err(vxd->dev, "firmware size %zu is not usable (< 0x10000 words)\n",
			fw->size);
		ret = -EINVAL;
		goto out_release;
	}
	fw_words = fw->size / sizeof(u32);
	vxd->fw_blob_size = fw->size;

	sha256(fw->data, fw->size, digest);
	if (memcmp(digest, apple_vxd_deva_sha256, sizeof(digest))) {
		print_hex_dump(KERN_WARNING, "vxd: unexpected fw digest ",
			       DUMP_PREFIX_OFFSET, 16, 1, digest,
			       sizeof(digest), false);
		dev_warn(vxd->dev,
			 "this is not the REL_812.0_14_Deva image we reverse engineered\n");
	} else {
		dev_info(vxd->dev, "DEVA firmware: REL_812.0_14_Deva (sha256 verified)\n");
	}

	vxd_step(vxd->dev, "boot: allocate + GART-map firmware buffer");
	vxd->fw_size = PAGE_ALIGN(fw->size);
	vxd->fw_vaddr = dma_alloc_coherent(vxd->dev, vxd->fw_size,
					   &vxd->fw_dma, GFP_KERNEL);
	if (!vxd->fw_vaddr) {
		ret = -ENOMEM;
		goto out_release;
	}

	/*
	 * Device VA = low 32 bits of the DMA address (identity GART), plus
	 * the va_offset module parameter applied to both the GART slot and
	 * the 0x6a00 fetch address, so a wrong aperture base can be tested
	 * without a rebuild (test plan T4).
	 */
	vxd->fw_dev_va = (u32)vxd->fw_dma + vxd->va_offset;
	ret = apple_vxd_gart_map_at(vxd, vxd->fw_dma, vxd->fw_size,
				    vxd->fw_dev_va, &vxd->fw_map);
	if (ret) {
		dev_err(vxd->dev,
			"failed to GART-map the firmware buffer (va %#x): %d\n",
			vxd->fw_dev_va, ret);
		goto out_fwbuf;
	}

	memcpy(vxd->fw_vaddr, fw->data, fw->size);
	release_firmware(fw);
	/* Every later failure unwinds through out_release: no double free. */
	fw = NULL;

	/*
	 * R3b: the eight command buffers are mapped now, before the MTX and
	 * its MMU start, so no mapping changes under a running core.
	 */
	vxd_step(vxd->dev, "boot: allocate + GART-map 8 x 256 KiB command buffers");
	ret = apple_vxd_cmdbufs_init(vxd);
	if (ret) {
		dev_err(vxd->dev, "command buffers: %d\n", ret);
		goto out_unmap;
	}

	/* PrepareCore [E], in the kext's order. */
	ret = vxd_known_state_v1(vxd, "PrepareCore");
	if (ret)
		goto out_cmdbufs;

	vxd_step(vxd->dev, "boot: core discovery");
	vxd_discover_core(vxd);

	/* LoadFirmwareAndStartCore [E], in the kext's order. */
	ret = vxd_known_state_v1(vxd, "LoadFirmwareAndStartCore");
	if (ret)
		goto out_cmdbufs;

	vxd_step(vxd->dev, "boot: clock save/quiesce/restore");
	vxd_quiesce_clocks(vxd);

	ret = vxd_known_state_v2(vxd);
	if (ret)
		goto out_cmdbufs;

	vxd_processor_initialise(vxd);
	vxd_boot_mtx(vxd, fw_words);

	vxd_step(vxd->dev, "boot: post-boot-init registers");
	vxd_write(vxd, VXD_REG_IRQ_ENABLE, VXD_IRQ_FW_MSG);
	vxd_post_boot_init(vxd);

	vxd_step(vxd->dev, "boot: firmware ready poll (1 s)");
	ret = vxd_poll_msvdx(vxd, VXD_REG_FW_READY_MAGIC, 0xffffffff,
			     VXD_FW_READY_MAGIC, 1000);
	if (ret) {
		dev_err(vxd->dev,
			"firmware ready indication - Poll returns %d (0 = no error): firmware did not come up\n",
			ret);
		goto out_cmdbufs;
	}

	/*
	 * 0x2024/0x2018 are the ring geometry the firmware publishes with the
	 * magic (rounds 1-3 logged them as "build halves").
	 */
	vxd_step(vxd->dev, "boot: firmware alive, reading the ring geometry (0x2024/0x2018)");
	ret = apple_vxd_mbox_open(vxd);
	if (ret)
		goto out_cmdbufs;

	/*
	 * The MTX runs from here on: keep the image, its mapping and the PM
	 * reference until apple_vxd_fw_cleanup(), whatever the mailbox does.
	 */
	vxd->fw_loaded = true;

	ret = apple_vxd_mbox_start(vxd);
	if (ret) {
		dev_err(vxd->dev,
			"DEVA firmware is up but did not take the init message (%d): decode stays off, debugfs mbox/ping still work\n",
			ret);
		mutex_unlock(&vxd->power_lock);
		return ret;
	}

	vxd->hw_ready = true;
	dev_info(vxd->dev,
		 "DEVA firmware ready: mailbox up, init message consumed, IRQ %d armed\n",
		 vxd->irq);
	mutex_unlock(&vxd->power_lock);
	return 0;

out_cmdbufs:
	apple_vxd_cmdbufs_fini(vxd);
out_unmap:
	apple_vxd_gart_unmap(vxd, &vxd->fw_map);
out_fwbuf:
	dma_free_coherent(vxd->dev, vxd->fw_size, vxd->fw_vaddr, vxd->fw_dma);
	vxd->fw_vaddr = NULL;
out_release:
	release_firmware(fw);
out_put:
	pm_runtime_put_autosuspend(vxd->dev);
	mutex_unlock(&vxd->power_lock);
	return ret;
}

void apple_vxd_fw_cleanup(struct apple_vxd_dev *vxd)
{
	bool was_loaded;

	mutex_lock(&vxd->power_lock);
	was_loaded = vxd->fw_loaded;
	/* IRQ off and every fence waiter woken before the buffers go. */
	apple_vxd_mbox_stop(vxd);
	apple_vxd_cmdbufs_fini(vxd);
	if (vxd->fw_vaddr) {
		apple_vxd_gart_unmap(vxd, &vxd->fw_map);
		dma_free_coherent(vxd->dev, vxd->fw_size, vxd->fw_vaddr,
				  vxd->fw_dma);
		vxd->fw_vaddr = NULL;
	}
	vxd->fw_loaded = false;
	vxd->hw_ready = false;
	mutex_unlock(&vxd->power_lock);

	if (was_loaded)
		pm_runtime_put_autosuspend(vxd->dev);
}

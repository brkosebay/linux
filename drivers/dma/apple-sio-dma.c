// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SmartIO DMA controller (S8000/T8010)
 *
 * Copyright (C) 2026 Paul Praschl
 * Multiple endpoints, cyclic transfers and stop handling: Berkay Kösebay
 *
 * Every DMA channel is a SmartIO firmware endpoint (0x1a SPI3 TX, 0x28 MCA0
 * TX, ...). The protocol, as iOS 16.7.10's AppleSmartIODMA drives it:
 *
 *   op 5 configure  data = descriptor slot holding the shim record
 *                   (struct apple_sio_dma_config); ack 0x65
 *   op 6 submit     data = first slot of a transfer: 6 header slots (word 14
 *                   flags, word 15 segment count), then one {addr lo,
 *                   addr hi, length} slot per segment. Ack 0x65, later an
 *                   async 0x68 with parameter bit 7, the same tag and
 *                   data = bytes moved. Commands complete in submit order;
 *                   several may be queued per endpoint.
 *   op 8 stop       ack 0x65; every queued command then completes (0x68)
 *   0x68 without parameter bit 7: channel idle; 0x66: async error
 *
 * A cyclic transfer queues one command per period and resubmits a period as
 * soon as it completes, so the firmware always has the rest of the ring
 * queued. All firmware communication happens in work items with bounded
 * timeouts; a dead coprocessor stalls the stream, never the caller.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/apple/sio.h>
#include <linux/workqueue.h>

#include "virt-dma.h"

#define SIO_DMA_OP_CONFIGURE		0x05
#define SIO_DMA_OP_SUBMIT		0x06
#define SIO_DMA_OP_STOP			0x08
#define SIO_DMA_OP_ACK			0x65
#define SIO_DMA_OP_ASYNC_ERROR		0x66
#define SIO_DMA_OP_COMPLETE		0x68

#define SIO_DMA_MSG_OPCODE		GENMASK_ULL(23, 16)
#define SIO_DMA_MSG_PARAMETER		GENMASK_ULL(31, 24)
#define SIO_DMA_MSG_DATA		GENMASK_ULL(63, 32)

#define SIO_DMA_SLOT_WORDS		3
#define SIO_DMA_HDR_SLOTS		6
#define SIO_DMA_HDR_FLAGS		14	/* word: kIOMemoryRemote, 0 */
#define SIO_DMA_HDR_SEGMENTS		15	/* word: segment count */
#define SIO_DMA_CFG_SLOTS		8
#define SIO_DMA_SEG_BOUNDARY		SZ_4K	/* iOS splits at pages */
#define SIO_DMA_MAX_SEGMENTS		(0x0aaa - SIO_DMA_HDR_SLOTS)
#define SIO_DMA_MAX_PERIODS		16	/* < 31 tags per endpoint */

#define SIO_DMA_CONFIG_TIMEOUT_MS	1000
#define SIO_DMA_STOP_TIMEOUT_MS		200
#define SIO_DMA_DRAIN_TIMEOUT_MS	500
#define SIO_DMA_SUBMIT_RETRIES		3

struct sio_dma_desc;

struct sio_dma_block {
	struct sio_dma_desc *desc;
	u32 first_slot;
	unsigned int slots;
	size_t len;
	bool queued;		/* submitted, completion pending */
};

struct sio_dma_desc {
	struct virt_dma_desc vd;
	struct sio_dma_chan *chan;
	bool cyclic;
	unsigned int next;	/* next block to submit */
	unsigned long done;	/* completed blocks */
	unsigned int nblocks;
	size_t total;
	int status;
	struct sio_dma_block blocks[] __counted_by(nblocks);
};

struct sio_dma_chan {
	struct virt_dma_chan vc;
	struct sio_dma *host;
	u8 endpoint;
	struct apple_sio_dma_config cfg;
	bool cfg_dirty;
	struct sio_dma_desc *active;
	unsigned int inflight;
	bool stopping;
	struct work_struct submit_work;
	struct work_struct stop_work;
	struct completion drained;
	struct completion stopped;
};

struct sio_dma {
	struct device *dev;
	struct apple_sio *sio;
	struct dma_device dma;
	unsigned int nchans;
	struct sio_dma_chan chans[] __counted_by(nchans);
};

/* SPI defaults used by the T8010 touch path when no shim record is given */
static const struct apple_sio_dma_config sio_dma_default_cfg = {
	.fifo = 0x80, .trigger = 0x40, .limit = 0x80,
};

/* SPI0-3, MCA0-4 and UART0-8 TX/RX, unless the SIO node lists endpoints */
static const u8 sio_dma_default_ranges[][2] = {
	{ 0x14, 0x1b }, { 0x28, 0x31 }, { 0x3c, 0x4d },
};

static struct sio_dma_chan *to_sio_chan(struct dma_chan *c)
{
	return container_of(c, struct sio_dma_chan, vc.chan);
}

static struct sio_dma_desc *to_sio_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct sio_dma_desc, vd);
}

/* --- descriptors ----------------------------------------------------------- */

static unsigned int sio_dma_count_segments(dma_addr_t addr, size_t len)
{
	unsigned int n = 0;

	while (len) {
		size_t chunk = min_t(size_t, len, SIO_DMA_SEG_BOUNDARY -
				     (addr & (SIO_DMA_SEG_BOUNDARY - 1)));

		addr += chunk;
		len -= chunk;
		n++;
	}
	return n;
}

static u32 *sio_dma_fill_segments(u32 *seg, dma_addr_t addr, size_t len)
{
	while (len) {
		size_t chunk = min_t(size_t, len, SIO_DMA_SEG_BOUNDARY -
				     (addr & (SIO_DMA_SEG_BOUNDARY - 1)));

		seg[0] = lower_32_bits(addr);
		seg[1] = upper_32_bits(addr);
		seg[2] = chunk;
		seg += SIO_DMA_SLOT_WORDS;
		addr += chunk;
		len -= chunk;
	}
	return seg;
}

/* Allocate a transfer descriptor in the shared slots; returns its segments. */
static u32 *sio_dma_block_alloc(struct sio_dma_chan *chan,
				struct sio_dma_block *b, unsigned int nsegs)
{
	u32 *hdr;

	if (!nsegs || nsegs > SIO_DMA_MAX_SEGMENTS)
		return NULL;

	b->slots = SIO_DMA_HDR_SLOTS + nsegs;
	hdr = apple_sio_desc0_alloc(chan->host->sio, b->slots, &b->first_slot);
	if (!hdr) {
		b->slots = 0;
		return NULL;
	}

	hdr[SIO_DMA_HDR_FLAGS] = 0;
	hdr[SIO_DMA_HDR_SEGMENTS] = nsegs;
	return hdr + SIO_DMA_HDR_SLOTS * SIO_DMA_SLOT_WORDS;
}

static void sio_dma_desc_free(struct virt_dma_desc *vd)
{
	struct sio_dma_desc *desc = to_sio_desc(vd);
	struct apple_sio *sio = desc->chan->host->sio;
	unsigned int i;

	for (i = 0; i < desc->nblocks; i++)
		if (desc->blocks[i].slots)
			apple_sio_desc0_free(sio, desc->blocks[i].first_slot,
					     desc->blocks[i].slots);
	kfree(desc);
}

static struct sio_dma_desc *sio_dma_desc_new(struct sio_dma_chan *chan,
					     unsigned int nblocks, bool cyclic)
{
	struct sio_dma_desc *desc;
	unsigned int i;

	desc = kzalloc_flex(*desc, blocks, nblocks, GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->nblocks = nblocks;
	desc->chan = chan;
	desc->cyclic = cyclic;
	for (i = 0; i < nblocks; i++)
		desc->blocks[i].desc = desc;
	return desc;
}

static bool sio_dma_dir_ok(enum dma_transfer_direction dir)
{
	return dir == DMA_MEM_TO_DEV || dir == DMA_DEV_TO_MEM;
}

static struct dma_async_tx_descriptor *
sio_dma_prep_slave_sg(struct dma_chan *c, struct scatterlist *sgl,
		      unsigned int sg_len, enum dma_transfer_direction dir,
		      unsigned long flags, void *context)
{
	struct sio_dma_chan *chan = to_sio_chan(c);
	struct sio_dma_desc *desc;
	struct scatterlist *sg;
	unsigned int nsegs = 0, i;
	u32 *seg;

	if (!sio_dma_dir_ok(dir) || !sg_len)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		if (!sg_dma_len(sg))
			return NULL;
		nsegs += sio_dma_count_segments(sg_dma_address(sg),
						sg_dma_len(sg));
	}

	desc = sio_dma_desc_new(chan, 1, false);
	if (!desc)
		return NULL;

	seg = sio_dma_block_alloc(chan, &desc->blocks[0], nsegs);
	if (!seg) {
		kfree(desc);
		return NULL;
	}

	for_each_sg(sgl, sg, sg_len, i) {
		seg = sio_dma_fill_segments(seg, sg_dma_address(sg),
					    sg_dma_len(sg));
		desc->total += sg_dma_len(sg);
	}
	desc->blocks[0].len = desc->total;

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

static struct dma_async_tx_descriptor *
sio_dma_prep_cyclic(struct dma_chan *c, dma_addr_t buf, size_t buf_len,
		    size_t period_len, enum dma_transfer_direction dir,
		    unsigned long flags)
{
	struct sio_dma_chan *chan = to_sio_chan(c);
	struct sio_dma_desc *desc;
	unsigned int n, i;

	if (!sio_dma_dir_ok(dir) || !period_len || buf_len % period_len)
		return NULL;

	n = buf_len / period_len;
	if (!n || n > SIO_DMA_MAX_PERIODS)
		return NULL;

	desc = sio_dma_desc_new(chan, n, true);
	if (!desc)
		return NULL;

	for (i = 0; i < n; i++) {
		dma_addr_t addr = buf + (dma_addr_t)i * period_len;
		struct sio_dma_block *b = &desc->blocks[i];
		u32 *seg;

		seg = sio_dma_block_alloc(chan, b,
					  sio_dma_count_segments(addr, period_len));
		if (!seg) {
			sio_dma_desc_free(&desc->vd);
			return NULL;
		}
		sio_dma_fill_segments(seg, addr, period_len);
		b->len = period_len;
	}
	desc->total = buf_len;

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

/* --- firmware interaction (work items) ----------------------------------- */

static int sio_dma_configure(struct sio_dma_chan *chan)
{
	struct apple_sio *sio = chan->host->sio;
	struct apple_sio_dma_config cfg;
	unsigned long flags;
	u32 slot, *rec;
	int ret;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (!chan->cfg_dirty) {
		spin_unlock_irqrestore(&chan->vc.lock, flags);
		return 0;
	}
	cfg = chan->cfg;
	chan->cfg_dirty = false;
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	rec = apple_sio_desc0_alloc(sio, SIO_DMA_CFG_SLOTS, &slot);
	if (!rec) {
		ret = -ENOMEM;
		goto fail;
	}

	/* AppleSmartIODMA::_configureDMA record layout */
	rec[0] = cfg.xfer;
	rec[1] = cfg.timeout;
	rec[2] = cfg.fifo;
	rec[3] = cfg.trigger;
	rec[4] = cfg.limit;
	rec[5] = cfg.word5;
	rec[6] = cfg.frame;
	rec[7] = cfg.word7;
	rec[SIO_DMA_HDR_FLAGS] = 0;
	dma_wmb();

	ret = apple_sio_request(sio, chan->endpoint, SIO_DMA_OP_CONFIGURE, 0,
				slot, SIO_DMA_OP_ACK,
				msecs_to_jiffies(SIO_DMA_CONFIG_TIMEOUT_MS),
				NULL);
	apple_sio_desc0_free(sio, slot, SIO_DMA_CFG_SLOTS);
	if (ret)
		goto fail;

	dev_dbg(chan->host->dev,
		"ep %#x shim: xfer %#x timeout %u fifo %u trigger %u limit %u\n",
		chan->endpoint, cfg.xfer, cfg.timeout, cfg.fifo, cfg.trigger,
		cfg.limit);
	return 0;

fail:
	spin_lock_irqsave(&chan->vc.lock, flags);
	chan->cfg_dirty = true;
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	dev_err_ratelimited(chan->host->dev, "ep %#x: configure failed: %d\n",
			    chan->endpoint, ret);
	return ret;
}

static void sio_dma_block_done(void *data, int status, u64 response)
{
	struct sio_dma_block *b = data;
	struct sio_dma_desc *desc = b->desc;
	struct sio_dma_chan *chan = desc->chan;
	bool kick = false;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	b->queued = false;
	if (!WARN_ON_ONCE(!chan->inflight))
		chan->inflight--;

	if (chan->active == desc && !chan->stopping) {
		if (status) {
			desc->status = status;
			dev_err_ratelimited(chan->host->dev,
					    "ep %#x: transfer failed: %d\n",
					    chan->endpoint, status);
		}
		if (desc->cyclic) {
			if (!status) {
				desc->done++;
				vchan_cyclic_callback(&desc->vd);
				kick = true;
			}
		} else {
			desc->vd.tx_result.result = status ?
				DMA_TRANS_ABORTED : DMA_TRANS_NOERROR;
			desc->vd.tx_result.residue = status ? desc->total : 0;
			vchan_cookie_complete(&desc->vd);
			chan->active = NULL;
			kick = true;
		}
	}

	if (chan->stopping && !chan->inflight)
		complete(&chan->drained);
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	if (kick)
		queue_work(system_highpri_wq, &chan->submit_work);
}

static void sio_dma_submit_work(struct work_struct *work)
{
	struct sio_dma_chan *chan = container_of(work, struct sio_dma_chan,
						 submit_work);
	struct apple_sio *sio = chan->host->sio;
	unsigned int retries = 0;

	if (sio_dma_configure(chan))
		return;

	for (;;) {
		struct apple_sio_async *req;
		struct virt_dma_desc *vd;
		struct sio_dma_desc *desc;
		struct sio_dma_block *b;
		unsigned long flags;
		unsigned int idx;

		spin_lock_irqsave(&chan->vc.lock, flags);
		if (chan->stopping)
			goto unlock_out;
		if (!chan->active) {
			vd = vchan_next_desc(&chan->vc);
			if (!vd)
				goto unlock_out;
			list_del(&vd->node);
			chan->active = to_sio_desc(vd);
		}
		desc = chan->active;
		if (desc->status || (!desc->cyclic && desc->next >= desc->nblocks))
			goto unlock_out;
		idx = desc->next;
		b = &desc->blocks[idx];
		if (b->queued)
			goto unlock_out;
		b->queued = true;
		chan->inflight++;
		desc->next = desc->cyclic ? (idx + 1) % desc->nblocks : idx + 1;
		spin_unlock_irqrestore(&chan->vc.lock, flags);

		dma_wmb();
		req = apple_sio_request_async(sio, chan->endpoint,
					      SIO_DMA_OP_SUBMIT, 0,
					      b->first_slot,
					      SIO_DMA_OP_COMPLETE,
					      sio_dma_block_done, b);
		if (!IS_ERR(req)) {
			/* errors after allocation arrive via the callback */
			apple_sio_async_put(req);
			continue;
		}

		spin_lock_irqsave(&chan->vc.lock, flags);
		b->queued = false;
		chan->inflight--;
		desc->next = idx;
		if (chan->stopping && !chan->inflight)
			complete(&chan->drained);
		spin_unlock_irqrestore(&chan->vc.lock, flags);

		if (++retries > SIO_DMA_SUBMIT_RETRIES) {
			dev_err_ratelimited(chan->host->dev,
					    "ep %#x: submit failed: %ld\n",
					    chan->endpoint, PTR_ERR(req));
			return;
		}
		usleep_range(500, 1000);
		continue;

unlock_out:
		spin_unlock_irqrestore(&chan->vc.lock, flags);
		return;
	}
}

static void sio_dma_stop_work(struct work_struct *work)
{
	struct sio_dma_chan *chan = container_of(work, struct sio_dma_chan,
						 stop_work);
	struct apple_sio *sio = chan->host->sio;
	struct sio_dma_desc *desc;
	unsigned long flags;
	bool busy, issued;
	int ret;

	spin_lock_irqsave(&chan->vc.lock, flags);
	busy = chan->inflight;
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	if (busy) {
		ret = apple_sio_request(sio, chan->endpoint, SIO_DMA_OP_STOP, 0,
					0, SIO_DMA_OP_ACK,
					msecs_to_jiffies(SIO_DMA_STOP_TIMEOUT_MS),
					NULL);
		if (ret)
			dev_warn(chan->host->dev, "ep %#x: stop failed: %d\n",
				 chan->endpoint, ret);

		if (!wait_for_completion_timeout(&chan->drained,
				msecs_to_jiffies(SIO_DMA_DRAIN_TIMEOUT_MS))) {
			dev_warn(chan->host->dev,
				 "ep %#x: %u commands left after stop, cancelling\n",
				 chan->endpoint, READ_ONCE(chan->inflight));
			apple_sio_cancel_endpoint(sio, chan->endpoint);
		}
	}

	spin_lock_irqsave(&chan->vc.lock, flags);
	desc = chan->active;
	chan->active = NULL;
	if (desc)
		vchan_terminate_vdesc(&desc->vd);
	chan->inflight = 0;
	chan->stopping = false;
	issued = !list_empty(&chan->vc.desc_issued);
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	complete_all(&chan->stopped);
	if (issued)
		queue_work(system_highpri_wq, &chan->submit_work);
}

/* --- dmaengine ops ----------------------------------------------------------- */

static void sio_dma_issue_pending(struct dma_chan *c)
{
	struct sio_dma_chan *chan = to_sio_chan(c);
	unsigned long flags;
	bool issued;

	spin_lock_irqsave(&chan->vc.lock, flags);
	issued = vchan_issue_pending(&chan->vc) && !chan->stopping;
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	if (issued)
		queue_work(system_highpri_wq, &chan->submit_work);
}

static int sio_dma_terminate_all(struct dma_chan *c)
{
	struct sio_dma_chan *chan = to_sio_chan(c);
	unsigned long flags;
	bool stop = false;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);
	vchan_get_all_descriptors(&chan->vc, &head);
	if (chan->active && !chan->stopping) {
		chan->stopping = true;
		reinit_completion(&chan->drained);
		reinit_completion(&chan->stopped);
		if (!chan->inflight)
			complete(&chan->drained);
		stop = true;
	}
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	vchan_dma_desc_free_list(&chan->vc, &head);
	if (stop)
		queue_work(system_highpri_wq, &chan->stop_work);
	return 0;
}

static void sio_dma_synchronize(struct dma_chan *c)
{
	struct sio_dma_chan *chan = to_sio_chan(c);

	if (!wait_for_completion_timeout(&chan->stopped,
			msecs_to_jiffies(SIO_DMA_STOP_TIMEOUT_MS +
					 SIO_DMA_DRAIN_TIMEOUT_MS + 100)))
		dev_warn(chan->host->dev, "ep %#x: stop did not finish\n",
			 chan->endpoint);
	vchan_synchronize(&chan->vc);
}

static enum dma_status sio_dma_tx_status(struct dma_chan *c,
					 dma_cookie_t cookie,
					 struct dma_tx_state *state)
{
	struct sio_dma_chan *chan = to_sio_chan(c);
	struct virt_dma_desc *vd;
	enum dma_status status;
	unsigned long flags;
	size_t residue = 0;

	status = dma_cookie_status(c, cookie, state);
	if (status == DMA_COMPLETE || !state)
		return status;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (chan->active && chan->active->vd.tx.cookie == cookie) {
		struct sio_dma_desc *desc = chan->active;

		if (desc->cyclic)
			residue = desc->total - (desc->done % desc->nblocks) *
				  desc->blocks[0].len;
		else
			residue = desc->total;
		if (desc->status)
			status = DMA_ERROR;
	} else {
		vd = vchan_find_desc(&chan->vc, cookie);
		if (vd)
			residue = to_sio_desc(vd)->total;
	}
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	dma_set_residue(state, residue);
	return status;
}

/* AppleSmartIODMA::validFIFODepth */
static u32 sio_dma_valid_depth(const struct apple_sio_dma_config *cfg,
			       u32 req, enum dma_transfer_direction dir)
{
	u32 lo;

	if (dir == DMA_DEV_TO_MEM)
		return req ? min(req, cfg->trigger) : 1;

	lo = min(cfg->trigger, cfg->fifo - cfg->trigger) + 4;
	return req < lo ? lo : min(req, cfg->fifo);
}

static int sio_dma_slave_config(struct dma_chan *c,
				struct dma_slave_config *sc)
{
	struct sio_dma_chan *chan = to_sio_chan(c);
	struct apple_sio_dma_config cfg = sio_dma_default_cfg;
	unsigned long flags;

	if (!sio_dma_dir_ok(sc->direction))
		return -EINVAL;

	if (sc->peripheral_config) {
		if (sc->peripheral_size != sizeof(cfg))
			return -EINVAL;
		memcpy(&cfg, sc->peripheral_config, sizeof(cfg));
		if (cfg.fifo <= cfg.trigger)
			return -EINVAL;
		if (!cfg.limit) {
			if (sc->direction == DMA_MEM_TO_DEV)
				cfg.limit = sc->dst_maxburst * sc->dst_addr_width;
			else
				cfg.limit = sc->src_maxburst * sc->src_addr_width;
		}
		cfg.limit = sio_dma_valid_depth(&cfg, cfg.limit, sc->direction);
	}

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (memcmp(&chan->cfg, &cfg, sizeof(cfg))) {
		chan->cfg = cfg;
		chan->cfg_dirty = true;
	}
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	return 0;
}

static int sio_dma_alloc_chan_resources(struct dma_chan *c)
{
	struct sio_dma_chan *chan = to_sio_chan(c);

	chan->cfg = sio_dma_default_cfg;
	chan->cfg_dirty = true;
	return 0;
}

static void sio_dma_free_chan_resources(struct dma_chan *c)
{
	struct sio_dma_chan *chan = to_sio_chan(c);

	sio_dma_terminate_all(c);
	sio_dma_synchronize(c);
	cancel_work_sync(&chan->submit_work);
	vchan_free_chan_resources(&chan->vc);
}

static struct dma_chan *sio_dma_of_xlate(struct of_phandle_args *spec,
					 struct of_dma *ofdma)
{
	struct sio_dma *host = ofdma->of_dma_data;
	unsigned int i;

	if (spec->args_count != 1)
		return NULL;

	for (i = 0; i < host->nchans; i++)
		if (host->chans[i].endpoint == spec->args[0])
			return dma_get_slave_channel(&host->chans[i].vc.chan);

	return NULL;
}

/* Firmware messages on our endpoints that match no request. */
static void sio_dma_notify(void *data, u64 message)
{
	struct sio_dma_chan *chan = data;
	u8 op = FIELD_GET(SIO_DMA_MSG_OPCODE, message);

	switch (op) {
	case SIO_DMA_OP_ACK:		/* ack of an async submit */
	case SIO_DMA_OP_COMPLETE:	/* idle notification (no tag) */
		break;
	case SIO_DMA_OP_ASYNC_ERROR:
		dev_err_ratelimited(chan->host->dev,
				    "ep %#x: async error %#llx\n",
				    chan->endpoint,
				    FIELD_GET(SIO_DMA_MSG_DATA, message));
		break;
	default:
		dev_dbg(chan->host->dev, "ep %#x: unexpected message %#llx\n",
			chan->endpoint, message);
	}
}

static int sio_dma_endpoints(struct device *dev, u8 *eps, unsigned int max)
{
	struct device_node *np = dev->parent->of_node;
	unsigned int n = 0, i, e;
	int count;

	count = of_property_count_u32_elems(np, "apple,dma-endpoints");
	if (count > 0) {
		for (i = 0; i < count && n < max; i++) {
			u32 ep;

			of_property_read_u32_index(np, "apple,dma-endpoints", i, &ep);
			if (ep < APPLE_SIO_MAX_ENDPOINTS)
				eps[n++] = ep;
		}
		return n;
	}

	for (i = 0; i < ARRAY_SIZE(sio_dma_default_ranges); i++)
		for (e = sio_dma_default_ranges[i][0];
		     e <= sio_dma_default_ranges[i][1] && n < max; e++)
			eps[n++] = e;
	return n;
}

static int sio_dma_probe(struct platform_device *pdev)
{
	struct apple_sio **parent_data = dev_get_platdata(&pdev->dev);
	u8 eps[APPLE_SIO_MAX_ENDPOINTS];
	struct dma_device *dma;
	struct sio_dma *host;
	unsigned int n, i;
	int ret;

	if (!parent_data || !*parent_data || !pdev->dev.parent)
		return -ENODEV;

	n = sio_dma_endpoints(&pdev->dev, eps, ARRAY_SIZE(eps));
	if (!n)
		return -ENODEV;

	host = devm_kzalloc(&pdev->dev, struct_size(host, chans, n), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->nchans = n;
	host->dev = &pdev->dev;
	host->sio = *parent_data;
	platform_set_drvdata(pdev, host);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to set DMA mask\n");

	dma = &host->dma;
	dma_cap_set(DMA_PRIVATE, dma->cap_mask);
	dma_cap_set(DMA_SLAVE, dma->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma->cap_mask);
	dma->dev = &pdev->dev;
	dma->device_alloc_chan_resources = sio_dma_alloc_chan_resources;
	dma->device_free_chan_resources = sio_dma_free_chan_resources;
	dma->device_prep_slave_sg = sio_dma_prep_slave_sg;
	dma->device_prep_dma_cyclic = sio_dma_prep_cyclic;
	dma->device_issue_pending = sio_dma_issue_pending;
	dma->device_tx_status = sio_dma_tx_status;
	dma->device_terminate_all = sio_dma_terminate_all;
	dma->device_synchronize = sio_dma_synchronize;
	dma->device_config = sio_dma_slave_config;
	dma->directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	dma->residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
	dma->src_addr_widths = dma->dst_addr_widths =
		BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	INIT_LIST_HEAD(&dma->channels);

	for (i = 0; i < n; i++) {
		struct sio_dma_chan *chan = &host->chans[i];

		chan->host = host;
		chan->endpoint = eps[i];
		chan->cfg = sio_dma_default_cfg;
		chan->cfg_dirty = true;
		INIT_WORK(&chan->submit_work, sio_dma_submit_work);
		INIT_WORK(&chan->stop_work, sio_dma_stop_work);
		init_completion(&chan->drained);
		init_completion(&chan->stopped);
		complete_all(&chan->stopped);
		vchan_init(&chan->vc, dma);
		chan->vc.desc_free = sio_dma_desc_free;
		apple_sio_set_notify(host->sio, chan->endpoint, sio_dma_notify,
				     chan);
	}

	ret = dma_async_device_register(dma);
	if (ret)
		goto err_notify;

	ret = of_dma_controller_register(pdev->dev.parent->of_node,
					 sio_dma_of_xlate, host);
	if (ret) {
		dma_async_device_unregister(dma);
		goto err_notify;
	}

	dev_info(&pdev->dev, "%u SmartIO DMA endpoints\n", n);
	return 0;

err_notify:
	for (i = 0; i < n; i++)
		apple_sio_set_notify(host->sio, host->chans[i].endpoint, NULL,
				     NULL);
	return dev_err_probe(&pdev->dev, ret, "failed to register DMA device\n");
}

static void sio_dma_remove(struct platform_device *pdev)
{
	struct sio_dma *host = platform_get_drvdata(pdev);
	unsigned int i;

	of_dma_controller_free(pdev->dev.parent->of_node);
	dma_async_device_unregister(&host->dma);
	for (i = 0; i < host->nchans; i++) {
		struct sio_dma_chan *chan = &host->chans[i];

		sio_dma_terminate_all(&chan->vc.chan);
		sio_dma_synchronize(&chan->vc.chan);
		cancel_work_sync(&chan->submit_work);
		cancel_work_sync(&chan->stop_work);
		apple_sio_set_notify(host->sio, chan->endpoint, NULL, NULL);
		vchan_free_chan_resources(&chan->vc);
	}
}

static struct platform_driver sio_dma_driver = {
	.driver = {
		.name = "apple-sio-dma",
	},
	.probe = sio_dma_probe,
	.remove = sio_dma_remove,
};
module_platform_driver(sio_dma_driver);

MODULE_ALIAS("platform:apple-sio-dma");
MODULE_DESCRIPTION("Apple SmartIO DMA controller");
MODULE_LICENSE("Dual MIT/GPL");

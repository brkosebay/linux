// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A9 (s8000/s8003) Secure Enclave mailbox driver.
 *
 * The SEP is an ASC coprocessor: registers at 0x20DA00000, a 32-bit mailbox
 * at +0x4000 and four GIC interrupts (ADT "interrupts" = 99 98 101 100).
 * The AP and the SEP exchange 64-bit little-endian messages:
 *
 *	bits  0..7	destination endpoint
 *	bits  8..15	tag
 *	bits 16..23	opcode
 *	bits 24..31	parameter
 *	bits 32..63	data
 *
 * sepOS announces its services on endpoint 0xfd: op 0 carries the endpoint's
 * 4CC, op 1 its OOL (out-of-line buffer) page-count ranges. The driver only
 * listens; it never asks for, guesses or probes an endpoint. A service is
 * usable once both halves of its advertisement are known, either from the
 * mailbox or declared by hand from a pongoOS capture.
 *
 * A usable endpoint gets a send and a receive OOL buffer, published with four
 * control messages on endpoint 0 ('cntl'). Commands then use tag 0xfc with the
 * command in the opcode field; payloads live in the OOL buffers. See
 * docs/sep/driver-notes.md and docs/sep/linux-bringup-next.md.
 *
 * Nothing is ever sent automatically: arming the mailbox, publishing buffers
 * and every command are explicit debugfs writes.
 *
 * Arming activates the AppleA7IOPV2 wrapper the way AppleA7IOP::start does
 * (docs/sep/xart-handshake.md section 6), but only while the SEP PMGR word is
 * what pongoOS handed over (0x4f0000ff; AUTO_ENABLE is only warned about):
 * the driver never writes PMGR, CPU RUN, NMI or reset. Every AP->SEP message
 * is one 64-bit store to +0x4010, as in AppleA7IOPV2::postMailbox. Probe and
 * arm log the PMGR word and SEP +0x28/+0x44/+0xb88 read-only, for comparison
 * with pongoOS peeks. debugfs wake-test runs XNU's endpoint-0 nap wake once
 * (section 7).
 *
 * SEP->AP messages are popped with the matching single 64-bit load from
 * +0x4038, but only after recv_sts bit 17 says that the FIFO is nonempty.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <asm/cacheflush.h>

#include <uapi/linux/apple-sep.h>

#define SEP_MBOX_OFFSET		0x4000
#define SEP_MBOX_SIZE		0x1000

/*
 * Mailbox register layout (pongoOS struct mailbox_registers32), except that
 * the AP->SEP message is sent the way AppleA7IOPV2::postMailbox
 * (0xfffffff005b4e4a8) does: one 64-bit store to +0x4010, not pongoOS's two
 * 32-bit stores to +0x4010/+0x4014. AppleA7IOPV2's receive primitive
 * (0xfffffff005b4e498) uses one 64-bit load from +0x4038.
 */
#define SEP_MBOX_DIS_INT	0x00
#define SEP_MBOX_EN_INT		0x04
#define SEP_MBOX_SND_STS	0x08
#define SEP_MBOX_SND		0x10	/* 64-bit, written with writeq() only */
#define SEP_MBOX_RECV_STS	0x20
#define SEP_MBOX_RECV		0x38	/* 64-bit, read with readq() only */

/*
 * Status bits, as in every Apple mailbox variant: bit 16 full, bit 17 empty.
 * recv_sts bit 17 clear => a message is waiting (pongoOS sep_check_mailbox);
 * snd_sts bit 16 set => the AP->SEP FIFO has no room.
 */
#define SEP_MBOX_STS_FULL	0x10000
#define SEP_MBOX_STS_EMPTY	0x20000
/* pongoOS enables exactly this interrupt bit after probe */
#define SEP_MBOX_INT_ENABLE	0x1000
#define SEP_TX_TIMEOUT_US	100000
/* transport proof: queued AP->SEP words must be consumed within 1 s */
#define SEP_DRAIN_TIMEOUT_US	1000000

/*
 * AppleA7IOPV2 wrapper, offsets from the SEP base (not the mailbox). The
 * interrupt mask has a set/clear pair in two views, primary +0x4000/+0x4004
 * (= DIS_INT/EN_INT above) and mirror +0xc00/+0xc04; reads of either register
 * of a pair return the mask. +0x4008 and +0x4020 are snd_sts and recv_sts.
 */
#define SEP_WRAP_MASK_SET	0x4000
#define SEP_WRAP_MASK_CLR	0x4004
#define SEP_WRAP_MIRROR_SET	0x0c00
#define SEP_WRAP_MIRROR_CLR	0x0c04
#define SEP_WRAP_INBOX_CTL	0x4008
#define SEP_WRAP_OUTBOX_CTL	0x4020
#define SEP_WRAP_MASK_ALL	0x1111
#define SEP_WRAP_INBOX_IRQ	0x0001	/* bit 0: AP->SEP (inbox) notification */
#define SEP_WRAP_OUTBOX_IRQ	0x1000	/* bit 12: SEP->AP (outbox) notification */
#define SEP_WRAP_OUTBOX_EN	0x0001	/* AppleA7IOPV2_setOutboxEnable */
#define SEP_WRAP_NREGS		6

/* PMGR power-state word, as in drivers/pmdomain/apple/pmgr-pwrstate.c */
#define SEP_PS_RESET		BIT(31)
#define SEP_PS_AUTO_ENABLE	BIT(28)
#define SEP_PS_ACTUAL		GENMASK(7, 4)
#define SEP_PS_TARGET		GENMASK(3, 0)
#define SEP_PS_ACTIVE		0xf
/*
 * The word pongoOS leaves after booting sepOS ("PMGR sep inherited
 * reg=4f0000ff", 15:42). ps_sep carries apple,no-auto-pm so Linux keeps it.
 * AUTO_ENABLE (bit 28) is no longer suspected: at 16:18 a word stayed queued
 * under pongoOS with the word at 0x4f0000ff. Arm and wake-test therefore only
 * warn about bit 28, and refuse any other difference from this value.
 */
#define SEP_PS_HANDOFF		0x4f0000ff

/*
 * SEP block registers snapshotted read-only at probe and arm, in this order:
 * +0x28, +0x44 (CPU control, RUN), +0xb88, then snd_sts and recv_sts.
 */
#define SEP_CORE_NREGS		5

/* Endpoint-0 nap wake (docs/sep/xart-handshake.md section 7) */
#define SEP_OP_NAP_WAKE		0x0e	/* AP: raw 0x00000000000e0000, twice */
#define SEP_OP_NAP_AWAKE	0x0f	/* SEP: wake indication */
#define SEP_OP_OS_ACTIVE	0x15	/* SEP: wake indication, also OS active */
#define SEP_WAKE_WAIT_US	1000000

#define SEP_LOG_ENTRIES		256

#define SEP_MSG_EP(ep)		((u64)(ep) & 0xff)
#define SEP_MSG_TAG(tag)	(((u64)(tag) & 0xff) << 8)
#define SEP_MSG_OP(op)		(((u64)(op) & 0xff) << 16)
#define SEP_MSG_PARAM(param)	(((u64)(param) & 0xff) << 24)
#define SEP_MSG_DATA(data)	(((u64)(data)) << 32)
#define SEP_MSG(ep, tag, op, param, data) \
	(SEP_MSG_EP(ep) | SEP_MSG_TAG(tag) | SEP_MSG_OP(op) | \
	 SEP_MSG_PARAM(param) | SEP_MSG_DATA(data))

#define SEP_MSG_GET_EP(msg)	((u8)((msg) & 0xff))
#define SEP_MSG_GET_TAG(msg)	((u8)(((msg) >> 8) & 0xff))
#define SEP_MSG_GET_OP(msg)	((u8)(((msg) >> 16) & 0xff))
#define SEP_MSG_GET_PARAM(msg)	((u8)(((msg) >> 24) & 0xff))
#define SEP_MSG_GET_DATA(msg)	((u32)((msg) >> 32))

/* The whole 8-bit endpoint namespace; 0xfd..0xff are transport endpoints. */
#define SEP_EP_COUNT		256
#define SEP_EP_DISCOVERY	0xfd
#define SEP_EP_CLIENT_MAX	0xfc

/* Endpoint 0xfd (AppleSEPDiscovery): param = endpoint ID. */
#define SEP_DISC_NAME		0x00	/* data = 4CC, 0x636e746c = 'cntl' */
#define SEP_DISC_OOL		0x01	/* data bytes LSB first: in min/max, out min/max */

/* OOL ranges are counted in 4 KiB SEP pages. "in" is AP->SEP (send). */
#define SEP_OOL_PAGE		0x1000

/* Control-endpoint opcodes that publish an endpoint OOL buffer. */
#define SEP_OP_SET_SEND_ADDR	0x02
#define SEP_OP_SET_RECV_ADDR	0x03
#define SEP_OP_SET_SEND_SIZE	0x04
#define SEP_OP_SET_RECV_SIZE	0x05

/* Endpoint transfer framing (docs/sep/sbio-commands.md §1). */
#define SEP_TAG_XFER		0xfc	/* request, and final completion */
#define SEP_TAG_XFER_CONT	0xfd	/* continuation chunk */
#define SEP_TAG_XFER_ACK	0xfe	/* SEP: send the next chunk */
#define SEP_TAG_XFER_ERR	0xff	/* SEP: error, status word in recv+0x10 */
#define SEP_GT_VERSION		1
#define SEP_GT_VERSION_OFF	0x00
#define SEP_GT_TOTAL_OFF	0x04
#define SEP_GT_OFFSET_OFF	0x08
#define SEP_GT_FLAGS_OFF	0x0c
#define SEP_GT_RESERVED_OFF	0x10
#define SEP_GT_COMMAND_OFF	0x14
#define SEP_GT_CHUNK_OFF	0x18
#define SEP_GT_PAYLOAD_OFF	0x1c
#define SEP_XFER_ERR_OFF	0x10
#define SEP_XFER_CHUNK		(0x4000 - SEP_GT_PAYLOAD_OFF)
#define SEP_XFER_TIMEOUT_MS	10000

#define SEP_CMD_SBIO_INIT	0x73

/* AppleSEPARTStorageService on the advertised 'arts' endpoint. */
#define SEP_NAME_ARTS		0x61727473	/* 'arts', big-endian 4CC */
#define SEP_ARTS_OOL_SIZE	0x4000		/* j71t advertises 4 pages */
#define SEP_ARTS_TAG		0x01		/* first XNU tag-pool entry + 1 */
#define SEP_ARTS_OP_MANIFEST	0x06		/* Opcode::MANIFEST (3) << 1 */
#define SEP_ARTS_OP_REPLY	0x07		/* request opcode | 1 */
#define SEP_ARTS_TIMEOUT_MS	10000
#define SEP_ARTS_DER_MIN	10

enum sep_arts_result {
	SEP_ARTS_NEVER,
	SEP_ARTS_WAITING,
	SEP_ARTS_REPLY,
	SEP_ARTS_TIMEOUT,
};

struct sep_ep {
	/* advertisement */
	u32 name;
	u8 in_min, in_max, out_min, out_max;	/* 4 KiB pages */
	bool has_name;
	bool has_ool;
	bool name_manual;	/* declared through debugfs advertise, */
	bool ool_manual;	/* not (yet) seen on the mailbox */
	bool conflict;		/* a later mailbox advertisement disagreed */

	/* OOL buffers, kept until the driver goes away once published */
	bool published;
	void *send;
	void *recv;
	phys_addr_t send_pa;
	phys_addr_t recv_pa;
	size_t send_len;
	size_t recv_len;
};

/* PMGR word and, only while it says on and out of reset, SEP core registers */
struct sep_core_snap {
	bool taken;
	int pmgr_ret;
	u32 pmgr;
	bool valid;			/* reg[] was read */
	u32 reg[SEP_CORE_NREGS];
};

struct sep_a9 {
	struct device *dev;
	void __iomem *base;
	void __iomem *mbox;

	struct mutex lock;		/* receive side: log, endpoint table */
	struct mutex tx_lock;		/* AP->SEP FIFO */
	struct mutex cfg_lock;		/* debugfs configuration and transfers */
	u64 log[SEP_LOG_ENTRIES];
	unsigned int log_head;
	unsigned int log_count;
	unsigned long last_rx;		/* jiffies */

	struct delayed_work poll;
	unsigned int poll_us;
	bool running;

	struct sep_ep *eps;		/* SEP_EP_COUNT entries */

	/* one transfer at a time, under cfg_lock */
	struct completion xfer_done;
	bool xfer_waiting;
	u8 xfer_ep;
	u8 xfer_cmd;
	u8 xfer_tag;			/* tag of the message that ended it */
	u8 xfer_op;
	u32 xfer_data;

	int sbio_ep;			/* -1 = unset */
	bool sbio_ready;		/* 0x73 acknowledged on sbio_ep */

	/* last debugfs xfer, for reading back */
	int last_ep;
	u8 last_cmd;
	int last_ret;
	u32 last_status;
	size_t last_out_len;
	u32 last_gt_version;
	u32 last_gt_total;
	u32 last_gt_offset;
	u32 last_gt_flags;
	u32 last_gt_reserved;
	u32 last_gt_command;
	u32 last_gt_chunk;
	u32 last_gt_mailbox_data;
	u8 last_out[SEP_XFER_CHUNK];

	/* One-shot AppleSEPARTStorage MANIFEST request, under cfg_lock. */
	struct completion arts_done;
	enum sep_arts_result arts_result;
	bool arts_ran;
	bool arts_waiting;
	int arts_ep;
	size_t arts_manifest_len;
	u8 arts_reply_tag;
	u8 arts_reply_op;
	u8 arts_reply_param;
	u32 arts_reply_data;
	u64 arts_reply_raw;
	int arts_ret;
	s64 arts_reply_us;
	ktime_t arts_t0;

	/* last wrapper activation (debugfs start), under cfg_lock */
	bool armed_once;
	u32 pwrstate;
	u32 wrap_before[SEP_WRAP_NREGS];
	u32 wrap_after[SEP_WRAP_NREGS];
	u32 outbox_read;		/* +0x4020 before the RMW */

	/* read-only snapshots: probe, then around the last arm (cfg_lock) */
	struct sep_core_snap probe_snap;
	struct sep_core_snap arm_snap;
	struct sep_core_snap arm_after_snap;

	/* nap-wake diagnostic, once per boot */
	bool wake_waiting;		/* under lock: sep_wake_rx records */
	bool wake_seen;			/* under lock */
	u64 wake_msg;			/* first ep 0 op 0x0f/0x15 */
	ktime_t wake_at;
	unsigned int wake_ind;		/* indications seen */
	unsigned int wake_other;	/* any other message in the window */
	u64 wake_other_first;
	bool wake_ran;			/* the rest under cfg_lock */
	int wake_ret;
	ktime_t wake_t0;		/* first word posted */
	u32 wake_pmgr;
	u32 wake_pmgr_after;		/* re-read at the end */
	int wake_pmgr_after_ret;
	u32 wake_sts[3];		/* snd_sts: before, after word 1, at the end */
	bool wake_w1_drained;
	bool wake_w2_posted;
	bool wake_w2_drained;
	s64 wake_w1_us;
	s64 wake_w2_us;

	/* last AP->SEP drain check (ool, send), under cfg_lock */
	char drain_what[32];
	int drain_ret;
	u32 drain_sts_before;
	u32 drain_sts_after;
	s64 drain_us;

	struct dentry *dbg;
	u64 tx_count;
	u64 rx_count;
	u64 tx_busy;
	u64 dropped;
};

/* Wrapper registers snapshotted around arming, in this order. */
static const u16 sep_wrap_regs[SEP_WRAP_NREGS] = {
	SEP_WRAP_MASK_SET, SEP_WRAP_MASK_CLR, SEP_WRAP_MIRROR_SET,
	SEP_WRAP_MIRROR_CLR, SEP_WRAP_INBOX_CTL, SEP_WRAP_OUTBOX_CTL,
};

static void sep_wrap_snapshot(struct sep_a9 *sep, u32 *v)
{
	unsigned int i;

	for (i = 0; i < SEP_WRAP_NREGS; i++)
		v[i] = readl(sep->base + sep_wrap_regs[i]);
}

static int sep_wrap_print(char *buf, size_t size, const u32 *v)
{
	return scnprintf(buf, size,
			 "set4000=%08x clr4004=%08x setc00=%08x clrc04=%08x snd4008=%08x rcv4020=%08x",
			 v[0], v[1], v[2], v[3], v[4], v[5]);
}

static const u16 sep_core_regs[SEP_CORE_NREGS] = {
	0x28, 0x44, 0xb88, SEP_WRAP_INBOX_CTL, SEP_WRAP_OUTBOX_CTL,
};

static int sep_core_print(char *buf, size_t size, const struct sep_core_snap *s)
{
	if (!s->taken)
		return scnprintf(buf, size, "not taken");
	if (s->pmgr_ret)
		return scnprintf(buf, size, "pmgr=read error %d, SEP not read",
				 s->pmgr_ret);
	if (!s->valid)
		return scnprintf(buf, size, "pmgr=%08x (not on), SEP not read",
				 s->pmgr);
	return scnprintf(buf, size,
			 "pmgr=%08x sep28=%08x sep44=%08x sepb88=%08x snd4008=%08x rcv4020=%08x",
			 s->pmgr, s->reg[0], s->reg[1], s->reg[2], s->reg[3],
			 s->reg[4]);
}

/* snd_sts/recv_sts: bit 17 empty, bit 16 full */
static const char *sep_fifo_state(u32 sts)
{
	if (sts & SEP_MBOX_STS_EMPTY)
		return "empty";
	return sts & SEP_MBOX_STS_FULL ? "full" : "not empty";
}

/*
 * Pop exactly one SEP->AP word. The caller must first observe recv_sts bit 17
 * clear. noinline keeps this primitive visible for the packaged-vmlinux
 * instruction audit: one 64-bit load from mbox + 0x38 and no access to +0x34.
 */
static noinline u64 sep_read_msg(struct sep_a9 *sep)
{
	return readq(sep->mbox + SEP_MBOX_RECV);
}

static void sep_log_push(struct sep_a9 *sep, u64 msg)
{
	unsigned int slot = (sep->log_head + sep->log_count) % SEP_LOG_ENTRIES;

	sep->log[slot] = msg;
	if (sep->log_count < SEP_LOG_ENTRIES)
		sep->log_count++;
	else
		sep->log_head = (sep->log_head + 1) % SEP_LOG_ENTRIES;

	sep->rx_count++;
	sep->last_rx = jiffies;
}

/* The 4CC is stored most significant byte first; decode only printable names. */
static bool sep_fourcc(u32 word, char name[5])
{
	int i;

	for (i = 0; i < 4; i++) {
		name[i] = (char)(word >> (24 - 8 * i));
		if (name[i] < 0x20 || name[i] > 0x7e)
			return false;
	}
	name[4] = '\0';
	return true;
}

static void sep_ool_word(u32 word, u8 *in_min, u8 *in_max, u8 *out_min,
			 u8 *out_max)
{
	*in_min = word & 0xff;
	*in_max = (word >> 8) & 0xff;
	*out_min = (word >> 16) & 0xff;
	*out_max = word >> 24;
}

static bool sep_ep_complete(const struct sep_ep *e)
{
	return e->has_name && e->has_ool;
}

/* Record one 0xfd message. Called with sep->lock held. */
static void sep_discovery_rx(struct sep_a9 *sep, u64 msg)
{
	u8 id = SEP_MSG_GET_PARAM(msg);
	u32 data = SEP_MSG_GET_DATA(msg);
	struct sep_ep *e = &sep->eps[id];
	u8 a, b, c, d;
	char name[5];

	switch (SEP_MSG_GET_OP(msg)) {
	case SEP_DISC_NAME:
		if (!sep_fourcc(data, name))
			strscpy(name, "????");
		if (e->has_name && e->name != data) {
			e->conflict = true;
			dev_warn(sep->dev, "ep %#04x: name %08x conflicts with %08x\n",
				 id, data, e->name);
			return;
		}
		e->name = data;
		e->has_name = true;
		e->name_manual = false;
		dev_info(sep->dev, "ep %#04x: advertised '%s'\n", id, name);
		break;
	case SEP_DISC_OOL:
		sep_ool_word(data, &a, &b, &c, &d);
		if (e->has_ool && (e->in_min != a || e->in_max != b ||
				   e->out_min != c || e->out_max != d)) {
			e->conflict = true;
			dev_warn(sep->dev, "ep %#04x: OOL advertisement %08x conflicts\n",
				 id, data);
			return;
		}
		e->in_min = a;
		e->in_max = b;
		e->out_min = c;
		e->out_max = d;
		e->has_ool = true;
		e->ool_manual = false;
		dev_info(sep->dev, "ep %#04x: OOL in [%u,%u] out [%u,%u] pages%s\n",
			 id, a, b, c, d, e->has_name ? "" : " (no name yet)");
		break;
	default:
		dev_warn(sep->dev, "ep 0xfd: unknown op %#x\n", SEP_MSG_GET_OP(msg));
		break;
	}
}

/* Does this message end the pending transfer? Called with sep->lock held. */
static void sep_xfer_rx(struct sep_a9 *sep, u64 msg)
{
	u8 tag = SEP_MSG_GET_TAG(msg);
	u8 op = SEP_MSG_GET_OP(msg);

	if (!sep->xfer_waiting || SEP_MSG_GET_EP(msg) != sep->xfer_ep)
		return;

	switch (tag) {
	case SEP_TAG_XFER:
	case SEP_TAG_XFER_CONT:
	case SEP_TAG_XFER_ACK:
		if (op != sep->xfer_cmd)
			return;
		break;
	case SEP_TAG_XFER_ERR:
		break;
	default:
		return;
	}

	sep->xfer_tag = tag;
	sep->xfer_op = op;
	sep->xfer_data = SEP_MSG_GET_DATA(msg);
	sep->xfer_waiting = false;
	complete(&sep->xfer_done);
}

/* Record the one expected arts MANIFEST response. Called with lock held. */
static void sep_arts_rx(struct sep_a9 *sep, u64 msg)
{
	if (!sep->arts_waiting || SEP_MSG_GET_EP(msg) != sep->arts_ep ||
	    SEP_MSG_GET_TAG(msg) != SEP_ARTS_TAG ||
	    SEP_MSG_GET_OP(msg) != SEP_ARTS_OP_REPLY)
		return;

	sep->arts_reply_tag = SEP_MSG_GET_TAG(msg);
	sep->arts_reply_op = SEP_MSG_GET_OP(msg);
	sep->arts_reply_param = SEP_MSG_GET_PARAM(msg);
	sep->arts_reply_data = SEP_MSG_GET_DATA(msg);
	sep->arts_reply_raw = msg;
	sep->arts_reply_us = ktime_us_delta(ktime_get(), sep->arts_t0);
	sep->arts_result = SEP_ARTS_REPLY;
	sep->arts_waiting = false;
	complete(&sep->arts_done);
}

/*
 * During a wake test: ep 0 op 0x0f or 0x15 is the indication the XNU mailbox
 * drain callback wakes its waiter for. Anything else is counted, first one
 * kept. Called with sep->lock held.
 */
static void sep_wake_rx(struct sep_a9 *sep, u64 msg)
{
	u8 op = SEP_MSG_GET_OP(msg);

	if (!sep->wake_waiting)
		return;

	if (SEP_MSG_GET_EP(msg) == 0 &&
	    (op == SEP_OP_NAP_AWAKE || op == SEP_OP_OS_ACTIVE)) {
		if (!sep->wake_ind++) {
			sep->wake_msg = msg;
			sep->wake_at = ktime_get();
			sep->wake_seen = true;
		}
		return;
	}
	if (!sep->wake_other++)
		sep->wake_other_first = msg;
}

/* Verdict of the last wake test. Called with cfg_lock held. */
static const char *sep_wake_result(const struct sep_a9 *sep)
{
	if (!sep->wake_ran)
		return "never";
	if (sep->wake_ret == -ETIMEDOUT)
		return "word1-stuck";
	if (sep->wake_seen)
		return "wake";
	if (sep->wake_ret == -ENODATA)
		return "no-wake";
	return "error";
}

/*
 * Drain everything the SEP queued. The status bit is the same handshake
 * pongoOS uses; there is no separate acknowledge on the AP side.
 */
static int sep_drain(struct sep_a9 *sep)
{
	unsigned int drained = 0;

	mutex_lock(&sep->lock);
	while (!(readl(sep->mbox + SEP_MBOX_RECV_STS) & SEP_MBOX_STS_EMPTY)) {
		u64 msg = sep_read_msg(sep);

		sep_log_push(sep, msg);
		sep_wake_rx(sep, msg);
		if (SEP_MSG_GET_EP(msg) == SEP_EP_DISCOVERY)
			sep_discovery_rx(sep, msg);
		else {
			sep_arts_rx(sep, msg);
			sep_xfer_rx(sep, msg);
		}

		if (++drained > SEP_LOG_ENTRIES) {
			sep->dropped++;
			break;
		}
	}
	mutex_unlock(&sep->lock);

	return drained;
}

static void sep_poll_work(struct work_struct *work)
{
	struct sep_a9 *sep = container_of(to_delayed_work(work),
					  struct sep_a9, poll);

	if (!sep->running)
		return;

	sep_drain(sep);
	schedule_delayed_work(&sep->poll, usecs_to_jiffies(sep->poll_us));
}

/*
 * Send one message once the AP->SEP FIFO has room. The mailbox must be armed:
 * nothing touches the SEP registers before that.
 *
 * This is the only AP->SEP write in the driver (ool, xfer, send and wake-test
 * all come here): a single 64-bit writeq() of low | high << 32 to +0x4010,
 * then mb() (dsb sy, as pongoOS "sep msg64" does, so both probes issue the
 * same sequence). XNU's postMailbox is one 64-bit store; tri53-sep..sep3 and
 * pongoOS wrote the two halves with two 32-bit stores, and on 2026-10-26 such
 * a word was left unconsumed even under pongoOS (16:18). noinline keeps one
 * store site in the object code, so the instruction can be checked.
 */
static noinline int sep_send_raw(struct sep_a9 *sep, u8 ep, u8 tag, u8 op,
				 u8 param, u32 data)
{
	u64 msg = SEP_MSG(ep, tag, op, param, data);
	u32 sts;
	int ret;

	if (!sep->running)
		return -ENXIO;

	mutex_lock(&sep->tx_lock);
	ret = readl_poll_timeout(sep->mbox + SEP_MBOX_SND_STS, sts,
				 !(sts & SEP_MBOX_STS_FULL), 100,
				 SEP_TX_TIMEOUT_US);
	if (ret) {
		sep->tx_busy++;
		dev_warn(sep->dev, "tx FIFO full (snd_sts=%#x), not sent: %016llx\n",
			 sts, msg);
	} else {
		writeq(msg, sep->mbox + SEP_MBOX_SND);
		mb();
		sep->tx_count++;
	}
	mutex_unlock(&sep->tx_lock);

	return ret ? -EBUSY : 0;
}

/*
 * Wait up to 1 s for the AP->SEP FIFO to empty (snd_sts bit 17): the transport
 * proof that sepOS consumed what was just queued. It only reports; nothing is
 * resent. 'before' is snd_sts read before queueing. Called with cfg_lock held
 * on an armed mailbox.
 */
static __printf(3, 4) int sep_tx_drain(struct sep_a9 *sep, u32 before,
				       const char *fmt, ...)
{
	ktime_t start = ktime_get();
	va_list args;
	u32 sts;
	int ret;

	ret = readl_poll_timeout(sep->mbox + SEP_MBOX_SND_STS, sts,
				 sts & SEP_MBOX_STS_EMPTY, 100,
				 SEP_DRAIN_TIMEOUT_US);

	va_start(args, fmt);
	vscnprintf(sep->drain_what, sizeof(sep->drain_what), fmt, args);
	va_end(args);
	sep->drain_ret = ret;
	sep->drain_sts_before = before;
	sep->drain_sts_after = sts;
	sep->drain_us = ktime_us_delta(ktime_get(), start);

	if (ret)
		dev_warn(sep->dev, "%s: AP->SEP FIFO not drained after %lld us (snd_sts %#x -> %#x): transport FAIL\n",
			 sep->drain_what, sep->drain_us, before, sts);
	else
		dev_info(sep->dev, "%s: AP->SEP FIFO drained in %lld us (snd_sts %#x -> %#x): transport pass\n",
			 sep->drain_what, sep->drain_us, before, sts);
	return ret;
}

/*
 * Publish an endpoint's OOL buffers through 'cntl' (endpoint 0, the target
 * endpoint in param): send size (0x04), send address (0x02), receive size
 * (0x05), receive address (0x03). Sizes in bytes, addresses as PA >> 12, as
 * AppleSEPManager::setSendOOLBuffer/setReceiveOOLBuffer do.
 */
static int sep_ool_publish(struct sep_a9 *sep, u8 ep, struct sep_ep *e)
{
	int ret;

	ret = sep_send_raw(sep, 0, 0, SEP_OP_SET_SEND_SIZE, ep, e->send_len);
	if (!ret)
		ret = sep_send_raw(sep, 0, 0, SEP_OP_SET_SEND_ADDR, ep,
				   e->send_pa >> 12);
	if (!ret)
		ret = sep_send_raw(sep, 0, 0, SEP_OP_SET_RECV_SIZE, ep,
				   e->recv_len);
	if (!ret)
		ret = sep_send_raw(sep, 0, 0, SEP_OP_SET_RECV_ADDR, ep,
				   e->recv_pa >> 12);
	return ret;
}

static bool sep_ool_size_ok(size_t len, u8 min, u8 max)
{
	return len && IS_ALIGNED(len, SEP_OOL_PAGE) &&
	       len >= (size_t)min * SEP_OOL_PAGE &&
	       len <= (size_t)max * SEP_OOL_PAGE;
}

/*
 * Allocate and publish an endpoint's send and receive buffers, sized within
 * its advertised ranges. Physically contiguous and page aligned: the SEP
 * reads AP DRAM directly (no DART), so the physical address is handed over
 * as-is. The AP->SEP FIFO must be empty before the four control words, and
 * must drain within 1 s after them (-ETIMEDOUT otherwise: stop, no retry).
 * Called with cfg_lock held.
 */
static int sep_ep_alloc(struct sep_a9 *sep, u8 ep, size_t send_len,
			size_t recv_len)
{
	struct sep_ep *e = &sep->eps[ep];
	struct sep_ep adv;
	void *send, *recv;
	u32 before;
	int ret;

	if (ep > SEP_EP_CLIENT_MAX)
		return -EINVAL;

	mutex_lock(&sep->lock);
	adv = *e;
	mutex_unlock(&sep->lock);

	if (adv.published)
		return -EBUSY;	/* sepOS keeps the first binding */
	if (!sep_ep_complete(&adv))
		return -ENOENT;	/* no complete advertisement for ep */
	if (!sep_ool_size_ok(send_len, adv.in_min, adv.in_max) ||
	    !sep_ool_size_ok(recv_len, adv.out_min, adv.out_max))
		return -ERANGE;

	/* Words still queued would only pile up behind the new ones. */
	before = readl(sep->mbox + SEP_MBOX_SND_STS);
	if (!(before & SEP_MBOX_STS_EMPTY)) {
		dev_warn(sep->dev, "ep %#04x: AP->SEP FIFO not empty (snd_sts=%#x), OOL not published, nothing sent\n",
			 ep, before);
		return -EBUSY;
	}

	send = alloc_pages_exact(PAGE_ALIGN(send_len), GFP_KERNEL | __GFP_ZERO);
	recv = alloc_pages_exact(PAGE_ALIGN(recv_len), GFP_KERNEL | __GFP_ZERO);
	if (!send || !recv) {
		ret = -ENOMEM;
		goto err_free;
	}

	/*
	 * Write the zeroes back and drop the lines: the SEP must not read
	 * stale data, and no dirty line may later overwrite a SEP write.
	 */
	dcache_clean_inval_poc((unsigned long)send,
			       (unsigned long)send + PAGE_ALIGN(send_len));
	dcache_clean_inval_poc((unsigned long)recv,
			       (unsigned long)recv + PAGE_ALIGN(recv_len));

	mutex_lock(&sep->lock);
	e->send = send;
	e->recv = recv;
	e->send_pa = virt_to_phys(send);
	e->recv_pa = virt_to_phys(recv);
	e->send_len = send_len;
	e->recv_len = recv_len;
	mutex_unlock(&sep->lock);

	ret = sep_ool_publish(sep, ep, e);
	/*
	 * Once any control message went out the SEP may hold these addresses,
	 * so the pages are never freed after a partial publish either.
	 */
	mutex_lock(&sep->lock);
	e->published = true;
	mutex_unlock(&sep->lock);

	dev_info(sep->dev, "ep %#04x: OOL send %pa len %#zx recv %pa len %#zx%s\n",
		 ep, &e->send_pa, send_len, &e->recv_pa, recv_len,
		 ret ? " (publish incomplete)" : "");
	if (!ret)
		ret = sep_tx_drain(sep, before, "ool ep %#04x", ep);
	return ret;

err_free:
	if (send)
		free_pages_exact(send, PAGE_ALIGN(send_len));
	if (recv)
		free_pages_exact(recv, PAGE_ALIGN(recv_len));
	return ret;
}

/*
 * Run one single-chunk endpoint command (docs/sep/sbio-commands.md §1):
 *
 *	AP:  ep=E tag=0xfc op=cmd param=0 data=sequence (zero here)
 *	SEP: ep=E tag=0xfc op=cmd param=0 data=sequence (zero here)
 *
 * Both OOL buffers contain AppleSEPGenericTransfer's seven-u32 little-endian
 * header: version, total length, offset, flags, reserved, command and chunk
 * length. Payload starts at +0x1c. A tag 0xff reply is an error with its
 * status word at receive OOL offset 0x10. The 0xfd/0xfe chunk sequence is not
 * implemented: a transfer that needs it fails with -EMSGSIZE or -EPROTO.
 * Called with cfg_lock held.
 */
static int sep_transfer(struct sep_a9 *sep, u8 ep, u8 cmd,
			const void *in, size_t inlen, void *out, size_t outlen,
			size_t *replylen, u32 *status)
{
	struct sep_ep *e = &sep->eps[ep];
	unsigned long left;
	u32 version, total, offset, flags, reserved, command, chunk;
	int ret;

	*replylen = 0;
	*status = 0;
	sep->last_gt_version = 0;
	sep->last_gt_total = 0;
	sep->last_gt_offset = 0;
	sep->last_gt_flags = 0;
	sep->last_gt_reserved = 0;
	sep->last_gt_command = 0;
	sep->last_gt_chunk = 0;
	sep->last_gt_mailbox_data = 0;

	if (ep > SEP_EP_CLIENT_MAX || !e->published)
		return -ENODEV;
	if (inlen > SEP_XFER_CHUNK || SEP_GT_PAYLOAD_OFF + inlen > e->send_len)
		return -EMSGSIZE;

	memset(e->send, 0, SEP_GT_PAYLOAD_OFF);
	put_unaligned_le32(SEP_GT_VERSION, e->send + SEP_GT_VERSION_OFF);
	put_unaligned_le32(inlen, e->send + SEP_GT_TOTAL_OFF);
	put_unaligned_le32(0, e->send + SEP_GT_OFFSET_OFF);
	put_unaligned_le32(0, e->send + SEP_GT_FLAGS_OFF);
	put_unaligned_le32(0, e->send + SEP_GT_RESERVED_OFF);
	put_unaligned_le32(cmd, e->send + SEP_GT_COMMAND_OFF);
	put_unaligned_le32(inlen, e->send + SEP_GT_CHUNK_OFF);
	if (inlen)
		memcpy(e->send + SEP_GT_PAYLOAD_OFF, in, inlen);
	dcache_clean_poc((unsigned long)e->send,
			 (unsigned long)e->send + SEP_GT_PAYLOAD_OFF + inlen);

	mutex_lock(&sep->lock);
	reinit_completion(&sep->xfer_done);
	sep->xfer_ep = ep;
	sep->xfer_cmd = cmd;
	sep->xfer_tag = 0;
	sep->xfer_waiting = true;
	mutex_unlock(&sep->lock);

	ret = sep_send_raw(sep, ep, SEP_TAG_XFER, cmd, 0, 0);
	if (ret)
		goto out_cancel;

	left = wait_for_completion_timeout(&sep->xfer_done,
					   msecs_to_jiffies(SEP_XFER_TIMEOUT_MS));
	if (!left) {
		ret = -ETIMEDOUT;
		goto out_cancel;
	}
	sep->last_gt_mailbox_data = sep->xfer_data;

	switch (sep->xfer_tag) {
	case SEP_TAG_XFER:
		break;
	case SEP_TAG_XFER_ERR:
		dcache_inval_poc((unsigned long)e->recv,
				 (unsigned long)e->recv + SEP_XFER_ERR_OFF + 4);
		*status = get_unaligned_le32(e->recv + SEP_XFER_ERR_OFF);
		dev_warn(sep->dev, "ep %#04x cmd %#x: SEP error op=%#x status %#x\n",
			 ep, cmd, sep->xfer_op, *status);
		return -EIO;
	default:
		dev_warn(sep->dev, "ep %#04x cmd %#x: chunk tag %#x not supported\n",
			 ep, cmd, sep->xfer_tag);
		return -EPROTO;
	}

	if (sep->xfer_data)
		return -EPROTO;	/* only first/single-chunk sequence zero is supported */

	dcache_inval_poc((unsigned long)e->recv,
			 (unsigned long)e->recv + SEP_GT_PAYLOAD_OFF);
	version = get_unaligned_le32(e->recv + SEP_GT_VERSION_OFF);
	total = get_unaligned_le32(e->recv + SEP_GT_TOTAL_OFF);
	offset = get_unaligned_le32(e->recv + SEP_GT_OFFSET_OFF);
	flags = get_unaligned_le32(e->recv + SEP_GT_FLAGS_OFF);
	reserved = get_unaligned_le32(e->recv + SEP_GT_RESERVED_OFF);
	command = get_unaligned_le32(e->recv + SEP_GT_COMMAND_OFF);
	chunk = get_unaligned_le32(e->recv + SEP_GT_CHUNK_OFF);

	sep->last_gt_version = version;
	sep->last_gt_total = total;
	sep->last_gt_offset = offset;
	sep->last_gt_flags = flags;
	sep->last_gt_reserved = reserved;
	sep->last_gt_command = command;
	sep->last_gt_chunk = chunk;
	if (version != SEP_GT_VERSION || offset || command != cmd ||
	    total != chunk)
		return -EPROTO;
	*replylen = total;
	if (chunk > SEP_XFER_CHUNK || SEP_GT_PAYLOAD_OFF + chunk > e->recv_len ||
	    total > outlen)
		return -EMSGSIZE;	/* continuation replies are not implemented */

	dcache_inval_poc((unsigned long)e->recv,
			 (unsigned long)e->recv + SEP_GT_PAYLOAD_OFF + chunk);
	if (chunk)
		memcpy(out, e->recv + SEP_GT_PAYLOAD_OFF, chunk);
	dev_info(sep->dev,
		 "ep %#04x cmd %#x: GT reply v=%u total=%u offset=%u flags=%#x reserved=%#x command=%#x chunk=%u mailbox_data=%#x\n",
		 ep, cmd, version, total, offset, flags, reserved, command, chunk,
		 sep->xfer_data);
	return chunk;

out_cancel:
	mutex_lock(&sep->lock);
	sep->xfer_waiting = false;
	mutex_unlock(&sep->lock);
	return ret;
}

static ssize_t sep_dbg_status_read(struct file *file, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int i, complete = 0, published = 0;
	char *buf;
	ssize_t ret;

	buf = kzalloc(1024, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&sep->lock);
	for (i = 0; i < SEP_EP_COUNT; i++) {
		complete += sep_ep_complete(&sep->eps[i]);
		published += sep->eps[i].published;
	}
	mutex_unlock(&sep->lock);

	if (sep->running) {
		u32 rsts = readl(sep->mbox + SEP_MBOX_RECV_STS);
		u32 ssts = readl(sep->mbox + SEP_MBOX_SND_STS);

		ret = scnprintf(buf, 1024,
				"base=%px mbox=%px running=1\n"
				"recv_sts=%#x (%s) snd_sts=%#x (%s) en_int=%#x\n",
				sep->base, sep->mbox,
				rsts, rsts & SEP_MBOX_STS_EMPTY ? "empty" : "pending",
				ssts, sep_fifo_state(ssts),
				readl(sep->mbox + SEP_MBOX_EN_INT));
	} else {
		ret = scnprintf(buf, 1024,
				"base=%px mbox=%px running=0 (mailbox untouched)\n",
				sep->base, sep->mbox);
	}
	ret += scnprintf(buf + ret, 1024 - ret,
			 "tx=%llu rx=%llu tx_busy=%llu dropped=%llu log=%u\n"
			 "rx_idle_ms=%u endpoints_complete=%u published=%u\n"
			 "sbio_ep=%d sbio_ready=%d\n",
			 sep->tx_count, sep->rx_count, sep->tx_busy, sep->dropped,
			 sep->log_count,
			 sep->rx_count ? jiffies_to_msecs(jiffies - sep->last_rx) : 0,
			 complete, published, sep->sbio_ep, sep->sbio_ready);

	ret = simple_read_from_buffer(ubuf, count, ppos, buf, ret);
	kfree(buf);
	return ret;
}

static const struct file_operations sep_dbg_status_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sep_dbg_status_read,
};

static ssize_t sep_dbg_log_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	size_t size = SEP_LOG_ENTRIES * 112;
	unsigned int i;
	char *buf;
	ssize_t len = 0;

	buf = kvzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&sep->lock);
	for (i = 0; i < sep->log_count; i++) {
		u64 msg = sep->log[(sep->log_head + i) % SEP_LOG_ENTRIES];
		u32 data = SEP_MSG_GET_DATA(msg);
		u8 a, b, c, d;
		char name[5];

		len += scnprintf(buf + len, size - len,
				 "ep=%02x tag=%02x op=%02x param=%02x data=%08x",
				 SEP_MSG_GET_EP(msg), SEP_MSG_GET_TAG(msg),
				 SEP_MSG_GET_OP(msg), SEP_MSG_GET_PARAM(msg),
				 data);
		if (SEP_MSG_GET_EP(msg) == SEP_EP_DISCOVERY &&
		    SEP_MSG_GET_OP(msg) == SEP_DISC_NAME &&
		    sep_fourcc(data, name)) {
			len += scnprintf(buf + len, size - len,
					 "  [ep %#04x '%s']",
					 SEP_MSG_GET_PARAM(msg), name);
		} else if (SEP_MSG_GET_EP(msg) == SEP_EP_DISCOVERY &&
			   SEP_MSG_GET_OP(msg) == SEP_DISC_OOL) {
			sep_ool_word(data, &a, &b, &c, &d);
			len += scnprintf(buf + len, size - len,
					 "  [ep %#04x OOL in %u-%u out %u-%u pages]",
					 SEP_MSG_GET_PARAM(msg), a, b, c, d);
		}
		len += scnprintf(buf + len, size - len, "\n");
	}
	mutex_unlock(&sep->lock);

	len = simple_read_from_buffer(ubuf, count, ppos, buf, len);
	kvfree(buf);
	return len;
}

static const struct file_operations sep_dbg_log_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sep_dbg_log_read,
};

static const char *sep_ep_source(const struct sep_ep *e)
{
	if (e->name_manual && e->ool_manual)
		return "manual";
	if (e->name_manual || e->ool_manual)
		return "mixed";
	return "mailbox";
}

static const char *sep_ep_state(const struct sep_ep *e)
{
	if (!e->has_ool)
		return "incomplete: no OOL message";
	if (!e->has_name)
		return "incomplete: no name message";
	if (e->published)
		return "published";
	if (!e->in_max && !e->out_max)
		return "complete, no OOL";
	return "complete";
}

/* Endpoint table, one line per endpoint the driver knows anything about. */
static ssize_t sep_dbg_endpoints_read(struct file *file, char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	size_t size = SEP_EP_COUNT * 192;
	unsigned int i;
	char *buf;
	ssize_t len = 0;

	buf = kvzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, size - len,
			 "# id   name  src     in_pages out_pages state\n");
	mutex_lock(&sep->lock);
	for (i = 0; i < SEP_EP_COUNT; i++) {
		struct sep_ep *e = &sep->eps[i];
		char name[5] = "-";
		char in[8] = "-", out[8] = "-";

		if (!e->has_name && !e->has_ool)
			continue;
		if (e->has_name && !sep_fourcc(e->name, name))
			snprintf(name, sizeof(name), "????");
		if (e->has_ool) {
			snprintf(in, sizeof(in), "%u-%u", e->in_min, e->in_max);
			snprintf(out, sizeof(out), "%u-%u", e->out_min, e->out_max);
		}
		len += scnprintf(buf + len, size - len,
				 "0x%02x %-5s %-7s %-8s %-9s %s",
				 i, name, sep_ep_source(e), in, out,
				 sep_ep_state(e));
		if (e->conflict)
			len += scnprintf(buf + len, size - len, " CONFLICT");
		if (sep_ep_complete(e) && !e->published && (e->in_max || e->out_max))
			len += scnprintf(buf + len, size - len,
					 " max_send=%#x max_recv=%#x",
					 e->in_max * SEP_OOL_PAGE,
					 e->out_max * SEP_OOL_PAGE);
		if (e->published)
			len += scnprintf(buf + len, size - len,
					 " send=%pa/%#zx recv=%pa/%#zx",
					 &e->send_pa, e->send_len,
					 &e->recv_pa, e->recv_len);
		len += scnprintf(buf + len, size - len, "\n");
	}
	mutex_unlock(&sep->lock);

	len = simple_read_from_buffer(ubuf, count, ppos, buf, len);
	kvfree(buf);
	return len;
}

static const struct file_operations sep_dbg_endpoints_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sep_dbg_endpoints_read,
};

/*
 * Declare an advertisement by hand, from a pongoOS log of the 0xfd messages
 * (all numbers hex):
 *
 *	echo "ID NAME IN_MIN IN_MAX OUT_MIN OUT_MAX" > advertise
 *	echo "raw ID NAME_WORD OOL_WORD" > advertise
 *
 * e.g. "10 xars 8 8 8 8" or, copying the two data words of
 * "ep=fd op=0 param=10 data=78617273" / "ep=fd op=1 param=10 data=8080808",
 * "raw 10 78617273 8080808". This only fills the table; nothing is sent.
 * A value the mailbox already delivered can't be overridden.
 */
static ssize_t sep_dbg_advertise_write(struct file *file,
				       const char __user *ubuf, size_t count,
				       loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int id, word, ool, v[4];
	u8 a, b, c, d;
	char buf[96], str[8];
	struct sep_ep *e;
	int ret = 0, i;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "raw %x %x %x", &id, &word, &ool) == 3) {
		if (!sep_fourcc(word, str))
			return -EINVAL;
	} else if (sscanf(buf, "%x %7s %x %x %x %x", &id, str, &v[0], &v[1],
			  &v[2], &v[3]) == 6) {
		if (strlen(str) != 4)
			return -EINVAL;
		word = 0;
		for (i = 0; i < 4; i++) {
			if (str[i] < 0x20 || str[i] > 0x7e || v[i] > 0xff)
				return -EINVAL;
			word = word << 8 | (u8)str[i];
		}
		ool = v[0] | v[1] << 8 | v[2] << 16 | v[3] << 24;
	} else {
		return -EINVAL;
	}
	if (id > SEP_EP_CLIENT_MAX)
		return -EINVAL;
	sep_ool_word(ool, &a, &b, &c, &d);

	mutex_lock(&sep->lock);
	e = &sep->eps[id];
	if (e->published) {
		ret = -EBUSY;
	} else if ((e->has_name && !e->name_manual && e->name != word) ||
		   (e->has_ool && !e->ool_manual &&
		    (e->in_min != a || e->in_max != b ||
		     e->out_min != c || e->out_max != d))) {
		ret = -EEXIST;	/* disagrees with what the mailbox delivered */
	} else {
		if (!e->has_name || e->name_manual) {
			e->name = word;
			e->has_name = true;
			e->name_manual = true;
		}
		if (!e->has_ool || e->ool_manual) {
			e->in_min = a;
			e->in_max = b;
			e->out_min = c;
			e->out_max = d;
			e->has_ool = true;
			e->ool_manual = true;
		}
	}
	mutex_unlock(&sep->lock);

	if (ret)
		return ret;
	dev_info(sep->dev, "ep %#04x: declared '%s' OOL in [%u,%u] out [%u,%u] pages\n",
		 id, str, a, b, c, d);
	return count;
}

static const struct file_operations sep_dbg_advertise_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = sep_dbg_advertise_write,
};

/*
 * Raw bring-up interface:
 *	echo "ep tag op param data" > send	(all hex, one message)
 *	send a message to an endpoint without any framing assumptions
 * After a successful write the AP->SEP FIFO is watched for up to 1 s and the
 * result logged (dmesg, debugfs fifo); the write itself still succeeds, since
 * the word is queued either way.
 */
static ssize_t sep_dbg_send_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int ep, tag, op, param, data;
	char buf[64];
	u32 before;
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%x %x %x %x %x", &ep, &tag, &op, &param, &data) != 5)
		return -EINVAL;

	mutex_lock(&sep->cfg_lock);
	if (!sep->running) {
		ret = -ENXIO;
	} else {
		before = readl(sep->mbox + SEP_MBOX_SND_STS);
		ret = sep_send_raw(sep, ep, tag, op, param, data);
		if (!ret)
			sep_tx_drain(sep, before, "send ep %#04x tag %#04x",
				     ep & 0xff, tag & 0xff);
	}
	mutex_unlock(&sep->cfg_lock);
	return ret ? ret : count;
}

static const struct file_operations sep_dbg_send_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = sep_dbg_send_write,
};

/*
 * OOL buffer bring-up:
 *	echo "EP SEND_SIZE RECV_SIZE" > ool	(all hex, e.g. "10 8000 8000")
 * The endpoint needs a complete advertisement and each size must be a 4 KiB
 * multiple inside its advertised range ("in" = send, "out" = receive). The
 * buffers are published through 'cntl' and kept for the lifetime of the
 * driver; sepOS accepts one binding per endpoint. The write fails with
 * -ETIMEDOUT if the four control words are not consumed within 1 s.
 */
static ssize_t sep_dbg_ool_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int ep, send_len, recv_len;
	char buf[64];
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%x %x %x", &ep, &send_len, &recv_len) != 3 ||
	    ep > SEP_EP_CLIENT_MAX)
		return -EINVAL;

	mutex_lock(&sep->cfg_lock);
	ret = sep->running ? sep_ep_alloc(sep, ep, send_len, recv_len) : -ENXIO;
	mutex_unlock(&sep->cfg_lock);
	return ret ? ret : count;
}

static const struct file_operations sep_dbg_ool_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = sep_dbg_ool_write,
};

static int sep_find_named_ep(struct sep_a9 *sep, u32 name)
{
	unsigned int i;
	int found = -1;

	mutex_lock(&sep->lock);
	for (i = 0; i <= SEP_EP_CLIENT_MAX; i++) {
		if (!sep_ep_complete(&sep->eps[i]) || sep->eps[i].name != name)
			continue;
		if (found >= 0) {
			found = -EEXIST;
			break;
		}
		found = i;
	}
	mutex_unlock(&sep->lock);
	return found;
}

/*
 * Binary write of the IM4M DER into the already-published arts AP->SEP OOL
 * buffer.  It only stages bytes; arts-start is the separate commit point.
 */
static ssize_t sep_dbg_arts_manifest_write(struct file *file,
					   const char __user *ubuf, size_t count,
					   loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	struct sep_ep *e;
	loff_t off = *ppos;
	int ep, ret = 0;

	if (!count)
		return 0;

	mutex_lock(&sep->cfg_lock);
	if (!sep->running) {
		ret = -ENXIO;
		goto out;
	}
	if (sep->arts_ran) {
		ret = -EALREADY;
		goto out;
	}
	ep = sep_find_named_ep(sep, SEP_NAME_ARTS);
	if (ep < 0) {
		ret = ep == -1 ? -ENOENT : ep;
		goto out;
	}
	e = &sep->eps[ep];
	if (!e->published || !e->send || e->send_len != SEP_ARTS_OOL_SIZE) {
		ret = -ENODEV;
		goto out;
	}
	if (off < 0 || off != sep->arts_manifest_len ||
	    count > e->send_len - off) {
		ret = -EINVAL;
		goto out;
	}
	if (copy_from_user(e->send + off, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}
	dcache_clean_poc((unsigned long)e->send + off,
			 (unsigned long)e->send + off + count);
	sep->arts_manifest_len += count;
	*ppos += count;
	dev_info(sep->dev, "arts ep %#04x: staged IM4M bytes %#zx/%#zx\n",
		 ep, sep->arts_manifest_len, e->send_len);
out:
	mutex_unlock(&sep->cfg_lock);
	return ret ? ret : count;
}

static const struct file_operations sep_dbg_arts_manifest_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = sep_dbg_arts_manifest_write,
};

static int sep_arts_manifest_start(struct sep_a9 *sep)
{
	struct sep_ep *e;
	unsigned long left;
	u16 der_content;
	u32 before;
	int ep, ret;

	if (!sep->running)
		return -ENXIO;
	if (sep->arts_ran)
		return -EALREADY;
	ep = sep_find_named_ep(sep, SEP_NAME_ARTS);
	if (ep < 0)
		return ep == -1 ? -ENOENT : ep;
	e = &sep->eps[ep];
	if (!e->published || !e->send || e->send_len != SEP_ARTS_OOL_SIZE)
		return -ENODEV;
	if (sep->arts_manifest_len < SEP_ARTS_DER_MIN)
		return -ENODATA;
	if (((u8 *)e->send)[0] != 0x30 || ((u8 *)e->send)[1] != 0x82 ||
	    memcmp(e->send + 4, "\x16\x04IM4M", 6))
		return -EBADMSG;
	der_content = get_unaligned_be16(e->send + 2);
	if ((size_t)der_content + 4 != sep->arts_manifest_len)
		return -EMSGSIZE;

	before = readl(sep->mbox + SEP_MBOX_SND_STS);
	if (!(before & SEP_MBOX_STS_EMPTY))
		return -EBUSY;

	/* This request is one-shot even on timeout: never resend in this boot. */
	sep->arts_ran = true;
	sep->arts_ep = ep;
	sep->arts_reply_tag = 0;
	sep->arts_reply_op = 0;
	sep->arts_reply_param = 0xff;
	sep->arts_reply_data = 0;
	sep->arts_reply_raw = 0;
	sep->arts_reply_us = 0;
	sep->arts_ret = 0;
	sep->arts_result = SEP_ARTS_WAITING;
	reinit_completion(&sep->arts_done);
	mutex_lock(&sep->lock);
	sep->arts_waiting = true;
	mutex_unlock(&sep->lock);
	dcache_clean_poc((unsigned long)e->send,
			 (unsigned long)e->send + sep->arts_manifest_len);

	dev_info(sep->dev, "arts ep %#04x: sending MANIFEST tag=%#x op=%#x, IM4M %#zx bytes\n",
		 ep, SEP_ARTS_TAG, SEP_ARTS_OP_MANIFEST,
		 sep->arts_manifest_len);
	sep->arts_t0 = ktime_get();
	ret = sep_send_raw(sep, ep, SEP_ARTS_TAG, SEP_ARTS_OP_MANIFEST, 0, 0);
	if (!ret)
		ret = sep_tx_drain(sep, before, "arts MANIFEST");
	if (ret)
		goto cancel;

	left = wait_for_completion_timeout(&sep->arts_done,
					   msecs_to_jiffies(SEP_ARTS_TIMEOUT_MS));
	if (!left) {
		ret = -ETIMEDOUT;
		sep->arts_result = SEP_ARTS_TIMEOUT;
		goto cancel;
	}

	ret = sep->arts_reply_param ? -EIO : 0;
	sep->arts_ret = ret;
	dev_info(sep->dev, "arts ep %#04x: MANIFEST reply raw=%016llx param=%#x data=%#x after %lld us: %s\n",
		 ep, sep->arts_reply_raw, sep->arts_reply_param,
		 sep->arts_reply_data, sep->arts_reply_us,
		 ret ? "SEP rejected" : "success");
	return ret;

cancel:
	mutex_lock(&sep->lock);
	sep->arts_waiting = false;
	mutex_unlock(&sep->lock);
	sep->arts_ret = ret;
	dev_warn(sep->dev, "arts ep %#04x: MANIFEST stopped: %d (no retry in this boot)\n",
		 ep, ret);
	return ret;
}

static ssize_t sep_dbg_arts_start_write(struct file *file,
					const char __user *ubuf, size_t count,
					loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	char buf[8];
	unsigned int on;
	int ret;

	if (count >= sizeof(buf) || copy_from_user(buf, ubuf, count))
		return -EINVAL;
	buf[count] = '\0';
	if (kstrtouint(strim(buf), 0, &on) || on != 1)
		return -EINVAL;
	mutex_lock(&sep->cfg_lock);
	ret = sep_arts_manifest_start(sep);
	mutex_unlock(&sep->cfg_lock);
	return ret ? ret : count;
}

static ssize_t sep_dbg_arts_start_read(struct file *file, char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	char buf[320];
	ssize_t len;

	mutex_lock(&sep->cfg_lock);
	len = scnprintf(buf, sizeof(buf),
			"ran=%d result=%u ret=%d ep=%d manifest_len=%zu\n"
			"reply tag=%02x op=%02x param=%02x data=%08x raw=%016llx after_us=%lld\n",
			sep->arts_ran, sep->arts_result, sep->arts_ret,
			sep->arts_ep, sep->arts_manifest_len,
			sep->arts_reply_tag, sep->arts_reply_op,
			sep->arts_reply_param, sep->arts_reply_data,
			sep->arts_reply_raw, sep->arts_reply_us);
	mutex_unlock(&sep->cfg_lock);
	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations sep_dbg_arts_start_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sep_dbg_arts_start_read,
	.write = sep_dbg_arts_start_write,
};

static int sep_pwrstate_read(u32 *val);

/*
 * cat fifo: both FIFO status words decoded, the four mask views, the current
 * PMGR word, the last wrapper activation (PMGR word, registers before and
 * after), the last AP->SEP drain check, the read-only probe/arm snapshots and
 * the wake-test verdict. Unarmed, no SEP register is read: only the PMGR word
 * and the recorded values are shown.
 */
static ssize_t sep_dbg_fifo_read(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	const size_t size = 2048;
	u32 v[SEP_WRAP_NREGS], ps;
	ssize_t len = 0;
	char *buf;
	int ret;

	buf = kzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&sep->cfg_lock);
	if (sep->running) {
		sep_wrap_snapshot(sep, v);
		len += scnprintf(buf + len, size - len,
				 "running=1\nap->sep snd_sts=%#x (%s)\nsep->ap recv_sts=%#x (%s)\nnow   ",
				 v[4], sep_fifo_state(v[4]),
				 v[5], sep_fifo_state(v[5]));
		len += sep_wrap_print(buf + len, size - len, v);
		len += scnprintf(buf + len, size - len, "\n");
	} else {
		len += scnprintf(buf + len, size - len,
				 "running=0 (mailbox untouched)\n");
	}

	ret = sep_pwrstate_read(&ps);
	if (ret)
		len += scnprintf(buf + len, size - len,
				 "pmgr now: read error %d\n", ret);
	else
		len += scnprintf(buf + len, size - len,
				 "pmgr now=%08x (%s %08x)\n", ps,
				 ps == SEP_PS_HANDOFF ? "handoff" :
				 (ps & ~SEP_PS_AUTO_ENABLE) == SEP_PS_HANDOFF ?
				 "handoff+AUTO" : "NOT handoff",
				 SEP_PS_HANDOFF);

	if (sep->armed_once) {
		len += scnprintf(buf + len, size - len,
				 "arm pwrstate=%08x actual=%lx target=%lx\nbefore ",
				 sep->pwrstate,
				 FIELD_GET(SEP_PS_ACTUAL, sep->pwrstate),
				 FIELD_GET(SEP_PS_TARGET, sep->pwrstate));
		len += sep_wrap_print(buf + len, size - len, sep->wrap_before);
		len += scnprintf(buf + len, size - len,
				 "\nrmw4020 read=%08x wrote=%08x\nafter  ",
				 sep->outbox_read,
				 sep->outbox_read | SEP_WRAP_OUTBOX_EN);
		len += sep_wrap_print(buf + len, size - len, sep->wrap_after);
		len += scnprintf(buf + len, size - len, "\n");
	} else {
		len += scnprintf(buf + len, size - len, "arm: never\n");
	}

	if (sep->drain_what[0])
		len += scnprintf(buf + len, size - len,
				 "drain %s: %s after %lld us, snd_sts %#x -> %#x\n",
				 sep->drain_what,
				 sep->drain_ret ? "FAIL (not drained)" : "pass",
				 sep->drain_us, sep->drain_sts_before,
				 sep->drain_sts_after);
	else
		len += scnprintf(buf + len, size - len, "drain: none yet\n");

	len += scnprintf(buf + len, size - len, "probe      ");
	len += sep_core_print(buf + len, size - len, &sep->probe_snap);
	len += scnprintf(buf + len, size - len, "\narm before ");
	len += sep_core_print(buf + len, size - len, &sep->arm_snap);
	len += scnprintf(buf + len, size - len, "\narm after  ");
	len += sep_core_print(buf + len, size - len, &sep->arm_after_snap);
	len += scnprintf(buf + len, size - len, "\nwake-test: %s\n",
			 sep_wake_result(sep));
	mutex_unlock(&sep->cfg_lock);

	len = simple_read_from_buffer(ubuf, count, ppos, buf, len);
	kfree(buf);
	return len;
}

static const struct file_operations sep_dbg_fifo_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sep_dbg_fifo_read,
};


/*
 * /dev/sep-bio: the ABI the Aurora Silicon libfprint driver speaks
 * (include/uapi/linux/apple-sep.h, interface version 4). Keeping the same
 * interface means their libfprint `aurora` driver, fprintd and Omarchy's
 * fingerprint UI work unchanged later. Until the SBIO state machine and the
 * real EnrollMode/MatchMode payloads exist, this interface never sends a
 * message: every SEP command goes through debugfs.
 */
static struct sep_a9 *sep_a9_instance;

static int sep_bio_open(struct inode *inode, struct file *file)
{
	file->private_data = sep_a9_instance;
	return 0;
}

static long sep_bio_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct sep_a9 *sep = file->private_data;
	struct sep_bio_info info = {
		.version = SEP_BIO_IFACE_VERSION,
		.sensor_present = 0,
		.enrolled = 0,
		.capacity = 0,
		.enroll_stages = 0,
	};
	struct sep_bio_list *list;

	if (!sep)
		return -ENODEV;

	switch (cmd) {
	case SEP_BIO_GET_INFO:
		/*
		 * sensor_present stays 0 until SBIO has acknowledged 0x73: the
		 * sensor only exists once sepOS has brought it up, and libfprint
		 * correctly declines to expose a reader without one.
		 */
		info.sensor_present = sep->sbio_ready;
		if (copy_to_user((void __user *)arg, &info, sizeof(info)))
			return -EFAULT;
		return 0;
	case SEP_BIO_LIST:
		/* too large for the stack, and empty until SBIO is implemented */
		list = kzalloc(sizeof(*list), GFP_KERNEL);
		if (!list)
			return -ENOMEM;
		if (copy_to_user((void __user *)arg, list, sizeof(*list))) {
			kfree(list);
			return -EFAULT;
		}
		kfree(list);
		return 0;
	case SEP_BIO_ENROL_POLL: {
		struct sep_bio_enrol_poll poll = {
			.state = SEP_BIO_STATE_IDLE,
		};

		if (copy_to_user((void __user *)arg, &poll, sizeof(poll)))
			return -EFAULT;
		return 0;
	}
	case SEP_BIO_VERIFY_POLL: {
		struct sep_bio_verify_poll poll = {
			.state = SEP_BIO_STATE_IDLE,
			.result = SEP_BIO_NOT_COMPARED,
		};

		if (copy_to_user((void __user *)arg, &poll, sizeof(poll)))
			return -EFAULT;
		return 0;
	}
	case SEP_BIO_ENROL_START:
	case SEP_BIO_VERIFY_START:
		/*
		 * EnrollMode (0x03) and MatchMode (0x25) need the real 68-byte
		 * payloads built by biometrickitd, plus sensor frames; zeroed
		 * payloads prove nothing (docs/sep/linux-bringup-next.md §4).
		 */
	case SEP_BIO_CANCEL:
	case SEP_BIO_DELETE:
	case SEP_BIO_DELETE_ALL:
	case SEP_BIO_ATTEST:
		return -ENOSYS;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations sep_bio_fops = {
	.owner = THIS_MODULE,
	.open = sep_bio_open,
	.unlocked_ioctl = sep_bio_ioctl,
	.llseek = noop_llseek,
};

static struct miscdevice sep_bio_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sep-bio",
	.mode = 0600,
	.fops = &sep_bio_fops,
};


/*
 * Read, never write, the SEP's PMGR power-state word: the apple,pmgr child
 * labelled "sep" (ps_sep, apple,always-on), through the syscon regmap the
 * pmgr-pwrstate driver uses. This is the hardware state behind the "sep"
 * line of pm_genpd_summary.
 */
static int sep_pwrstate_read(u32 *val)
{
	struct device_node *pmgr, *np = NULL, *child;
	struct regmap *map;
	const char *label;
	u32 off;
	int ret;

	for_each_compatible_node(pmgr, NULL, "apple,pmgr") {
		for_each_child_of_node(pmgr, child) {
			if (of_device_is_compatible(child, "apple,pmgr-pwrstate") &&
			    !of_property_read_string(child, "label", &label) &&
			    !strcmp(label, "sep")) {
				np = child;
				break;
			}
		}
		if (np)
			break;
	}
	if (!np)
		return -ENODEV;

	ret = of_property_read_u32(np, "reg", &off);
	if (!ret) {
		map = syscon_node_to_regmap(pmgr);
		ret = IS_ERR(map) ? PTR_ERR(map) : regmap_read(map, off, val);
	}
	of_node_put(np);
	of_node_put(pmgr);
	return ret;
}

/*
 * Gate for arm and wake-test on a PMGR word just read: refuse (-EIO, nothing
 * touched) unless the domain is on and out of reset and the word equals the
 * handoff value apart from AUTO_ENABLE; AUTO_ENABLE alone is a warning and is
 * never repaired.
 */
static int sep_pwrstate_gate(struct sep_a9 *sep, u32 ps, const char *what)
{
	if (FIELD_GET(SEP_PS_ACTUAL, ps) != SEP_PS_ACTIVE || (ps & SEP_PS_RESET)) {
		dev_err(sep->dev, "%s refused: sep power domain not on (pwrstate=%08x actual=%lx target=%lx%s), SEP untouched\n",
			what, ps, FIELD_GET(SEP_PS_ACTUAL, ps),
			FIELD_GET(SEP_PS_TARGET, ps),
			ps & SEP_PS_RESET ? " reset" : "");
		return -EIO;
	}
	if ((ps & ~SEP_PS_AUTO_ENABLE) != SEP_PS_HANDOFF) {
		dev_err(sep->dev, "%s refused: pwrstate=%08x differs from the pongoOS handoff word %08x beyond AUTO_ENABLE, SEP untouched\n",
			what, ps, SEP_PS_HANDOFF);
		return -EIO;
	}
	if (ps & SEP_PS_AUTO_ENABLE)
		dev_warn(sep->dev, "%s: PMGR sep AUTO_ENABLE (bit 28) set (pwrstate=%08x), not repaired, continuing\n",
			 what, ps);
	return 0;
}

/*
 * Read-only snapshot, the counterpart of pongoOS "peek": the PMGR word, then
 * SEP +0x28, +0x44, +0xb88, +0x4008 and +0x4020, but only if the PMGR word
 * says actual on and no reset (a read of the powered-down block can stall).
 */
static void sep_core_snapshot(struct sep_a9 *sep, struct sep_core_snap *s)
{
	unsigned int i;

	memset(s, 0, sizeof(*s));
	s->taken = true;
	s->pmgr_ret = sep_pwrstate_read(&s->pmgr);
	if (s->pmgr_ret || FIELD_GET(SEP_PS_ACTUAL, s->pmgr) != SEP_PS_ACTIVE ||
	    (s->pmgr & SEP_PS_RESET))
		return;
	for (i = 0; i < SEP_CORE_NREGS; i++)
		s->reg[i] = readl(sep->base + sep_core_regs[i]);
	s->valid = true;
}

/*
 * Arm: AppleA7IOPV2 wrapper activation as AppleA7IOP::start performs it after
 * setActive(true) (docs/sep/xart-handshake.md section 6). Mask the four known
 * sources in both views, unmask bit 0 (inbox) then bit 12 (outbox) in both
 * views, set +0x4020 bit 0 by read-modify-write, read back to flush. The old
 * arm cleared only bit 12 in the primary view: a sepOS waiting in WFI then
 * never saw the AP->SEP words (14:00 run, five words left queued for 40 s).
 *
 * setActive(true) has no SEP-local MMIO equivalent, so the PMGR word must be
 * the pongoOS handoff value 0x4f0000ff (sep_pwrstate_gate): on and out of
 * reset. AUTO_ENABLE (bit 28) only draws a warning and is not repaired; any
 * other difference is refused and nothing is touched. No PMGR, CPU RUN
 * (+0x44), NMI, reset or auto-boot write; +0x4008 is never written and +0x4020
 * only gains bit 0. The core registers are snapshotted before and after so
 * that can be checked. Called with cfg_lock held.
 */
static int sep_arm(struct sep_a9 *sep)
{
	struct sep_core_snap *pre = &sep->arm_snap, *post = &sep->arm_after_snap;
	void __iomem *b = sep->base;
	u32 ps, v;
	char line[192];
	int ret;

	ret = sep_pwrstate_read(&ps);
	if (ret) {
		dev_err(sep->dev, "arm refused: no PMGR sep power state (%d), SEP untouched\n",
			ret);
		return ret;
	}
	ret = sep_pwrstate_gate(sep, ps, "arm");
	if (ret)
		return ret;
	sep->pwrstate = ps;
	dev_info(sep->dev, "arm: sep power domain on (pwrstate=%08x)\n", ps);

	sep_core_snapshot(sep, pre);
	sep_core_print(line, sizeof(line), pre);
	dev_info(sep->dev, "arm: core before %s\n", line);

	sep_wrap_snapshot(sep, sep->wrap_before);
	sep_wrap_print(line, sizeof(line), sep->wrap_before);
	dev_info(sep->dev, "arm: before %s\n", line);

	writel(SEP_WRAP_MASK_ALL, b + SEP_WRAP_MASK_SET);
	writel(SEP_WRAP_MASK_ALL, b + SEP_WRAP_MIRROR_SET);
	writel(SEP_WRAP_INBOX_IRQ, b + SEP_WRAP_MASK_CLR);
	writel(SEP_WRAP_INBOX_IRQ, b + SEP_WRAP_MIRROR_CLR);
	writel(SEP_WRAP_OUTBOX_IRQ, b + SEP_WRAP_MASK_CLR);
	writel(SEP_WRAP_OUTBOX_IRQ, b + SEP_WRAP_MIRROR_CLR);
	v = readl(b + SEP_WRAP_OUTBOX_CTL);
	writel(v | SEP_WRAP_OUTBOX_EN, b + SEP_WRAP_OUTBOX_CTL);
	/* flush the posted writes before any mailbox word can follow */
	readl(b + SEP_WRAP_OUTBOX_CTL);
	mb();

	sep->outbox_read = v;
	sep_wrap_snapshot(sep, sep->wrap_after);
	sep->armed_once = true;
	dev_info(sep->dev, "arm: wrote 4000=%x c00=%x 4004=%x c04=%x 4004=%x c04=%x 4020=%08x (read %08x)\n",
		 SEP_WRAP_MASK_ALL, SEP_WRAP_MASK_ALL, SEP_WRAP_INBOX_IRQ,
		 SEP_WRAP_INBOX_IRQ, SEP_WRAP_OUTBOX_IRQ, SEP_WRAP_OUTBOX_IRQ,
		 v | SEP_WRAP_OUTBOX_EN, v);
	sep_wrap_print(line, sizeof(line), sep->wrap_after);
	dev_info(sep->dev, "arm: after %s (AP->SEP FIFO %s)\n", line,
		 sep_fifo_state(sep->wrap_after[4]));

	sep_core_snapshot(sep, post);
	sep_core_print(line, sizeof(line), post);
	dev_info(sep->dev, "arm: core after %s\n", line);
	if (post->pmgr_ret || post->pmgr != ps)
		dev_warn(sep->dev, "arm: PMGR sep word changed during arm: %08x -> %08x\n",
			 ps, post->pmgr);
	if (pre->valid && post->valid) {
		if (pre->reg[1] != post->reg[1])
			dev_warn(sep->dev, "arm: CPU control +0x44 changed %08x -> %08x\n",
				 pre->reg[1], post->reg[1]);
		if ((pre->reg[3] ^ post->reg[3]) & BIT(0))
			dev_warn(sep->dev, "arm: +0x4008 bit 0 changed %08x -> %08x\n",
				 pre->reg[3], post->reg[3]);
		if (!(post->reg[4] & SEP_WRAP_OUTBOX_EN))
			dev_warn(sep->dev, "arm: +0x4020 bit 0 clear after arm (%08x)\n",
				 post->reg[4]);
	}
	return 0;
}

/*
 * Arming control. Nothing writes the SEP registers until the user writes 1
 * here, and before that they are only read once at probe, and only while the
 * PMGR word says the domain is on: the first read of a powered-down SEP block
 * can stall the AP, and on a checkm8 boot sepOS is usually not running. Arm
 * only after the pongoOS/m1n1 side has confirmed the SEP is up. Arming
 * activates the wrapper (sep_arm), then only drains and records; it sends
 * nothing.
 */
static ssize_t sep_dbg_start_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int on;
	char buf[8];
	int ret = 0;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtouint(buf, 0, &on))
		return -EINVAL;

	mutex_lock(&sep->cfg_lock);
	if (on && !sep->running) {
		ret = sep_arm(sep);
		if (!ret) {
			sep->running = true;
			schedule_delayed_work(&sep->poll, 0);
		}
	} else if (!on && sep->running) {
		sep->running = false;
		cancel_delayed_work_sync(&sep->poll);
		writel(SEP_MBOX_INT_ENABLE, sep->mbox + SEP_MBOX_DIS_INT);
	}
	mutex_unlock(&sep->cfg_lock);

	return ret ? ret : count;
}

static const struct file_operations sep_dbg_start_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = sep_dbg_start_write,
};

static void sep_wake_report(struct sep_a9 *sep)
{
	const char *res = sep_wake_result(sep);
	u64 m = sep->wake_msg;

	if (sep->wake_pmgr_after_ret)
		dev_warn(sep->dev, "wake-test: PMGR sep re-read failed (%d)\n",
			 sep->wake_pmgr_after_ret);
	else
		dev_info(sep->dev, "wake-test: pwrstate before=%08x after=%08x%s\n",
			 sep->wake_pmgr, sep->wake_pmgr_after,
			 sep->wake_pmgr_after == sep->wake_pmgr ? "" : " CHANGED");

	if (sep->wake_seen)
		dev_info(sep->dev, "wake-test: result=%s: ep=%02x tag=%02x op=%02x param=%02x data=%08x (raw %016llx) %lld us after word 1, indications=%u, word 2 %s, other=%u\n",
			 res, SEP_MSG_GET_EP(m), SEP_MSG_GET_TAG(m),
			 SEP_MSG_GET_OP(m), SEP_MSG_GET_PARAM(m),
			 SEP_MSG_GET_DATA(m), m,
			 ktime_us_delta(sep->wake_at, sep->wake_t0), sep->wake_ind,
			 sep->wake_w2_drained ? "drained" :
			 sep->wake_w2_posted ? "NOT drained" : "not posted",
			 sep->wake_other);
	else
		dev_warn(sep->dev, "wake-test: result=%s (ret %d): word 1 %s, word 2 %s, no ep 0 op 0x0f/0x15, other=%u first=%016llx, snd_sts %#x -> %#x -> %#x\n",
			 res, sep->wake_ret,
			 sep->wake_w1_drained ? "drained" : "NOT drained",
			 sep->wake_w2_drained ? "drained" :
			 sep->wake_w2_posted ? "NOT drained" : "not posted",
			 sep->wake_other, sep->wake_other_first,
			 sep->wake_sts[0], sep->wake_sts[1], sep->wake_sts[2]);
}

/*
 * XNU's endpoint-0 nap wake (AppleSEPManager_waitUntilCanSendMessage,
 * docs/sep/xart-handshake.md section 7), bounded and once per boot. Preconds:
 * armed, PMGR word still 0x4f0000ff (AUTO_ENABLE only warned about, see
 * sep_pwrstate_gate), no published OOL buffer, AP->SEP FIFO empty; otherwise
 * nothing is sent. Then: post raw 0x00000000000e0000 (ep 0 tag 0 op 0x0e
 * param 0 data 0) through sep_send_raw, one 64-bit store to +0x4010, wait
 * up to 1 s for the FIFO to drain, and post the same word a second time only
 * if it did; then wait up to 1 s for ep 0 op 0x0f or 0x15. Nothing is retried
 * and there is no 200-pair loop. Called with cfg_lock held.
 */
static int sep_wake_test(struct sep_a9 *sep)
{
	unsigned int i, published = 0;
	ktime_t t2, deadline;
	u32 ps, sts;
	int ret;

	if (!sep->running)
		return -ENXIO;
	if (sep->wake_ran)
		return -EALREADY;

	mutex_lock(&sep->lock);
	for (i = 0; i < SEP_EP_COUNT; i++)
		published += sep->eps[i].published;
	mutex_unlock(&sep->lock);
	if (published) {
		dev_warn(sep->dev, "wake-test refused: %u endpoint(s) already have OOL published, nothing sent\n",
			 published);
		return -EBUSY;
	}

	ret = sep_pwrstate_read(&ps);
	if (ret) {
		dev_err(sep->dev, "wake-test refused: no PMGR sep power state (%d), nothing sent\n",
			ret);
		return ret;
	}
	ret = sep_pwrstate_gate(sep, ps, "wake-test");
	if (ret)
		return ret;

	sts = readl(sep->mbox + SEP_MBOX_SND_STS);
	if (!(sts & SEP_MBOX_STS_EMPTY)) {
		dev_warn(sep->dev, "wake-test refused: AP->SEP FIFO not empty (snd_sts=%#x), nothing sent\n",
			 sts);
		return -EBUSY;
	}

	/* From here on, this boot has used its one wake test. */
	sep->wake_ran = true;
	sep->wake_pmgr = ps;
	sep->wake_sts[0] = sts;
	sep->wake_sts[1] = sep->wake_sts[2] = 0;
	sep->wake_w1_drained = sep->wake_w2_posted = sep->wake_w2_drained = false;
	sep->wake_w1_us = sep->wake_w2_us = 0;

	mutex_lock(&sep->lock);
	sep->wake_seen = false;
	sep->wake_msg = 0;
	sep->wake_ind = 0;
	sep->wake_other = 0;
	sep->wake_other_first = 0;
	sep->wake_waiting = true;
	mutex_unlock(&sep->lock);

	dev_info(sep->dev, "wake-test: pwrstate=%08x snd_sts=%#x, posting %016llx (64-bit store to +0x4010)\n",
		 ps, sts, SEP_MSG(0, 0, SEP_OP_NAP_WAKE, 0, 0));
	sep->wake_t0 = ktime_get();
	ret = sep_send_raw(sep, 0, 0, SEP_OP_NAP_WAKE, 0, 0);
	if (ret)
		goto out;
	ret = sep_tx_drain(sep, sts, "wake-test word 1");
	sep->wake_sts[1] = sep->drain_sts_after;
	sep->wake_w1_us = sep->drain_us;
	if (ret) {
		dev_warn(sep->dev, "wake-test: word 1 not consumed, word 2 not posted\n");
		ret = -ETIMEDOUT;
		goto out;
	}
	sep->wake_w1_drained = true;

	ret = sep_send_raw(sep, 0, 0, SEP_OP_NAP_WAKE, 0, 0);
	if (ret)
		goto out;
	sep->wake_w2_posted = true;
	t2 = ktime_get();
	deadline = ktime_add_us(t2, SEP_WAKE_WAIT_US);
	for (;;) {
		bool seen;

		sep_drain(sep);
		mutex_lock(&sep->lock);
		seen = sep->wake_seen;
		mutex_unlock(&sep->lock);

		sts = readl(sep->mbox + SEP_MBOX_SND_STS);
		if (!sep->wake_w2_drained && (sts & SEP_MBOX_STS_EMPTY)) {
			sep->wake_w2_drained = true;
			sep->wake_w2_us = ktime_us_delta(ktime_get(), t2);
		}
		if ((seen && sep->wake_w2_drained) ||
		    ktime_after(ktime_get(), deadline))
			break;
		usleep_range(500, 1000);
	}
	sep->wake_sts[2] = sts;

out:
	if (!sep->wake_w2_posted)
		sep->wake_sts[2] = readl(sep->mbox + SEP_MBOX_SND_STS);
	sep->wake_pmgr_after_ret = sep_pwrstate_read(&sep->wake_pmgr_after);
	mutex_lock(&sep->lock);
	sep->wake_waiting = false;
	if (!ret)
		ret = sep->wake_seen ? 0 : -ENODATA;
	mutex_unlock(&sep->lock);
	sep->wake_ret = ret;
	sep_wake_report(sep);
	return ret;
}

/*
 * echo 1 > wake-test: run sep_wake_test once. The write fails with
 * ENXIO (unarmed), EALREADY (already run this boot), EBUSY (OOL published or
 * FIFO not empty: nothing sent), EIO (PMGR word not 0x4f0000ff apart from
 * AUTO_ENABLE, or off: nothing sent),
 * ETIMEDOUT (word 1 not consumed: word 2 not sent) or ENODATA (no wake
 * indication). cat wake-test reads the recorded result; it reads no register.
 */
static ssize_t sep_dbg_wake_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int on;
	char buf[8];
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtouint(buf, 0, &on) || on != 1)
		return -EINVAL;

	mutex_lock(&sep->cfg_lock);
	ret = sep_wake_test(sep);
	mutex_unlock(&sep->cfg_lock);
	return ret ? ret : count;
}

static ssize_t sep_dbg_wake_read(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	const size_t size = 1024;
	ssize_t len = 0;
	char *buf;
	u64 m;

	buf = kzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&sep->cfg_lock);
	len += scnprintf(buf + len, size - len, "result=%s", sep_wake_result(sep));
	if (!sep->wake_ran) {
		len += scnprintf(buf + len, size - len, "\n");
		goto out;
	}
	m = sep->wake_msg;
	len += scnprintf(buf + len, size - len,
			 " ret=%d\npwrstate before=%08x after=%08x (read %d) word=%016llx tx=writeq+0x4010\n"
			 "word1 drained=%d after %lld us, snd_sts %#x -> %#x\n"
			 "word2 posted=%d drained=%d after %lld us, snd_sts at end %#x\n",
			 sep->wake_ret, sep->wake_pmgr, sep->wake_pmgr_after,
			 sep->wake_pmgr_after_ret,
			 SEP_MSG(0, 0, SEP_OP_NAP_WAKE, 0, 0),
			 sep->wake_w1_drained, sep->wake_w1_us,
			 sep->wake_sts[0], sep->wake_sts[1],
			 sep->wake_w2_posted, sep->wake_w2_drained,
			 sep->wake_w2_us, sep->wake_sts[2]);
	if (sep->wake_seen)
		len += scnprintf(buf + len, size - len,
				 "indication ep=%02x tag=%02x op=%02x param=%02x data=%08x raw=%016llx after %lld us (count %u)\n",
				 SEP_MSG_GET_EP(m), SEP_MSG_GET_TAG(m),
				 SEP_MSG_GET_OP(m), SEP_MSG_GET_PARAM(m),
				 SEP_MSG_GET_DATA(m), m,
				 ktime_us_delta(sep->wake_at, sep->wake_t0),
				 sep->wake_ind);
	else
		len += scnprintf(buf + len, size - len,
				 "indication none (ep 0 op 0x0f/0x15)\n");
	len += scnprintf(buf + len, size - len, "other=%u first=%016llx\n",
			 sep->wake_other, sep->wake_other_first);
out:
	mutex_unlock(&sep->cfg_lock);

	len = simple_read_from_buffer(ubuf, count, ppos, buf, len);
	kfree(buf);
	return len;
}

static const struct file_operations sep_dbg_wake_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sep_dbg_wake_read,
	.write = sep_dbg_wake_write,
};


static int sep_hex2bin(u8 *dst, const char *src, size_t max)
{
	size_t n = 0;

	while (*src && n < max) {
		unsigned int v;
		char byte[3];

		while (*src == ' ')
			src++;
		if (!src[0] || !src[1])
			break;
		byte[0] = src[0];
		byte[1] = src[1];
		byte[2] = '\0';
		if (kstrtouint(byte, 16, &v))
			break;
		dst[n++] = v;
		src += 2;
	}
	return n;
}

/*
 * echo "EP CMD [HEXPAYLOAD]" > xfer	(hex; payload as bytes, "01000000")
 * Runs one single-chunk endpoint command and keeps the result for read().
 */
static ssize_t sep_dbg_xfer_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int ep, cmd;
	size_t n, replylen;
	u32 status;
	u8 *payload;
	char *buf, *hex;
	int ret;

	if (count > SEP_XFER_CHUNK * 2 + 32)
		return -EINVAL;

	buf = memdup_user_nul(ubuf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	if (sscanf(buf, "%x %x", &ep, &cmd) != 2 || ep > SEP_EP_CLIENT_MAX ||
	    cmd > 0xff) {
		kfree(buf);
		return -EINVAL;
	}

	payload = kmalloc(SEP_XFER_CHUNK, GFP_KERNEL);
	if (!payload) {
		kfree(buf);
		return -ENOMEM;
	}

	hex = strchr(buf, ' ');
	hex = hex ? strchr(hex + 1, ' ') : NULL;
	n = hex ? sep_hex2bin(payload, hex + 1, SEP_XFER_CHUNK) : 0;

	mutex_lock(&sep->cfg_lock);
	ret = sep_transfer(sep, ep, cmd, payload, n, sep->last_out,
			   sizeof(sep->last_out), &replylen, &status);
	sep->last_ep = ep;
	sep->last_cmd = cmd;
	sep->last_ret = ret < 0 ? ret : 0;
	sep->last_status = status;
	sep->last_out_len = ret < 0 ? 0 : ret;
	if (ret >= 0 && cmd == SEP_CMD_SBIO_INIT && (int)ep == sep->sbio_ep) {
		sep->sbio_ready = true;
		dev_info(sep->dev, "sbio ep %#04x: init (0x73) acknowledged\n", ep);
	}
	mutex_unlock(&sep->cfg_lock);

	if (ret >= 0 && replylen > sizeof(sep->last_out))
		dev_warn(sep->dev, "ep %#04x cmd %#x: reply %zu bytes truncated\n",
			 ep, cmd, replylen);
	kfree(payload);
	kfree(buf);

	return ret < 0 ? ret : count;
}

static ssize_t sep_dbg_xfer_read(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	size_t size = SEP_XFER_CHUNK * 2 + 128;
	char *buf;
	ssize_t len = 0;
	size_t i;

	buf = kvzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&sep->cfg_lock);
	len += scnprintf(buf + len, size - len,
			 "out_len=%zu\n", sep->last_out_len);
	for (i = 0; i < sep->last_out_len; i++)
		len += scnprintf(buf + len, size - len, "%02x",
				 sep->last_out[i]);
	len += scnprintf(buf + len, size - len,
			 "\nep=%#04x cmd=%#04x ret=%d sep_status=%#x\n",
			 sep->last_ep, sep->last_cmd, sep->last_ret,
			 sep->last_status);
	len += scnprintf(buf + len, size - len,
			 "gt_reply version=%u total=%u offset=%u flags=%#x reserved=%#x command=%#x chunk=%u mailbox_data=%#x\n",
			 sep->last_gt_version, sep->last_gt_total,
			 sep->last_gt_offset, sep->last_gt_flags,
			 sep->last_gt_reserved, sep->last_gt_command,
			 sep->last_gt_chunk, sep->last_gt_mailbox_data);
	mutex_unlock(&sep->cfg_lock);

	len = simple_read_from_buffer(ubuf, count, ppos, buf, len);
	kvfree(buf);
	return len;
}

static const struct file_operations sep_dbg_xfer_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sep_dbg_xfer_read,
	.write = sep_dbg_xfer_write,
};


/*
 * echo EP > sbio : select the sbio endpoint (hex). Accepted only for an
 * endpoint whose complete advertisement names it 'sbio'.
 */
static ssize_t sep_dbg_sbio_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct sep_a9 *sep = file->private_data;
	unsigned int ep;
	char buf[8];
	bool ok;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtouint(strim(buf), 16, &ep) || ep > SEP_EP_CLIENT_MAX)
		return -EINVAL;

	mutex_lock(&sep->lock);
	ok = sep_ep_complete(&sep->eps[ep]) &&
	     sep->eps[ep].name == 0x7362696f;	/* 'sbio' */
	mutex_unlock(&sep->lock);
	if (!ok)
		return -ENOENT;

	mutex_lock(&sep->cfg_lock);
	sep->sbio_ep = ep;
	sep->sbio_ready = false;
	mutex_unlock(&sep->cfg_lock);
	return count;
}

static const struct file_operations sep_dbg_sbio_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = sep_dbg_sbio_write,
};

static void sep_debugfs_init(struct sep_a9 *sep)
{
	sep->dbg = debugfs_create_dir("sep-a9", NULL);
	if (IS_ERR_OR_NULL(sep->dbg)) {
		sep->dbg = NULL;
		return;
	}

	debugfs_create_file("status", 0444, sep->dbg, sep, &sep_dbg_status_fops);
	debugfs_create_file("log", 0444, sep->dbg, sep, &sep_dbg_log_fops);
	debugfs_create_file("endpoints", 0444, sep->dbg, sep,
			    &sep_dbg_endpoints_fops);
	debugfs_create_file("advertise", 0200, sep->dbg, sep,
			    &sep_dbg_advertise_fops);
	debugfs_create_file("send", 0200, sep->dbg, sep, &sep_dbg_send_fops);
	debugfs_create_file("ool", 0200, sep->dbg, sep, &sep_dbg_ool_fops);
	debugfs_create_file("arts-manifest", 0200, sep->dbg, sep,
			    &sep_dbg_arts_manifest_fops);
	debugfs_create_file("arts-start", 0600, sep->dbg, sep,
			    &sep_dbg_arts_start_fops);
	debugfs_create_file("start", 0200, sep->dbg, sep, &sep_dbg_start_fops);
	debugfs_create_file("fifo", 0444, sep->dbg, sep, &sep_dbg_fifo_fops);
	debugfs_create_file("xfer", 0600, sep->dbg, sep, &sep_dbg_xfer_fops);
	debugfs_create_file("sbio", 0200, sep->dbg, sep, &sep_dbg_sbio_fops);
	debugfs_create_file("wake-test", 0600, sep->dbg, sep, &sep_dbg_wake_fops);
}

static int sep_probe(struct platform_device *pdev)
{
	struct sep_a9 *sep;
	struct resource *res;
	char line[192];

	sep = devm_kzalloc(&pdev->dev, sizeof(*sep), GFP_KERNEL);
	if (!sep)
		return -ENOMEM;
	sep->eps = devm_kcalloc(&pdev->dev, SEP_EP_COUNT, sizeof(*sep->eps),
				GFP_KERNEL);
	if (!sep->eps)
		return -ENOMEM;

	sep->dev = &pdev->dev;
	mutex_init(&sep->lock);
	mutex_init(&sep->tx_lock);
	mutex_init(&sep->cfg_lock);
	init_completion(&sep->xfer_done);
	init_completion(&sep->arts_done);
	INIT_DELAYED_WORK(&sep->poll, sep_poll_work);
	sep->poll_us = 10000;
	sep->sbio_ep = -1;
	sep->last_ep = -1;
	sep->arts_ep = -1;
	sep->arts_result = SEP_ARTS_NEVER;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	sep->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sep->base))
		return PTR_ERR(sep->base);

	sep->mbox = sep->base + SEP_MBOX_OFFSET;

	/*
	 * No SEP write here; the mailbox is armed through debugfs start. The
	 * only access is this read-only snapshot, and its SEP reads happen only
	 * if the PMGR word says on. sep-a9 probes before pmgr-pwrstate, so this
	 * PMGR word is the one the bootloader chain left.
	 */
	sep_core_snapshot(sep, &sep->probe_snap);
	sep_core_print(line, sizeof(line), &sep->probe_snap);
	dev_info(&pdev->dev, "probe: %s\n", line);

	platform_set_drvdata(pdev, sep);
	sep_a9_instance = sep;

	if (misc_register(&sep_bio_dev))
		dev_warn(&pdev->dev, "could not register /dev/sep-bio\n");

	sep_debugfs_init(sep);

	dev_info(&pdev->dev, "mailbox at %px (sep %px), unarmed\n", sep->mbox,
		 sep->base);
	dev_info(&pdev->dev, "tx: one 64-bit store per message to +0x%x (writeq)\n",
		 SEP_MBOX_OFFSET + SEP_MBOX_SND);
	dev_info(&pdev->dev, "rx: one 64-bit load per message from +0x%x (readq, nonempty only)\n",
		 SEP_MBOX_OFFSET + SEP_MBOX_RECV);
	return 0;
}

static void sep_remove(struct platform_device *pdev)
{
	struct sep_a9 *sep = platform_get_drvdata(pdev);

	sep->running = false;
	cancel_delayed_work_sync(&sep->poll);
	misc_deregister(&sep_bio_dev);
	sep_a9_instance = NULL;
	writel(SEP_MBOX_INT_ENABLE, sep->mbox + SEP_MBOX_DIS_INT);
	debugfs_remove_recursive(sep->dbg);
	/* Published OOL buffers stay allocated: the SEP may still use them. */
}

static const struct of_device_id sep_of_match[] = {
	{ .compatible = "apple,s8000-sep" },
	{ .compatible = "apple,s8003-sep" },
	{ }
};
MODULE_DEVICE_TABLE(of, sep_of_match);

static struct platform_driver sep_driver = {
	.probe = sep_probe,
	.remove = sep_remove,
	.driver = {
		.name = "sep-a9",
		.of_match_table = sep_of_match,
	},
};

/*
 * Without a SEP node in the device tree, instantiate the device ourselves
 * with the j71t addresses from the ADT. Interrupts are not wired up in this
 * stage (see docs/sep/driver-notes.md); the mailbox is polled.
 */
#define SEP_A9_BASE	0x20da00000ULL
#define SEP_A9_SIZE	0x10000

static struct platform_device *sep_pdev;

/*
 * Off by default: reading the SEP block while its power domain is down can
 * stall the AP, and on a checkm8 boot sepOS is usually not running at all.
 * Enable explicitly with sep_a9.enable=1 on the kernel command line.
 */
static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Probe the A9 SEP mailbox (default: off)");

static int __init sep_init(void)
{
	struct resource res[] = {
		DEFINE_RES_MEM(SEP_A9_BASE, SEP_A9_SIZE),
	};
	struct device_node *np;
	int ret;

	if (!enable) {
		pr_info("sep-a9: disabled; pass sep_a9.enable=1 to probe\n");
		return 0;
	}

	/*
	 * If the device tree describes the SEP (apple,s8003-sep), the platform
	 * driver instantiates itself from it; registering our own device would
	 * create a second instance.
	 */
	np = of_find_compatible_node(NULL, NULL, "apple,s8003-sep");
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "apple,s8000-sep");
	if (np) {
		of_node_put(np);
		return platform_driver_register(&sep_driver);
	}

	sep_pdev = platform_device_register_simple("sep-a9", -1, res,
						   ARRAY_SIZE(res));
	if (IS_ERR(sep_pdev))
		return PTR_ERR(sep_pdev);

	ret = platform_driver_register(&sep_driver);
	if (ret) {
		platform_device_unregister(sep_pdev);
		return ret;
	}

	return 0;
}

static void __exit sep_exit(void)
{
	platform_driver_unregister(&sep_driver);
	if (sep_pdev)
		platform_device_unregister(sep_pdev);
}

module_init(sep_init);
module_exit(sep_exit);

MODULE_DESCRIPTION("Apple A9 Secure Enclave mailbox");
MODULE_LICENSE("GPL");

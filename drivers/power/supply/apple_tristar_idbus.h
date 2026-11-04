/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CBTL1610 (Tristar) Lightning ID-bus master, driven over I2C, as
 * reconstructed from the iOS 16.7.10 J71t kernelcache (AppleCBTL1610 and
 * AppleTriStarIC) and corrected by the 2026-10-26 hardware trial. Tristar
 * clocks the wire, frames BREAKs and appends the request CRC itself. The
 * host prepares the engine (0x60 = 0x01), loads the TX window that ends at
 * 0x60 (request bytes reversed, then the RX length byte at 0x60), reads it
 * back, writes GO to 0x61 and reads the response forward from the RX window
 * at 0x20.
 *
 * The RX window is never written: on this IC a write there overwrites the
 * retained response. 0x20/0x40/0x41 are another Tristar generation's
 * transfer registers and are inert here.
 *
 * All register access goes through struct tristar_idbus_ops so that the
 * offline harness runs this exact code against a fake register file. The
 * includer provides u8, bool, size_t, memcpy, memcmp, memset, snprintf,
 * __printf and the errno values.
 *
 * Safety envelope: only volatile registers 0x05, 0x0a and the TX window
 * 0x3c..0x61 are written. The RX window 0x20..0x35, 0x17 (AID switch
 * config), 0x0e (detect latch), the mux registers 0x01..0x03 and every
 * OTP/ESN request stay untouched.
 */
#ifndef APPLE_TRISTAR_IDBUS_H
#define APPLE_TRISTAR_IDBUS_H

#define TRISTAR_IDBUS_REG_DXCTRL	0x01
#define TRISTAR_IDBUS_REG_ACCMUX	0x02
/* bit 0 VBUS sink, bit 1 reorient, bit 2 BREAK, bit 3 ID-bus reset hold */
#define TRISTAR_IDBUS_REG_CTRL		0x05
#define TRISTAR_IDBUS_CTRL_REORIENT	0x02
#define TRISTAR_IDBUS_CTRL_BREAK	0x04
#define TRISTAR_IDBUS_CTRL_RESET	0x08
/* Pin faults: bit 0 ACC1, bit 3 ACC2, bit 1 DP1/DN1, bit 2 DP2/DN2 */
#define TRISTAR_IDBUS_REG_FAULT		0x0a
#define TRISTAR_IDBUS_FAULT_PINS	0x0f
#define TRISTAR_IDBUS_REG_STATUS	0x0c
#define TRISTAR_IDBUS_REG_EVENT		0x0d
#define TRISTAR_IDBUS_REG_REVISION	0x0f
#define TRISTAR_IDBUS_REG_AIDSW		0x17
#define TRISTAR_IDBUS_REG_RX		0x20	/* response, forward; read only */
#define TRISTAR_IDBUS_REG_TX_END	0x60	/* RX length byte; prepare */
#define TRISTAR_IDBUS_REG_GO		0x61	/* ((tx - 1) << 1) | 1 */
#define TRISTAR_IDBUS_TX_PREPARE	0x01
#define TRISTAR_IDBUS_REVISION_J71T	0x8a

#define TRISTAR_IDBUS_EV_DETECT		0x01
#define TRISTAR_IDBUS_EV_WAKE		0x02
#define TRISTAR_IDBUS_EV_RESET		0x04
#define TRISTAR_IDBUS_EV_DIGITAL_ID	0x08
#define TRISTAR_IDBUS_EV_DONE		0x10
#define TRISTAR_IDBUS_EV_ERROR		0x20
#define TRISTAR_IDBUS_EV_TIMEOUT	0x40
#define TRISTAR_IDBUS_EV_FAULT		0x80
#define TRISTAR_IDBUS_EV_DONE_MASK	0x38

#define TRISTAR_IDBUS_GET_ID		0x74
#define TRISTAR_IDBUS_ID		0x75
#define TRISTAR_IDBUS_ID_LEN		6
#define TRISTAR_IDBUS_HOSTID		0x0002	/* IOAccessoryManagerSystemID */
#define TRISTAR_IDBUS_POWER		0x70
#define TRISTAR_IDBUS_POWER_ACK		0x71
#define TRISTAR_IDBUS_GET_STATE		0x72
#define TRISTAR_IDBUS_STATE		0x73
#define TRISTAR_IDBUS_GET_INTERFACE_INFO	0x76
#define TRISTAR_IDBUS_INTERFACE_INFO	0x77
#define TRISTAR_IDBUS_GET_MODULE_SERIAL	0x78
#define TRISTAR_IDBUS_MODULE_SERIAL	0x79
#define TRISTAR_IDBUS_GET_ACCESSORY_SERIAL 0x7a
#define TRISTAR_IDBUS_ACCESSORY_SERIAL	0x7b
#define TRISTAR_IDBUS_SYSTEM_INFO	0x84
#define TRISTAR_IDBUS_POWER_RETRY	0x08	/* 0x70 flags bit on iOS's retry path */
#define TRISTAR_IDBUS_MAX_TX		36	/* 32 data bytes + 0x84,type,length */
#define TRISTAR_IDBUS_MAX_FRAME		22	/* 0x79/0x7b + 20 bytes + CRC */
#define TRISTAR_IDBUS_ID_FRAME_LEN	8

/* iOS-order adapter information steps; module parameter masks with ALL. */
#define TRISTAR_IDBUS_INFO_DEVICE	0x01
#define TRISTAR_IDBUS_INFO_MODULE_SERIAL	0x02
#define TRISTAR_IDBUS_INFO_ACCESSORY_SERIAL 0x04
#define TRISTAR_IDBUS_INFO_MODEL		0x08
#define TRISTAR_IDBUS_INFO_OSVERSION	0x10
#define TRISTAR_IDBUS_INFO_ALL		0x1f
#define TRISTAR_IDBUS_IOS_TIMEOUT_COMPLETION	1

/* BREAK after a SystemInfo timeout: never / only before the next 0x84 / also after */
#define TRISTAR_IDBUS_TIMEOUT_BREAK_OFF		0
#define TRISTAR_IDBUS_TIMEOUT_BREAK_INFO	1
#define TRISTAR_IDBUS_TIMEOUT_BREAK_NEXT	2

#define TRISTAR_IDBUS_POLL_MS		10
#define TRISTAR_IDBUS_POLL_STEPS	10	/* 100 ms per transfer */
#define TRISTAR_IDBUS_SETTLE_MS		20
#define TRISTAR_IDBUS_REORIENT_STEPS	3	/* 20 + 3 * 10 = 50 ms after a reorient */
#define TRISTAR_IDBUS_RESET_HOLD_MS	500	/* AppleCBTL1610 reset pulse */
#define TRISTAR_IDBUS_CMDID_GAP_MS	100	/* iOS idCommandTooFast */

/* Identify mode flags (manual variants; never applied automatically) */
#define TRISTAR_IDBUS_SWAP_HOSTID	0x1
#define TRISTAR_IDBUS_NO_REORIENT	0x2

/* Link recovery steps (ds-lightning-host-idbus.md 9.6) */
#define TRISTAR_IDBUS_RECOVER_REORIENT	0
#define TRISTAR_IDBUS_RECOVER_RESET	1
#define TRISTAR_IDBUS_RECOVER_FAULT_ACK	2

struct tristar_idbus_ops {
	/* One register: value 0..255 or -errno */
	int (*read)(void *ctx, u8 reg);
	int (*write)(void *ctx, u8 reg, u8 val);
	/* Bus-atomic read, write (old & ~clear) | set; returns old or -errno */
	int (*update)(void *ctx, u8 reg, u8 clear, u8 set);
	/* len bytes upward from reg, one I2C transaction each; 0 or -errno */
	int (*block_read)(void *ctx, u8 reg, u8 *buf, unsigned int len);
	int (*block_write)(void *ctx, u8 reg, const u8 *buf, unsigned int len);
	void (*sleep_ms)(void *ctx, unsigned int ms);
	void (*log)(void *ctx, const char *fmt, ...) __printf(2, 3);
};

struct tristar_idbus {
	const struct tristar_idbus_ops *ops;
	void *ctx;
	/* First I/O error of the current run; every later access is refused */
	int io_error;
	/* The previous transfer ended in a SystemInfo timeout: BREAK before the next */
	bool break_pending;
	/* Most recent values seen; 0 until read */
	u8 last_ctrl;
	u8 last_event;
	u8 last_fault;
	u8 events;		/* OR of every event-register read */
	unsigned int faults;	/* pin-fault acknowledgements */
};

static const u8 tristar_idbus_baseline_regs[] = {
	TRISTAR_IDBUS_REG_STATUS, TRISTAR_IDBUS_REG_EVENT, TRISTAR_IDBUS_REG_CTRL,
	TRISTAR_IDBUS_REG_DXCTRL, TRISTAR_IDBUS_REG_ACCMUX, TRISTAR_IDBUS_REG_AIDSW,
	TRISTAR_IDBUS_REG_FAULT,
};

#define TRISTAR_IDBUS_NBASE	ARRAY_SIZE(tristar_idbus_baseline_regs)

struct tristar_idbus_id_result {
	int status;
	unsigned int flags;
	unsigned int attempts;			/* GetID commands started */
	bool released_reset;
	bool reoriented;
	bool autonomous;			/* ID from the reorient-armed exchange */
	u8 baseline[TRISTAR_IDBUS_NBASE];
	u8 retained[TRISTAR_IDBUS_ID_FRAME_LEN];	/* RX window before this run */
	bool retained_ok;			/* ... and a valid 0x75 frame */
	u8 reorient_frame[TRISTAR_IDBUS_ID_FRAME_LEN]; /* RX right after the reorient */
	bool reorient_read;
	int reorient_status;			/* verdict, or -ESTALE */
	u8 reorient_events;			/* 0x0d OR after the reorient */
	u8 frame[TRISTAR_IDBUS_ID_FRAME_LEN]; /* the response, kept if bad */
	unsigned int frame_len;
	int frame_status;			/* header/CRC verdict */
	u8 orientation;				/* 0x0c read with the frame */
	u8 id[TRISTAR_IDBUS_ID_LEN];
	bool id_valid;
};

struct tristar_idbus_ph_result {
	int status;
	u8 state[6];		/* 0x73 + 4 bytes + CRC */
	bool state_ok;
	u8 request[3];		/* the 0x70 frame sent */
	u8 ack[2];		/* 0x71 + CRC */
	bool ack_ok;
};

struct tristar_idbus_recover_result {
	int status;
	unsigned int action;
	int status_before;	/* 0x0c, or -1 if not read */
	int status_after;
	u8 events;		/* 0x0d OR after a reorient */
	u8 fault;		/* 0x0a read by fault_ack */
	u8 before[TRISTAR_IDBUS_ID_FRAME_LEN]; /* RX window before a reorient */
	u8 frame[TRISTAR_IDBUS_ID_FRAME_LEN]; /* ... and after */
	bool frame_read;
	int frame_status;
};

struct tristar_idbus_fields {
	u8 usb, accx, db, mb, uart;
	u8 pwr, hv, ai, au;
	u8 pu, bp, cc, bd;
	u8 po, hs, hpm;
};

struct tristar_idbus_accessory_result {
	int status;
	unsigned int requested;
	unsigned int completed;
	u8 device_info[4];
	u8 device_serial[6];
	u8 device_frame[12];
	u8 module_serial[20];
	u8 module_frame[22];
	u8 accessory_serial[20];
	u8 accessory_frame[22];
	u8 model_ack[2];
	u8 osversion_ack[2];
	bool model_timeout;
	bool osversion_timeout;
};

/* Generator 0x8c (0x31 reflected), LSB first. Seed 0xff on the wire. */
static u8 tristar_idbus_crc8_seed(u8 crc, const u8 *buf, unsigned int len)
{
	unsigned int i;

	while (len--) {
		crc ^= *buf++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
	}
	return crc;
}

static u8 tristar_idbus_crc8(const u8 *buf, unsigned int len)
{
	return tristar_idbus_crc8_seed(0xff, buf, len);
}

/* A response is type + rx_len - 1 data bytes + CRC over the first rx_len. */
static int tristar_idbus_check_frame(const u8 *frame, unsigned int rx_len, u8 type)
{
	/* type zero mirrors iOS's 0x84 completion handler: CRC, raw opcode kept. */
	if (type && frame[0] != type)
		return -EPROTO;
	if (tristar_idbus_crc8(frame, rx_len) != frame[rx_len])
		return -EBADMSG;
	return 0;
}

static const char *tristar_idbus_verdict(int verdict)
{
	return verdict == -EPROTO ? "bad header" : verdict == -EBADMSG ? "bad CRC" :
	       verdict == -ESTALE ? "same as the retained frame, no completion event" :
	       verdict ? "not read" : "header and CRC ok";
}

static const char *tristar_idbus_recover_name(unsigned int action)
{
	return action == TRISTAR_IDBUS_RECOVER_REORIENT ? "reorient" :
	       action == TRISTAR_IDBUS_RECOVER_RESET ? "idbus_reset" :
	       action == TRISTAR_IDBUS_RECOVER_FAULT_ACK ? "fault_ack" : "unknown";
}

/* IOAccessoryManagerAID::handleDigitalID field split */
static void tristar_idbus_decode(const u8 *id, struct tristar_idbus_fields *f)
{
	f->accx = id[0] >> 6;
	f->db = (id[0] >> 5) & 1;
	f->usb = (id[0] >> 3) & 3;
	f->mb = (id[0] >> 2) & 1;
	f->uart = id[0] & 3;
	f->pwr = id[1] >> 4;
	f->hv = (id[1] >> 2) & 3;
	f->ai = (id[1] >> 1) & 1;
	f->au = id[1] & 1;
	f->pu = id[2] >> 7;
	f->bp = (id[2] >> 6) & 1;
	f->cc = (id[2] >> 3) & 1;
	f->bd = id[2] & 1;
	f->po = id[3] >> 7;
	f->hs = (id[3] >> 4) & 7;
	f->hpm = id[3] & 7;
}

/* iOS's own log order and names */
static int tristar_idbus_format_fields(const u8 *id, char *buf, size_t size)
{
	struct tristar_idbus_fields f;

	tristar_idbus_decode(id, &f);
	return snprintf(buf, size,
			"USB=%x ACCx=%x DB=%x MB=%x UART=%x PWR=%x HV=%x AI=%x AU=%x PU=%x BP=%x PO=%x CC=%x BD=%x HS=%x HPM=%x",
			f.usb, f.accx, f.db, f.mb, f.uart, f.pwr, f.hv, f.ai, f.au,
			f.pu, f.bp, f.po, f.cc, f.bd, f.hs, f.hpm);
}

static int tristar_idbus_format_hex(const u8 *data, unsigned int len, char *buf, size_t size)
{
	unsigned int i;
	int n = 0;

	if (size)
		buf[0] = '\0';
	for (i = 0; i < len && (size_t)n + 3 < size; i++)
		n += snprintf(buf + n, size - n, i ? " %02x" : "%02x", data[i]);
	return n;
}

static void tristar_idbus_log_status(struct tristar_idbus *ib, const char *when, int val)
{
	ib->ops->log(ib->ctx, "0x0c %s: 0x%02x (VBUS %s, bit7 %u, orientation bit6 %u)", when, val,
		     (val & 3) == 3 ? "present" : "absent", (val >> 7) & 1, (val >> 6) & 1);
}

/*
 * Register access. The first I/O error is kept in ib->io_error and refuses
 * every later access of the run, so nothing is written after a failure.
 */
static int tristar_idbus_fail(struct tristar_idbus *ib, int err)
{
	if (!ib->io_error)
		ib->io_error = err;
	return err;
}

static int tristar_idbus_read(struct tristar_idbus *ib, u8 reg)
{
	int ret;

	if (ib->io_error)
		return ib->io_error;
	ret = ib->ops->read(ib->ctx, reg);
	if (ret < 0) {
		ib->ops->log(ib->ctx, "read 0x%02x failed: %d", reg, ret);
		return tristar_idbus_fail(ib, ret);
	}
	if (reg == TRISTAR_IDBUS_REG_CTRL)
		ib->last_ctrl = ret;
	else if (reg == TRISTAR_IDBUS_REG_FAULT)
		ib->last_fault = ret;
	return ret;
}

static int tristar_idbus_write(struct tristar_idbus *ib, u8 reg, u8 val)
{
	int ret;

	if (ib->io_error)
		return ib->io_error;
	ret = ib->ops->write(ib->ctx, reg, val);
	if (ret) {
		ib->ops->log(ib->ctx, "write 0x%02x = 0x%02x failed: %d", reg, val, ret);
		return tristar_idbus_fail(ib, ret);
	}
	return 0;
}

static int tristar_idbus_block_read(struct tristar_idbus *ib, u8 reg, u8 *buf, unsigned int len)
{
	int ret;

	if (ib->io_error)
		return ib->io_error;
	ret = ib->ops->block_read(ib->ctx, reg, buf, len);
	if (ret) {
		ib->ops->log(ib->ctx, "read of %u bytes at 0x%02x failed: %d", len, reg, ret);
		return tristar_idbus_fail(ib, ret);
	}
	return 0;
}

static int tristar_idbus_block_write(struct tristar_idbus *ib, u8 reg, const u8 *buf,
				     unsigned int len)
{
	int ret;

	if (ib->io_error)
		return ib->io_error;
	ret = ib->ops->block_write(ib->ctx, reg, buf, len);
	if (ret) {
		ib->ops->log(ib->ctx, "write of %u bytes at 0x%02x failed: %d", len, reg, ret);
		return tristar_idbus_fail(ib, ret);
	}
	return 0;
}

/*
 * Read-modify-write; returns the value read back afterwards. *old gets the
 * value the write was computed from.
 */
static int tristar_idbus_update_logged(struct tristar_idbus *ib, u8 reg, u8 clear, u8 set,
				       u8 *old)
{
	int before, after;

	if (ib->io_error)
		return ib->io_error;
	before = ib->ops->update(ib->ctx, reg, clear, set);
	if (before < 0) {
		ib->ops->log(ib->ctx, "update 0x%02x (clear 0x%02x, set 0x%02x) failed: %d",
			     reg, clear, set, before);
		return tristar_idbus_fail(ib, before);
	}
	*old = before;
	after = tristar_idbus_read(ib, reg);
	if (after < 0)
		return after;
	ib->ops->log(ib->ctx, "write 0x%02x: before 0x%02x, wrote 0x%02x, after 0x%02x",
		     reg, before, (u8)((before & ~clear) | set), after);
	return after;
}

static void tristar_idbus_log_fault_names(struct tristar_idbus *ib, const char *what, u8 fault)
{
	ib->ops->log(ib->ctx, "pin fault 0x%02x %s:%s%s%s%s", fault, what,
		     fault & 0x01 ? " ACC1" : "", fault & 0x08 ? " ACC2" : "",
		     fault & 0x02 ? " DP1/DN1" : "", fault & 0x04 ? " DP2/DN2" : "");
}

/* 0x0d bit 7: report, then acknowledge by echoing exactly the value read. */
static int tristar_idbus_ack_fault(struct tristar_idbus *ib)
{
	u8 fault;
	int ret = tristar_idbus_update_logged(ib, TRISTAR_IDBUS_REG_FAULT, 0, 0, &fault);

	if (ret < 0)
		return ret;
	ib->faults++;
	tristar_idbus_log_fault_names(ib, "acknowledged", fault);
	return 0;
}

static int tristar_idbus_read_event(struct tristar_idbus *ib)
{
	int ev = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_EVENT);

	if (ev < 0)
		return ev;
	ib->last_event = ev;
	ib->events |= ev;
	return ev;
}

static int tristar_idbus_check_revision(struct tristar_idbus *ib)
{
	int ret = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_REVISION);

	if (ret < 0)
		return ret;
	if (ret != TRISTAR_IDBUS_REVISION_J71T) {
		ib->ops->log(ib->ctx, "revision 0x%02x is not CBTL1610 0x8a; nothing written", ret);
		return -ENODEV;
	}
	return 0;
}

/*
 * getIdBusData: the response sits forward in the RX window at 0x20 until
 * the next transfer replaces it. Read rx_len + 1 bytes (CRC last) and, when
 * asked, 0x0c for the orientation. Never writes. Returns 0 or -errno; the
 * header/CRC verdict goes to *verdict.
 */
static int tristar_idbus_read_rx(struct tristar_idbus *ib, const char *what, u8 *frame,
				 unsigned int rx_len, u8 type, int *orientation, int *verdict)
{
	char hex[3 * TRISTAR_IDBUS_MAX_FRAME + 1];
	int ret, val;

	ret = tristar_idbus_block_read(ib, TRISTAR_IDBUS_REG_RX, frame, rx_len + 1);
	if (ret)
		return ret;
	*verdict = tristar_idbus_check_frame(frame, rx_len, type);
	tristar_idbus_format_hex(frame, rx_len + 1, hex, sizeof(hex));
	if (!orientation) {
		ib->ops->log(ib->ctx, "%s RX 0x20: [%s] %s", what, hex,
			     !type && !*verdict ? "CRC ok (opcode intentionally unchecked)" :
			     tristar_idbus_verdict(*verdict));
		return 0;
	}
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_STATUS);
	if (val < 0)
		return val;
	*orientation = val;
	ib->ops->log(ib->ctx, "%s RX 0x20: [%s] %s; 0x0c = 0x%02x (orientation bit6 %u)", what, hex,
		     tristar_idbus_verdict(*verdict), val, (val >> 6) & 1);
	return 0;
}

/*
 * AppleCBTL1610_sendIDBusBreak_reg5bit2 (iOS fffffff005ff583c): 0x05 =
 * cached | 0x04, then IODelay(20 us). IOAccessoryIDBusTransport starts the
 * next transfer with it when the previous one completed with a non-zero
 * status and transfer flag 0x20 was clear (transport byte +0x98, checked in
 * fffffff005e3ab28; slot +0x598 -> +0x6a0). The 0x84 transfers use flags 2,
 * so a SystemInfo timeout sets it. The register read-back is the 20 us wait.
 */
static int tristar_idbus_send_break(struct tristar_idbus *ib)
{
	u8 old;
	int ret = tristar_idbus_update_logged(ib, TRISTAR_IDBUS_REG_CTRL, 0,
					      TRISTAR_IDBUS_CTRL_BREAK, &old);

	if (ret < 0)
		return ret;
	ib->ops->log(ib->ctx, "ID-bus BREAK sent before the next transfer (iOS sendIDBusBreak)");
	return 0;
}

/*
 * The transport's 100 ms timer (iOS fffffff005e390d0) first calls the TriStar
 * abort (slot +0x590 -> fffffff005fef218): 0x60 = 0x01 and forget the expected
 * RX length. Then it completes the transfer with kIOReturnTimeout, which
 * arms the BREAK above for the next transfer.
 */
static int tristar_idbus_timeout_abort(struct tristar_idbus *ib, unsigned int timeout_break)
{
	int ret = tristar_idbus_write(ib, TRISTAR_IDBUS_REG_TX_END, TRISTAR_IDBUS_TX_PREPARE);

	if (ret)
		return ret;
	ib->ops->log(ib->ctx, "write 0x60 = 0x01 (iOS timeout abort)");
	ib->break_pending = timeout_break != TRISTAR_IDBUS_TIMEOUT_BREAK_OFF;
	return 0;
}

/*
 * One command (AppleCBTL1610 sendIDBusCommand, the J71t dock path):
 * prepare 0x60 = 0x01, write the request reversed plus the RX length byte
 * (rx_len - 1) << 1 so that it ends at 0x60, read the window back and
 * compare (no GO on a mismatch), then GO at 0x61. Nothing else touches the
 * bus in between. Poll 0x0d every 10 ms for at most 100 ms. Completion is
 * the transfer-done code ((0x0d & 0x38) == 0x10) or, for GetID only, the
 * EVENT_DIGITAL_ID bit (hardware: 0x18), whose iOS handler reads the same
 * frame. The response (rx_len + CRC byte) is read back and kept even when
 * its header or CRC is wrong. *waited accumulates the poll sleeps.
 */
static int tristar_idbus_transfer(struct tristar_idbus *ib, const u8 *tx, unsigned int tx_len,
				  unsigned int rx_len, u8 rx_type, bool digital_id,
				  u8 *frame, unsigned int *frame_len, int *orientation,
				  unsigned int *waited)
{
	u8 win[TRISTAR_IDBUS_MAX_TX + 1], back[TRISTAR_IDBUS_MAX_TX + 1];
	char req[3 * TRISTAR_IDBUS_MAX_TX + 1], hex[3 * (TRISTAR_IDBUS_MAX_TX + 1) + 1];
	u8 base = TRISTAR_IDBUS_REG_TX_END - tx_len, go = ((tx_len - 1) << 1) | 1;
	unsigned int i, step;
	bool system_info = tx[0] == TRISTAR_IDBUS_SYSTEM_INFO;
	int ev, ret, verdict;

	*frame_len = 0;
	if (!tx_len || tx_len > TRISTAR_IDBUS_MAX_TX || !rx_len ||
	    rx_len >= TRISTAR_IDBUS_MAX_FRAME)
		return -EINVAL;
	for (i = 0; i < tx_len; i++)
		win[i] = tx[tx_len - 1 - i];
	win[tx_len] = (rx_len - 1) << 1;
	tristar_idbus_format_hex(tx, tx_len, req, sizeof(req));
	tristar_idbus_format_hex(win, tx_len + 1, hex, sizeof(hex));

	if (ib->break_pending) {
		ib->break_pending = false;
		ret = tristar_idbus_send_break(ib);
		if (ret)
			return ret;
	}
	ret = tristar_idbus_write(ib, TRISTAR_IDBUS_REG_TX_END, TRISTAR_IDBUS_TX_PREPARE);
	if (ret)
		return ret;
	ib->ops->log(ib->ctx, "write 0x60 = 0x01 (prepare)");
	ret = tristar_idbus_block_write(ib, base, win, tx_len + 1);
	if (ret)
		return ret;
	ib->ops->log(ib->ctx,
		     "write TX 0x%02x: [%s] (request [%s] reversed, RX length 0x%02x; Tristar appends CRC 0x%02x)",
		     base, hex, req, win[tx_len], tristar_idbus_crc8(tx, tx_len));
	ret = tristar_idbus_block_read(ib, base, back, tx_len + 1);
	if (ret)
		return ret;
	if (memcmp(back, win, tx_len + 1)) {
		tristar_idbus_format_hex(back, tx_len + 1, hex, sizeof(hex));
		ib->ops->log(ib->ctx, "readback mismatch at 0x%02x: [%s]; no GO", base, hex);
		return -EREMOTEIO;
	}
	ib->ops->log(ib->ctx, "readback TX 0x%02x matches", base);
	ret = tristar_idbus_write(ib, TRISTAR_IDBUS_REG_GO, go);
	if (ret)
		return ret;
	ib->ops->log(ib->ctx, "write 0x61 = 0x%02x (GO)", go);

	for (step = 0; ; step++) {
		ev = tristar_idbus_read_event(ib);
		if (ev < 0)
			return ev;
		if (system_info)
			ib->ops->log(ib->ctx,
				     "SystemInfo 0x84 type %u poll %u ms: 0x0d = 0x%02x",
				     tx_len > 1 ? tx[1] : 0xff, *waited, ev);
		if (system_info)
			ib->ops->log(ib->ctx,
				     "0x0d bits: detect %u wake %u reset %u digital-id %u done %u error %u timeout %u fault %u",
				     !!(ev & TRISTAR_IDBUS_EV_DETECT),
				     !!(ev & TRISTAR_IDBUS_EV_WAKE),
				     !!(ev & TRISTAR_IDBUS_EV_RESET),
				     !!(ev & TRISTAR_IDBUS_EV_DIGITAL_ID),
				     !!(ev & TRISTAR_IDBUS_EV_DONE),
				     !!(ev & TRISTAR_IDBUS_EV_ERROR),
				     !!(ev & TRISTAR_IDBUS_EV_TIMEOUT),
				     !!(ev & TRISTAR_IDBUS_EV_FAULT));
		else if (ev)
			ib->ops->log(ib->ctx, "poll %u ms: 0x0d = 0x%02x", *waited, ev);
		if (ev & TRISTAR_IDBUS_EV_FAULT) {
			ret = tristar_idbus_ack_fault(ib);
			if (ret)
				return ret;
		}
		if ((digital_id && (ev & TRISTAR_IDBUS_EV_DIGITAL_ID)) ||
		    (ev & TRISTAR_IDBUS_EV_DONE_MASK) == TRISTAR_IDBUS_EV_DONE)
			break;
		if (ev & TRISTAR_IDBUS_EV_ERROR) {
			ib->ops->log(ib->ctx, "transfer error event; no retry");
			return -EIO;
		}
		if (ev & TRISTAR_IDBUS_EV_RESET) {
			ib->ops->log(ib->ctx, "ID-bus reset/detach event during transfer");
			return -ECONNRESET;
		}
		if (ev & TRISTAR_IDBUS_EV_TIMEOUT) {
			ib->ops->log(ib->ctx, "Tristar timeout event after %u ms", *waited);
			return -ETIMEDOUT;
		}
		if (step == TRISTAR_IDBUS_POLL_STEPS) {
			ib->ops->log(ib->ctx, "no completion after %u ms (last 0x0d 0x%02x)",
				     *waited, ev);
			return -ETIMEDOUT;
		}
		ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_POLL_MS);
		*waited += TRISTAR_IDBUS_POLL_MS;
	}

	ret = tristar_idbus_read_rx(ib, "read", frame, rx_len, rx_type, orientation, &verdict);
	if (ret)
		return ret;
	*frame_len = rx_len + 1;
	return verdict;
}

/*
 * Reorient (0x05 |= 0x02, VBUS-sink bit kept; the bit self-clears), then
 * wait 20 ms and poll 0x0d for up to 30 ms more for the exchange it re-arms.
 * Only 0x05 and, on a pin fault, 0x0a are written.
 */
static int tristar_idbus_reorient(struct tristar_idbus *ib, u8 *events, unsigned int *waited)
{
	unsigned int step, elapsed = TRISTAR_IDBUS_SETTLE_MS;
	int ev, ret;
	u8 old;

	ret = tristar_idbus_update_logged(ib, TRISTAR_IDBUS_REG_CTRL, 0,
					  TRISTAR_IDBUS_CTRL_REORIENT, &old);
	if (ret < 0)
		return ret;
	ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_SETTLE_MS);
	for (step = 0; ; step++) {
		ev = tristar_idbus_read_event(ib);
		if (ev < 0)
			return ev;
		*events |= ev;
		if (ev)
			ib->ops->log(ib->ctx, "%u ms after reorient: 0x0d = 0x%02x", elapsed, ev);
		if (ev & TRISTAR_IDBUS_EV_FAULT) {
			ret = tristar_idbus_ack_fault(ib);
			if (ret)
				return ret;
		}
		if (ev & (TRISTAR_IDBUS_EV_DIGITAL_ID | TRISTAR_IDBUS_EV_DONE))
			break;
		if (step == TRISTAR_IDBUS_REORIENT_STEPS) {
			ib->ops->log(ib->ctx, "no completion event %u ms after reorient", elapsed);
			break;
		}
		ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_POLL_MS);
		elapsed += TRISTAR_IDBUS_POLL_MS;
	}
	*waited += elapsed;
	return 0;
}

/*
 * After the reorient, read what the re-armed exchange left in the RX window
 * before writing anything. Accept it only if it is valid and fresh: a
 * completion event since the reorient, or bytes that differ from the frame
 * retained before this run (that one may be an older accessory's). Returns
 * 0 when accepted, 1 to send GetID again, or -errno.
 */
static int tristar_idbus_after_reorient(struct tristar_idbus *ib,
					struct tristar_idbus_id_result *res, unsigned int *waited)
{
	int ret, verdict, orientation;
	bool event;

	ret = tristar_idbus_reorient(ib, &res->reorient_events, waited);
	if (ret)
		return ret;
	ret = tristar_idbus_read_rx(ib, "after reorient", res->reorient_frame, 7,
				    TRISTAR_IDBUS_ID, &orientation, &verdict);
	if (ret)
		return ret;
	res->reorient_read = true;
	event = res->reorient_events & (TRISTAR_IDBUS_EV_DIGITAL_ID | TRISTAR_IDBUS_EV_DONE);
	if (!verdict && !event &&
	    !memcmp(res->reorient_frame, res->retained, sizeof(res->retained)))
		verdict = -ESTALE;
	res->reorient_status = verdict;
	if (verdict) {
		ib->ops->log(ib->ctx, "frame after reorient not accepted (%s); sending GetID again",
			     tristar_idbus_verdict(verdict));
		return 1;
	}
	memcpy(res->frame, res->reorient_frame, sizeof(res->frame));
	res->frame_len = sizeof(res->frame);
	res->frame_status = 0;
	res->orientation = orientation;
	res->autonomous = true;
	ib->ops->log(ib->ctx, "accepted the frame from the reorient-armed exchange (%s)",
		     event ? "completion event" : "differs from the retained frame");
	return 0;
}

/*
 * The manual GetID sequence of ds-lightning-host-idbus.md 9.2. Only the
 * reorient retry is automatic, once, and only after a timeout with VBUS.
 */
static int tristar_idbus_identify(struct tristar_idbus *ib, unsigned int flags,
				  struct tristar_idbus_id_result *res)
{
	char text[160];
	u8 tx[3] = { TRISTAR_IDBUS_GET_ID, TRISTAR_IDBUS_HOSTID >> 8, TRISTAR_IDBUS_HOSTID & 0xff };
	unsigned int i, waited;
	int ret, val, orientation;
	u8 old;

	memset(res, 0, sizeof(*res));
	ib->io_error = 0;
	ib->break_pending = false;
	res->flags = flags;
	if (flags & TRISTAR_IDBUS_SWAP_HOSTID) {
		tx[1] = TRISTAR_IDBUS_HOSTID & 0xff;
		tx[2] = TRISTAR_IDBUS_HOSTID >> 8;
	}
	ib->ops->log(ib->ctx, "identify: HOSTID bytes %02x %02x%s", tx[1], tx[2],
		     flags & TRISTAR_IDBUS_NO_REORIENT ? ", reorient disabled" : "");
	ret = tristar_idbus_check_revision(ib);
	if (ret)
		goto out;

	/* 0. Baseline, then the RX window's retained frame (read only). */
	for (i = 0; i < TRISTAR_IDBUS_NBASE; i++) {
		u8 reg = tristar_idbus_baseline_regs[i];

		val = reg == TRISTAR_IDBUS_REG_EVENT ? tristar_idbus_read_event(ib) :
						       tristar_idbus_read(ib, reg);
		if (val < 0) {
			ret = val;
			goto out;
		}
		res->baseline[i] = val;
		ib->ops->log(ib->ctx, "baseline 0x%02x = 0x%02x", reg, val);
	}
	ret = tristar_idbus_block_read(ib, TRISTAR_IDBUS_REG_RX, res->retained,
				       TRISTAR_IDBUS_ID_FRAME_LEN);
	if (ret)
		goto out;
	tristar_idbus_format_hex(res->retained, TRISTAR_IDBUS_ID_FRAME_LEN, text, sizeof(text));
	res->retained_ok = !tristar_idbus_check_frame(res->retained, 7, TRISTAR_IDBUS_ID);
	ib->ops->log(ib->ctx, "baseline RX 0x20 = [%s]%s", text,
		     res->retained_ok ? " (retained valid ID frame; may be stale)" : "");
	tristar_idbus_log_status(ib, "baseline", res->baseline[0]);
	if (ib->last_event & TRISTAR_IDBUS_EV_FAULT) {
		ret = tristar_idbus_ack_fault(ib);
		if (ret)
			goto out;
	}

	/* 1. Release a latched ID-bus reset hold. */
	if (ib->last_ctrl & TRISTAR_IDBUS_CTRL_RESET) {
		val = tristar_idbus_update_logged(ib, TRISTAR_IDBUS_REG_CTRL,
						  TRISTAR_IDBUS_CTRL_RESET, 0, &old);
		if (val < 0) {
			ret = val;
			goto out;
		}
		res->released_reset = true;
		ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_SETTLE_MS);
		if (val & TRISTAR_IDBUS_CTRL_RESET) {
			ib->ops->log(ib->ctx, "ID-bus reset hold still set; giving up");
			ret = -EBUSY;
			goto out;
		}
	}

	/* 2./3. GetID; after a timeout with VBUS, one reorient, then at most once more. */
	for (;;) {
		waited = 0;
		res->attempts++;
		ret = tristar_idbus_transfer(ib, tx, sizeof(tx), 7, TRISTAR_IDBUS_ID, true,
					     res->frame, &res->frame_len, &orientation, &waited);
		res->frame_status = ret;
		if (!ret)
			res->orientation = orientation;
		if (ret != -ETIMEDOUT || ib->io_error || res->reoriented ||
		    (flags & TRISTAR_IDBUS_NO_REORIENT))
			break;
		val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_STATUS);
		if (val < 0) {
			ret = val;
			break;
		}
		if ((val & 3) != 3) {
			ib->ops->log(ib->ctx, "timeout without VBUS (0x0c = 0x%02x): no reorient",
				     val);
			break;
		}
		res->reoriented = true;
		ret = tristar_idbus_after_reorient(ib, res, &waited);
		if (ret <= 0)
			break;
		if (waited < TRISTAR_IDBUS_CMDID_GAP_MS)
			ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_CMDID_GAP_MS - waited);
	}
	if (!ret) {
		memcpy(res->id, res->frame + 1, TRISTAR_IDBUS_ID_LEN);
		res->id_valid = true;
		tristar_idbus_format_hex(res->id, TRISTAR_IDBUS_ID_LEN, text, sizeof(text));
		ib->ops->log(ib->ctx, "digital ID %s (%s)", text,
			     res->autonomous ? "reorient-armed exchange" : "GetID command");
		tristar_idbus_format_fields(res->id, text, sizeof(text));
		ib->ops->log(ib->ctx, "%s", text);
	}
out:
	res->status = ret;
	ib->ops->log(ib->ctx, "identify %s (%d) after %u GetID command(s)%s",
		     ret ? "failed" : "ok", ret, res->attempts,
		     res->reoriented ? ", reoriented once" : "");
	return ret;
}

static unsigned int tristar_idbus_text_len(const u8 *text, unsigned int size)
{
	unsigned int len;

	for (len = 0; len < size && text[len]; len++)
		;
	return len;
}

/*
 * iOS 16.7.10 IOAccessoryManagerAID's post-GetID sequence for DigitalID
 * 11 f0: 0x76 -> 0x78 -> 0x7a, then two 0x84 SystemInfo packets. AI/AU
 * are both zero, so 0x82 and 0x80 are not sent for this adapter. No
 * authentication command is part of this path.
 *
 * IOAccessoryManagerAID advances states 0x15 (model) and 0x16 (OS version)
 * when the transport callback reports kIOReturnTimeout. The adapter normally
 * sends no response to these packets. Match that narrow exception when
 * @timeout_ok is true; a reset/detach, controller error, I/O error, or bad
 * actual response remains fatal. If the adapter does respond, the iOS states
 * do not inspect the opcode, so accept any opcode with a valid CRC and retain
 * both raw bytes.
 *
 * A timeout also runs iOS's recovery: the timer's TriStar abort (0x60 = 0x01)
 * and, because the transport marks the transfer failed, an ID-bus BREAK
 * before the next transfer. @timeout_break selects TRISTAR_IDBUS_TIMEOUT_BREAK_*:
 * OFF never, INFO only before the OS-version command, NEXT (iOS) additionally
 * before the first transfer after this function (the power handshake).
 *
 * @mask is clamped by the caller and exists only as a bounded one-boot A/B
 * control; the default is the complete iOS order.
 */
static int tristar_idbus_accessory_info(struct tristar_idbus *ib, unsigned int mask,
					const u8 *model, unsigned int model_len,
					const u8 *osversion, unsigned int osversion_len,
					bool timeout_ok, unsigned int timeout_break,
					struct tristar_idbus_accessory_result *res)
{
	u8 tx[TRISTAR_IDBUS_MAX_TX], *frame;
	unsigned int len, waited;
	int ret = 0;

	memset(res, 0, sizeof(*res));
	ib->io_error = 0;
	ib->break_pending = false;
	res->requested = mask & TRISTAR_IDBUS_INFO_ALL;
	ret = tristar_idbus_check_revision(ib);
	if (ret)
		goto out;

	if (res->requested & TRISTAR_IDBUS_INFO_DEVICE) {
		tx[0] = TRISTAR_IDBUS_GET_INTERFACE_INFO;
		waited = 0;
		ret = tristar_idbus_transfer(ib, tx, 1, 11,
					     TRISTAR_IDBUS_INTERFACE_INFO, false,
					     res->device_frame, &len, NULL, &waited);
		if (ret)
			goto out;
		memcpy(res->device_info, res->device_frame + 1, sizeof(res->device_info));
		memcpy(res->device_serial, res->device_frame + 5, sizeof(res->device_serial));
		res->completed |= TRISTAR_IDBUS_INFO_DEVICE;
		ib->ops->log(ib->ctx,
			     "interface device info %02x %02x %02x %02x; "
			     "serial %02x %02x %02x %02x %02x %02x",
			     res->device_info[0], res->device_info[1], res->device_info[2],
			     res->device_info[3], res->device_serial[0], res->device_serial[1],
			     res->device_serial[2], res->device_serial[3], res->device_serial[4],
			     res->device_serial[5]);
	}
	if (res->requested & TRISTAR_IDBUS_INFO_MODULE_SERIAL) {
		tx[0] = TRISTAR_IDBUS_GET_MODULE_SERIAL;
		waited = 0;
		ret = tristar_idbus_transfer(ib, tx, 1, 21,
					     TRISTAR_IDBUS_MODULE_SERIAL, false,
					     res->module_frame, &len, NULL, &waited);
		if (ret)
			goto out;
		memcpy(res->module_serial, res->module_frame + 1, sizeof(res->module_serial));
		res->completed |= TRISTAR_IDBUS_INFO_MODULE_SERIAL;
		len = tristar_idbus_text_len(res->module_serial, sizeof(res->module_serial));
		ib->ops->log(ib->ctx, "interface module serial [%.*s] (%u byte%s)",
			     len, (char *)res->module_serial, len, len == 1 ? "" : "s");
	}
	if (res->requested & TRISTAR_IDBUS_INFO_ACCESSORY_SERIAL) {
		tx[0] = TRISTAR_IDBUS_GET_ACCESSORY_SERIAL;
		waited = 0;
		ret = tristar_idbus_transfer(ib, tx, 1, 21,
					     TRISTAR_IDBUS_ACCESSORY_SERIAL, false,
					     res->accessory_frame, &len, NULL, &waited);
		if (ret)
			goto out;
		memcpy(res->accessory_serial, res->accessory_frame + 1,
		       sizeof(res->accessory_serial));
		res->completed |= TRISTAR_IDBUS_INFO_ACCESSORY_SERIAL;
		len = tristar_idbus_text_len(res->accessory_serial, sizeof(res->accessory_serial));
		ib->ops->log(ib->ctx, "accessory serial [%.*s] (%u byte%s)",
			     len, (char *)res->accessory_serial, len, len == 1 ? "" : "s");
	}

	if (res->requested & TRISTAR_IDBUS_INFO_MODEL) {
		if (model_len > TRISTAR_IDBUS_MAX_TX - 4)
			model_len = TRISTAR_IDBUS_MAX_TX - 4;
		tx[0] = TRISTAR_IDBUS_SYSTEM_INFO;
		tx[1] = 0;
		tx[2] = model_len >> 8;
		tx[3] = model_len;
		memcpy(tx + 4, model, model_len);
		waited = 0;
		ret = tristar_idbus_transfer(ib, tx, model_len + 4, 1, 0,
					     false, res->model_ack, &len, NULL, &waited);
		if (ret == -ETIMEDOUT && timeout_ok && !ib->io_error) {
			ret = tristar_idbus_timeout_abort(ib, timeout_break);
			if (ret)
				goto out;
			res->model_timeout = true;
			res->completed |= TRISTAR_IDBUS_INFO_MODEL;
			ib->ops->log(ib->ctx,
				     "SystemInfo model [%.*s] timed out after %u ms; iOS state 0x15 advances",
				     model_len, (char *)model, waited);
		} else if (ret) {
			goto out;
		} else {
			res->completed |= TRISTAR_IDBUS_INFO_MODEL;
			ib->ops->log(ib->ctx,
				     "SystemInfo model [%.*s], raw response %02x %02x",
				     model_len, (char *)model, res->model_ack[0],
				     res->model_ack[1]);
		}
	}
	if (res->requested & TRISTAR_IDBUS_INFO_OSVERSION) {
		if (osversion_len > TRISTAR_IDBUS_MAX_TX - 4)
			osversion_len = TRISTAR_IDBUS_MAX_TX - 4;
		tx[0] = TRISTAR_IDBUS_SYSTEM_INFO;
		tx[1] = 1;
		tx[2] = osversion_len >> 8;
		tx[3] = osversion_len;
		memcpy(tx + 4, osversion, osversion_len);
		waited = 0;
		frame = res->osversion_ack;
		ret = tristar_idbus_transfer(ib, tx, osversion_len + 4, 1, 0,
					     false, frame, &len, NULL, &waited);
		if (ret == -ETIMEDOUT && timeout_ok && !ib->io_error) {
			ret = tristar_idbus_timeout_abort(ib, timeout_break);
			if (ret)
				goto out;
			res->osversion_timeout = true;
			res->completed |= TRISTAR_IDBUS_INFO_OSVERSION;
			ib->ops->log(ib->ctx,
				     "SystemInfo OS version [%.*s] timed out after %u ms; iOS state 0x16 advances",
				     osversion_len, (char *)osversion, waited);
		} else if (ret) {
			goto out;
		} else {
			res->completed |= TRISTAR_IDBUS_INFO_OSVERSION;
			ib->ops->log(ib->ctx,
				     "SystemInfo OS version [%.*s], raw response %02x %02x",
				     osversion_len, (char *)osversion, frame[0], frame[1]);
		}
	}
out:
	if (ret || timeout_break != TRISTAR_IDBUS_TIMEOUT_BREAK_NEXT)
		ib->break_pending = false;
	res->status = ret;
	ib->ops->log(ib->ctx, "accessory identification %s (%d), requested 0x%02x completed 0x%02x",
		     ret ? "failed" : "ok", ret, res->requested, res->completed);
	return ret;
}

/*
 * Optional power handshake (ds-lightning-host-idbus.md 6): VBUS-gated 0x72
 * GetState, expecting 0x73 + 4 bytes, then one 3-byte 0x70 frame
 * {0x70, state[1] & ~8, state[2]} expecting 0x71. Which state bytes iOS
 * echoes is inferred. Same TX window as GetID; no retries.
 */
static int tristar_idbus_power_handshake(struct tristar_idbus *ib,
					 struct tristar_idbus_ph_result *res)
{
	const u8 get_state = TRISTAR_IDBUS_GET_STATE;
	unsigned int len, waited = 0;
	int ret, val;

	memset(res, 0, sizeof(*res));
	ib->io_error = 0;
	ret = tristar_idbus_check_revision(ib);
	if (ret)
		goto out;
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_STATUS);
	if (val < 0) {
		ret = val;
		goto out;
	}
	if ((val & 3) != 3) {
		ib->ops->log(ib->ctx,
			     "no VBUS detected when starting powerhandshake (0x0c = 0x%02x)", val);
		ret = -ENODEV;
		goto out;
	}
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_CTRL);
	if (val < 0) {
		ret = val;
		goto out;
	}
	if (val & TRISTAR_IDBUS_CTRL_RESET) {
		ib->ops->log(ib->ctx, "ID-bus in reset (0x05 = 0x%02x); run identify first", val);
		ret = -EBUSY;
		goto out;
	}
	ret = tristar_idbus_transfer(ib, &get_state, 1, 5, TRISTAR_IDBUS_STATE, false,
				     res->state, &len, NULL, &waited);
	if (ret)
		goto out;
	res->state_ok = true;
	res->request[0] = TRISTAR_IDBUS_POWER;
	res->request[1] = res->state[1] & ~TRISTAR_IDBUS_POWER_RETRY;
	res->request[2] = res->state[2];
	waited = 0;
	ret = tristar_idbus_transfer(ib, res->request, sizeof(res->request), 1,
				     TRISTAR_IDBUS_POWER_ACK, false, res->ack, &len, NULL, &waited);
	res->ack_ok = !ret;
out:
	res->status = ret;
	ib->ops->log(ib->ctx, "power handshake %s (%d)", ret ? "failed" : "ok", ret);
	return ret;
}

/*
 * ID-bus reset pulse (AppleCBTL1610 5ff5930): 0x05 |= 0x08, 500 ms, then
 * clear bit 3. Once the hold may have been set, the release is sent even
 * after an I/O error, with two retries: a held reset blocks every ID-bus
 * operation and must not outlive the pulse.
 */
static int tristar_idbus_reset_pulse(struct tristar_idbus *ib)
{
	int ret = 0, old, release = 0, err = 0, val, i;

	if (ib->io_error)
		return ib->io_error;
	old = ib->ops->update(ib->ctx, TRISTAR_IDBUS_REG_CTRL, 0, TRISTAR_IDBUS_CTRL_RESET);
	if (old < 0) {
		ib->ops->log(ib->ctx, "update 0x05 (set 0x08) failed: %d", old);
		ret = tristar_idbus_fail(ib, old);
	} else {
		ib->ops->log(ib->ctx, "write 0x05: before 0x%02x, wrote 0x%02x (reset hold)", old,
			     old | TRISTAR_IDBUS_CTRL_RESET);
		ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_RESET_HOLD_MS);
	}
	for (i = 0; i < 3; i++) {
		if (i)
			ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_SETTLE_MS);
		release = ib->ops->update(ib->ctx, TRISTAR_IDBUS_REG_CTRL,
					  TRISTAR_IDBUS_CTRL_RESET, 0);
		if (release >= 0)
			break;
		ib->ops->log(ib->ctx, "ID-bus reset release failed: %d", release);
		if (!err)
			err = release;
	}
	if (release < 0) {
		ib->ops->log(ib->ctx, "ID-bus reset hold may still be set; run idbus_reset again");
		return tristar_idbus_fail(ib, ret ?: release);
	}
	ib->ops->log(ib->ctx, "write 0x05: before 0x%02x, wrote 0x%02x (release)", release,
		     release & ~TRISTAR_IDBUS_CTRL_RESET);
	if (ret)
		return ret;
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_CTRL);
	if (val < 0)
		return val;
	if (val & TRISTAR_IDBUS_CTRL_RESET) {
		ib->ops->log(ib->ctx, "0x05 = 0x%02x: ID-bus reset hold still set", val);
		return -EBUSY;
	}
	ib->ops->log(ib->ctx, "0x05 after release: 0x%02x", val);
	ib->ops->sleep_ms(ib->ctx, TRISTAR_IDBUS_SETTLE_MS);
	/* Released on a retry: still report the I/O error. */
	return err ? tristar_idbus_fail(ib, err) : 0;
}

/* Read 0x0a; only if a pin fault is latched, write back exactly that value. */
static int tristar_idbus_fault_ack(struct tristar_idbus *ib,
				   struct tristar_idbus_recover_result *res)
{
	int val, ret;

	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_FAULT);
	if (val < 0)
		return val;
	res->fault = val;
	if (!(val & TRISTAR_IDBUS_FAULT_PINS)) {
		ib->ops->log(ib->ctx, "0x0a = 0x%02x: no pin fault latched; nothing written", val);
		return 0;
	}
	ret = tristar_idbus_write(ib, TRISTAR_IDBUS_REG_FAULT, val);
	if (ret)
		return ret;
	ib->faults++;
	tristar_idbus_log_fault_names(ib, "written back", val);
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_FAULT);
	if (val < 0)
		return val;
	ib->ops->log(ib->ctx, "0x0a after: 0x%02x", val);
	return 0;
}

/* Reorient again and consume the RX window read only (9.6 step 1). */
static int tristar_idbus_recover_reorient(struct tristar_idbus *ib,
					  struct tristar_idbus_recover_result *res)
{
	char hex[3 * TRISTAR_IDBUS_MAX_FRAME + 1], text[160];
	unsigned int waited = 0;
	int ret, val, orientation;

	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_CTRL);
	if (val < 0)
		return val;
	if (val & TRISTAR_IDBUS_CTRL_RESET) {
		ib->ops->log(ib->ctx, "0x05 = 0x%02x: ID-bus reset hold set; use idbus_reset", val);
		return -EBUSY;
	}
	ret = tristar_idbus_block_read(ib, TRISTAR_IDBUS_REG_RX, res->before, sizeof(res->before));
	if (ret)
		return ret;
	tristar_idbus_format_hex(res->before, sizeof(res->before), hex, sizeof(hex));
	ib->ops->log(ib->ctx, "before RX 0x20: [%s]", hex);
	ret = tristar_idbus_reorient(ib, &res->events, &waited);
	if (ret)
		return ret;
	ret = tristar_idbus_read_rx(ib, "after reorient", res->frame, 7, TRISTAR_IDBUS_ID,
				    &orientation, &res->frame_status);
	if (ret)
		return ret;
	res->frame_read = true;
	ib->ops->log(ib->ctx, "RX %s; completion event %s",
		     memcmp(res->frame, res->before, sizeof(res->frame)) ? "changed" : "unchanged",
		     res->events & (TRISTAR_IDBUS_EV_DIGITAL_ID | TRISTAR_IDBUS_EV_DONE) ?
		     "seen" : "not seen");
	if (!res->frame_status) {
		tristar_idbus_format_fields(res->frame + 1, text, sizeof(text));
		ib->ops->log(ib->ctx, "%s", text);
	}
	return 0;
}

/* The volatile link recovery levers of ds-lightning-host-idbus.md 9.6. */
static int tristar_idbus_recover(struct tristar_idbus *ib, unsigned int action,
				 struct tristar_idbus_recover_result *res)
{
	int ret, val;

	memset(res, 0, sizeof(*res));
	ib->io_error = 0;
	ib->break_pending = false;
	res->action = action;
	res->status_before = -1;
	res->status_after = -1;
	ib->ops->log(ib->ctx, "recover: %s", tristar_idbus_recover_name(action));
	if (action > TRISTAR_IDBUS_RECOVER_FAULT_ACK) {
		ret = -EINVAL;
		goto out;
	}
	ret = tristar_idbus_check_revision(ib);
	if (ret)
		goto out;
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_STATUS);
	if (val < 0) {
		ret = val;
		goto out;
	}
	res->status_before = val;
	tristar_idbus_log_status(ib, "before", val);
	if (action == TRISTAR_IDBUS_RECOVER_REORIENT)
		ret = tristar_idbus_recover_reorient(ib, res);
	else if (action == TRISTAR_IDBUS_RECOVER_RESET)
		ret = tristar_idbus_reset_pulse(ib);
	else
		ret = tristar_idbus_fault_ack(ib, res);
	if (!ib->io_error) {
		val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_STATUS);
		if (val >= 0) {
			res->status_after = val;
			tristar_idbus_log_status(ib, "after", val);
		} else if (!ret) {
			ret = val;
		}
	}
out:
	res->status = ret;
	ib->ops->log(ib->ctx, "recover %s %s (%d)", tristar_idbus_recover_name(action),
		     ret ? "failed" : "done", ret);
	return ret;
}

#endif /* APPLE_TRISTAR_IDBUS_H */

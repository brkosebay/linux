/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TriStar (CBTL1610) transport route and ACC pin configuration the iOS accessory
 * stack writes for the Lightning camera adapter, DigitalID 11 f0 00 00 00 00
 * (USB=2, UART=1, PWR=f). Pure arithmetic, no I/O, so the offline harness runs
 * the code the kernel runs.
 *
 * iOS 16.7.10 (j71t kernelcache):
 *  - IOAccessoryDock9Pin::configureTransports, fffffff005e4e57c: decodes the ID
 *    to the transports {usb0, urt0} and the ACC roles {idbs, apwr}, then swaps
 *    both pairs when the orientation is 2, and calls the route writer and
 *    setACCxConfig.
 *  - orientation, fffffff005ff4928: 0 unless 0x0c bit 7, else 1 + 0x0c bit 6.
 *  - AppleTriStarIC_configureTransportRoute_reg01, fffffff005ff4a08: dxctrl
 *    byte, low nibble = first lane pair, high nibble = second. The encoder has two
 *    override branches keyed on the digital ID (port slot +0x5a8); ID byte 0 of 11 f0
 *    has bit 4 (USB field = 2) set, which selects the orientation-dependent branch
 *    implemented here. Codes: usb0 1,
 *    urt0 2 (low) / 0x20 (high). Bit 3 / bit 7 mark a pair that differs from
 *    the default, which is USB on the oriented pair and the other pair open.
 *    A cached byte is first normalised to that default, and when a pair changes
 *    an intermediate byte with the pair's override bit is written before the
 *    final one.
 *  - AppleCBTL1610::setACCxConfig, fffffff005fee000: ACC_CTRL = register 0x02,
 *    ACC1 nibble idbs 0 / urt1 1 / jtag 2 / apwr 3, ACC2 the same << 4.
 *
 * iOS captured on this adapter in orientation 1: ACC_CTRL=0x30 and
 * reset(dxctrl:0xa1).
 */
#ifndef APPLE_S8000_TRISTAR_ROUTE_H
#define APPLE_S8000_TRISTAR_ROUTE_H

#define TRISTAR_ROUTE_STATUS_PRESENT	0x80
#define TRISTAR_ROUTE_STATUS_ORIENT2	0x40

#define TRISTAR_ROUTE_ACC_O1		0x30	/* ACC1 idbs, ACC2 apwr */
#define TRISTAR_ROUTE_ACC_O2		0x03	/* ACC1 apwr, ACC2 idbs */
#define TRISTAR_ROUTE_DX_O1		0xa1	/* usb0 | urt0 << 4 | override of pair 2 */
#define TRISTAR_ROUTE_DX_O2		0x1a	/* urt0 | usb0 << 4 | override of pair 1 */

struct tristar_route_plan {
	unsigned int orientation;	/* 0 no plug, 1 or 2 */
	u8 acc;				/* ACC_CTRL, register 0x02 */
	u8 dx_first;			/* intermediate dxctrl, register 0x01 */
	u8 dx_target;			/* final dxctrl */
	bool dx_write;			/* register 0x01 differs from the target */
	bool dx_two_steps;		/* the intermediate byte is written first */
};

static inline unsigned int tristar_route_orientation(u8 status)
{
	if (!(status & TRISTAR_ROUTE_STATUS_PRESENT))
		return 0;
	return status & TRISTAR_ROUTE_STATUS_ORIENT2 ? 2 : 1;
}

/* @status is 0x0c and @dx_cur the current register 0x01. Returns false without a plug. */
static inline bool tristar_route_plan_for(u8 status, u8 dx_cur, struct tristar_route_plan *plan)
{
	u8 norm = dx_cur, lo, hi;

	plan->orientation = tristar_route_orientation(status);
	if (!plan->orientation)
		return false;
	plan->acc = plan->orientation == 1 ? TRISTAR_ROUTE_ACC_O1 : TRISTAR_ROUTE_ACC_O2;
	plan->dx_target = plan->orientation == 1 ? TRISTAR_ROUTE_DX_O1 : TRISTAR_ROUTE_DX_O2;
	plan->dx_write = dx_cur != plan->dx_target;
	/* A cached zero means the default: USB on the oriented pair. */
	if (plan->orientation == 1) {
		if (!(norm & 0x08))
			norm = (norm & ~0x07) | 0x01;
	} else if (!(norm & 0x80)) {
		norm = (norm & ~0x70) | 0x10;
	}
	lo = (plan->dx_target ^ norm) & 0x07 ? 0x08 : plan->dx_target & 0x0f;
	hi = (plan->dx_target ^ norm) & 0x70 ? 0x80 : plan->dx_target & 0xf0;
	plan->dx_first = lo | hi;
	plan->dx_two_steps = plan->dx_write && plan->dx_first != plan->dx_target;
	return true;
}

#endif /* APPLE_S8000_TRISTAR_ROUTE_H */

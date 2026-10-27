/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Apple S8000 (A9) Lightning role switch: host role owned by an accessory.
 *
 * The Lightning accessory policy (apple_tristar_power) claims the next host
 * entry before it calls usb_role_switch_set_role(). A claimed host role has
 * no revert timeout and uses the accessory power gates; it ends when the
 * policy, or anyone else, sets the device role.
 */
#ifndef __LINUX_USB_APPLE_S8000_ROLE_H
#define __LINUX_USB_APPLE_S8000_ROLE_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct usb_role_switch;

#if IS_REACHABLE(CONFIG_USB_ROLES_APPLE_S8000)
int apple_s8000_usb_role_monitor(struct usb_role_switch *sw,
				 int (*sample)(void *data), void *data);
int apple_s8000_usb_role_claim(struct usb_role_switch *sw, bool claim);
bool apple_s8000_usb_role_accessory(struct usb_role_switch *sw);
#else
static inline int apple_s8000_usb_role_monitor(struct usb_role_switch *sw,
					       int (*sample)(void *data), void *data)
{
	return -EOPNOTSUPP;
}

static inline int apple_s8000_usb_role_claim(struct usb_role_switch *sw, bool claim)
{
	return -EOPNOTSUPP;
}

static inline bool apple_s8000_usb_role_accessory(struct usb_role_switch *sw)
{
	return false;
}
#endif

#endif /* __LINUX_USB_APPLE_S8000_ROLE_H */

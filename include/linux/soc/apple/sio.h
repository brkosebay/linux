/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_SIO_H_
#define _LINUX_SOC_APPLE_SIO_H_

#include <linux/types.h>

struct apple_sio;
struct apple_sio_async;

/* Endpoint numbers are 8 bits; the firmware names 0x00..0x81. */
#define APPLE_SIO_MAX_ENDPOINTS		0x82

/*
 * The firmware keeps 32 response slots per endpoint and indexes them by
 * tag - 1 (AppleSmartIODMAController::responseForEndpointAndTag), so tags
 * are allocated per endpoint in 1..APPLE_SIO_MAX_TAG.
 */
#define APPLE_SIO_MAX_TAG		31

/*
 * Shim configuration of a DMA endpoint (op 5). These are the words of an
 * Apple ADT "dma-channels" record after the endpoint number, as sent by
 * AppleSmartIODMA::_configureDMA. DMA clients pass it through
 * dma_slave_config.peripheral_config; @limit 0 means "derive from the
 * slave config's burst".
 */
struct apple_sio_dma_config {
	u32 xfer;	/* transfer control, e.g. 0x21 16-bit / 0x22 32-bit (MCA) */
	u32 timeout;
	u32 fifo;	/* shim FIFO, bytes */
	u32 trigger;	/* shim trigger level, bytes */
	u32 limit;	/* client FIFO depth, bytes (setFIFODepth) */
	u32 word5;	/* ADT word 5 */
	u32 frame;	/* setFrameSize, 0 unless the client sets it */
	u32 word7;	/* ADT word 6 */
};

typedef void (*apple_sio_async_callback_t)(void *data, int status,
					   u64 response);
typedef void (*apple_sio_notify_t)(void *data, u64 message);

void *apple_sio_desc0_alloc(struct apple_sio *sio, unsigned int slots,
			    u32 *first_slot);
void apple_sio_desc0_free(struct apple_sio *sio, u32 first_slot,
			  unsigned int slots);

struct apple_sio_async *
apple_sio_request_async(struct apple_sio *sio, u8 endpoint, u8 opcode,
			u8 parameter, u32 data, u8 expected_opcode,
			apple_sio_async_callback_t callback,
			void *callback_data);
struct apple_sio_async *
apple_sio_request_async_atomic(struct apple_sio *sio, u8 endpoint, u8 opcode,
			       u8 parameter, u32 data, u8 expected_opcode,
			       apple_sio_async_callback_t callback,
			       void *callback_data);
int apple_sio_async_cancel(struct apple_sio_async *request);
void apple_sio_async_put(struct apple_sio_async *request);

int apple_sio_request(struct apple_sio *sio, u8 endpoint, u8 opcode,
		      u8 parameter, u32 data, u8 expected_opcode,
		      unsigned long timeout, u64 *response);

/* Fail every outstanding request on @endpoint with -ECANCELED. */
void apple_sio_cancel_endpoint(struct apple_sio *sio, u8 endpoint);

/* Messages on @endpoint that match no pending request (IRQ context). */
void apple_sio_set_notify(struct apple_sio *sio, u8 endpoint,
			  apple_sio_notify_t notify, void *data);

#endif

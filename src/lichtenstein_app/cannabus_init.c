/*
 * cannabus_init.c
 *
 * Various initialization routines for CANnabus.
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */
#include "cannabus_init.h"
#include "cannabus_init_private.h"

#include "cannabus_lichtenstein.h"

#include "lichtenstein.h"

#include "../hw/nvram.h"
#include "../periph/canbus.h"
#include "../cannabus/cannabus.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * Callbacks for CANnabus IO
 */
const cannabus_callbacks_t kCannabusCallbacks = {
	.can_init = lichtenstein_cannabus_can_init,
	.can_config_filter = lichtenstein_cannabus_can_config_filter,
	.can_rx_waiting = lichtenstein_cannabus_can_rx_waiting,
	.can_rx_message = lichtenstein_cannabus_can_rx_message,
	.can_tx_message = lichtenstein_cannabus_can_tx_message,

	.handle_operation = lichtenstein_cannabus_cb,
};



/**
 * CANnabus callback: initializes CAN bus.
 */
int lichtenstein_cannabus_can_init(void) {
	can_start();
	return 0;
}
/**
 * CANnabus callback: configures a mask-based filter.
 */
int lichtenstein_cannabus_can_config_filter(unsigned int filter, uint32_t mask, uint32_t identifier) {
	return can_filter_mask(filter, mask, identifier);
}
/**
 * CANnabus callback: are there any messages waiting?
 */
bool lichtenstein_cannabus_can_rx_waiting(void) {
	return can_messages_available();
}
/**
 * CANnabus callback: receives a message from CAN peripheral.
 */
int lichtenstein_cannabus_can_rx_message(cannabus_can_frame_t *frame) {
	int err;

	// receive message from CAN peripheral
	can_message_t rawFrame;

	err = can_get_last_message(&rawFrame);

	if(err < kErrSuccess) {
		return err;
	}

	// copy fields from the message
	frame->identifier = rawFrame.identifier;
	frame->rtr = rawFrame.rtr;
	frame->data_len = rawFrame.length;

	memcpy(&frame->data, &rawFrame.data, 8);

//	LOG("CANnabus rx from %x, %u bytes\n", rawFrame.identifier, rawFrame.length);

	// success!
	return kErrSuccess;
}
/**
 * CANnabus callback: transmits a message.
 */
int lichtenstein_cannabus_can_tx_message(cannabus_can_frame_t *frame) {
	int err;

//	LOG("CANnabus tx to %x, %u bytes\n", frame->identifier, frame->data_len);

	// create a CAN driver frame structure
	can_message_t rawFrame;
	memset(&rawFrame, 0, sizeof(rawFrame));

	rawFrame.valid = 1;
	rawFrame.identifier = frame->identifier;
	rawFrame.rtr = frame->rtr;
	rawFrame.length = frame->data_len;

	memcpy(&rawFrame.data, &frame->data, 8);

	// transmit the frame
	err = can_transmit_message(&rawFrame);
	return err;
}



/**
 * Initializes CANnabus communication.
 */
void lichtenstein_cannabus_init(void) {
	int err;

	// get the node id from nvram
	nvram_t *nvram = nvram_get();
	uint16_t busId = nvram->cannabusId;

	// get testing bus id for nucleo board
#ifdef STM32F072
	busId = 0xDEAD;
#endif

	// initialize bus
	err = cannabus_init(busId, 0x0B, &kCannabusCallbacks);

	if(err < kErrSuccess) {
		LOG("cannabus init failed: %d\n", err);
	}
}

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
#include "../hw/spi_flash.h"
#include "../periph/canbus.h"
#include "../cannabus/cannabus.h"
#include "../util/crc.h"

#include "version.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/// internal state for the Lichtenstein CANnabus driver
static lichtenstein_cannabus_state_t gState;



/**
 * Callbacks for CANnabus IO
 */
const cannabus_callbacks_t kCannabusCallbacks = {
	.can_init = controller_cannabus_can_init,
	.can_config_filter = controller_cannabus_can_config_filter,
	.can_rx_waiting = controller_cannabus_can_rx_waiting,
	.can_rx_message = controller_cannabus_can_rx_message,
	.can_tx_message = lichtenstein_cannabus_can_tx_message,

	.get_fw_version = controller_cannabus_get_fw_version,

	.upgrade_begin = lichtenstein_cannabus_upgrade_begin,
	.upgrade_write = lichtenstein_cannabus_upgrade_write,
	.upgrade_end = lichtenstein_cannabus_upgrade_end,
	.upgrade_reset = lichtenstein_cannabus_upgrade_reset,

	.handle_operation = lichtenstein_cannabus_cb,
};



/**
 * CANnabus callback: initializes CAN bus.
 */
int controller_cannabus_can_init(void) {
	// clear the internal state
	memset(&gState, 0, sizeof(gState));

	// start CAN peripheral
	can_start();
	return 0;
}
/**
 * CANnabus callback: configures a mask-based filter.
 */
int controller_cannabus_can_config_filter(unsigned int filter, uint32_t mask, uint32_t identifier) {
	return can_filter_mask(filter, mask, identifier);
}
/**
 * CANnabus callback: are there any messages waiting?
 */
bool controller_cannabus_can_rx_waiting(void) {
	return can_messages_available();
}
/**
 * CANnabus callback: receives a message from CAN peripheral.
 */
int controller_cannabus_can_rx_message(cannabus_can_frame_t *frame) {
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
void controller_cannabus_init(void) {
	int err;

	// get the node id from nvram
	nvram_t *nvram = nvram_get();
	uint16_t busId = nvram->cannabusId;

	// get testing bus id for nucleo board
#ifdef STM32F072
	busId = 0xDEAD;
#endif

	// initialize bus
	err = cannabus_init(busId, 0x0B, (0b10000000 | 15), &kCannabusCallbacks);

	if(err < kErrSuccess) {
		LOG("cannabus init failed: %d\n", err);
	}
}



/**
 * CANnabus callback: begins a firmware upgrade session. This finds a blank page
 * in the loader flash/the oldest firmware, erases it, and prepares the internal
 * state.
 */
int lichtenstein_cannabus_upgrade_begin(uint16_t crc) {
	int err;

	// copy CRC and reset CRC accumulator
	gState.expectedCRC = crc;
	gState.currentCRC = 0xFFFF;

	// TODO: find a free firmware upgrade bank in flash
	gState.flashAddr = 0x8000;

	// erase 32K of memory (one firmware image)
	err = spiflash_erase(0x7FFF, gState.flashAddr);

	return err;
}

/**
 * CANnabus callback: writes data to the firmware flash. Data is provided in
 * chunks of 8 bytes, so they will never cross the boundary of a 256 byte page.
 */
int lichtenstein_cannabus_upgrade_write(size_t numBytes, void *data) {
	int err;

	// update the CRC
	gState.currentCRC = crc16(data, numBytes, gState.currentCRC);

	// write to the flash
	err = spiflash_write(numBytes, data, gState.flashAddr);

	// increment flash address
	gState.flashAddr += numBytes;

	return err;
}

/**
 * CANnabus callback: finishes a firmware upgrade session. The CRC of existing
 * data is validated, and if it is correct, the loader info block is updated to
 * boot this firmware.
 */
int lichtenstein_cannabus_upgrade_end(void) {
	// abort if CRC doesn't match
	if(gState.expectedCRC != gState.currentCRC) {
		return kErrCannabusCRCInvalid;
	}

	// TODO: mark firmware as good in loader
	return kErrUnimplemented;
}

/**
 * CANnabus callback: resets the device after a firmware upgrade.
 */
int lichtenstein_cannabus_upgrade_reset(void) {
	NVIC_SystemReset();

	// unreachable
	return kErrSuccess;
}



/**
 * CANnabus callback: returns the device firmware version.
 */
uint16_t controller_cannabus_get_fw_version(void) {
	return kLichtensteinVersion.version;
}

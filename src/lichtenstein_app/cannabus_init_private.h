/*
 * cannabus_init_private.h
 *
 * Prototypes of Cannabus callbacks
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */

#ifndef LICHTENSTEIN_APP_CANNABUS_INIT_PRIVATE_H_
#define LICHTENSTEIN_APP_CANNABUS_INIT_PRIVATE_H_

#include "../cannabus/cannabus.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * Lichtenstein CANnabus internal state: this is mostly used to support the
 * firmware upgrade facility.
 */
typedef struct {
	/// expected CRC of the firmware
	uint16_t expectedCRC;
	/// current CRC
	uint16_t currentCRC;

	/// flash address counter
	uint32_t flashAddr;
} lichtenstein_cannabus_state_t;

/**
 * CANnabus callback: initializes CAN bus.
 */
int lichtenstein_cannabus_can_init(void);
/**
 * CANnabus callback: configures a mask-based filter.
 */
int lichtenstein_cannabus_can_config_filter(unsigned int filter, uint32_t mask, uint32_t identifier);
/**
 * CANnabus callback: are there any messages waiting?
 */
bool lichtenstein_cannabus_can_rx_waiting(void);
/**
 * CANnabus callback: receives a message from CAN peripheral.
 */
int lichtenstein_cannabus_can_rx_message(cannabus_can_frame_t *frame);
/**
 * CANnabus callback: transmits a message.
 */
int lichtenstein_cannabus_can_tx_message(cannabus_can_frame_t *frame);



/**
 * CANnabus callback: begins a firmware upgrade session. This finds a blank page
 * in the loader flash/the oldest firmware, erases it, and prepares the internal
 * state.
 */
int lichtenstein_cannabus_upgrade_begin(uint16_t crc);
/**
 * CANnabus callback: writes data to the firmware flash. Data is provided in
 * chunks of 8 bytes, so they will never cross the boundary of a 256 byte page.
 */
int lichtenstein_cannabus_upgrade_write(size_t numBytes, void *data);
/**
 * CANnabus callback: finishes a firmware upgrade session. The CRC of existing
 * data is validated, and if it is correct, the loader info block is updated to
 * boot this firmware.
 */
int lichtenstein_cannabus_upgrade_end(void);
/**
 * CANnabus callback: resets the device after a firmware upgrade.
 */
int lichtenstein_cannabus_upgrade_reset(void);

/**
 * CANnabus callback: returns the device firmware version.
 */
uint16_t lichtenstein_cannabus_get_fw_version(void);

#endif /* LICHTENSTEIN_APP_CANNABUS_INIT_PRIVATE_H_ */

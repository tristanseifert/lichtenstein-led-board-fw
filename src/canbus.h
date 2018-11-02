/*
 * canbus.h
 *
 * Provides an interface to the CAN bus. This driver is mostly interrupt and
 * DMA driven, and maintains an internal queue of messages that have been
 * received, so reception happens in the background.
 *
 * Transmitting of messages is blocking and will not return until the message
 * has been handed over to the CAN peripheral, blocking if there are no transmit
 * FIFO slots available.
 *
 * Filtering is possible by specifying one exact match, and one mask filter.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef CANBUS_H_
#define CANBUS_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * A single CAN message.
 */
typedef struct {
	/// is this a valid message?
	uint32_t valid		: 1;

	/// CAN identifier: only the low 29 bits are used.
	uint32_t identifier	: 29;
	// is this message a remote transmission request?
	uint32_t rtr		: 1;

	/// data length
	uint8_t length;
	/// data
	char data[8];
} canbus_message_t;



/**
 * Initializes the CAN peripheral.
 */
void canbus_init(void);

/**
 * Starts the CAN driver.
 */
void canbus_start(void);

/**
 * Stops the CAN driver.
 */
void canbus_stop(void);



/**
 * Configures the exact identifier filter.
 *
 * @note Only the low 29 bits are considered.
 */
void canbus_filter_exact(uint32_t identifier);

/**
 * Configures the mask identifier filter.
 *
 * @note Only the low 29 bits of the identifier and mask are considered.
 */
void canbus_filter_mask(uint32_t mask, uint32_t identifier);


/**
 * Are there any messages waiting to be read?
 */
bool canbus_messages_available(void);

/**
 * Were messages dropped since the last invocation of this function?
 */
bool canbus_messages_dropped(void);

/**
 * Copies the oldest message to the specified buffer, then removes it from the
 * internal queue.
 */
int canbus_get_last_message(canbus_message_t *msg);



/**
 * Transmits the given message.
 */
int canbus_transmit_message(canbus_message_t *msg);



#endif /* CANBUS_H_ */

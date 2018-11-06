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

#endif /* LICHTENSTEIN_APP_CANNABUS_INIT_PRIVATE_H_ */

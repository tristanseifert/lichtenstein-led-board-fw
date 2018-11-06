/*
 * canbus_private.h
 *
 * Private internal functions for the CAN bus.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef CANBUS_PRIVATE_H_
#define CANBUS_PRIVATE_H_

#include "canbus.h"

/**
 * Declare prototype for the IRQ
 */
void CEC_CAN_IRQHandler(void);



/**
 * Reads a message from the specified FIFO.
 */
void can_read_fifo(int fifo);

/**
 * Checks whether the receive FIFOs were overrun.
 */
void can_check_fifo_overrun(void);



/**
 * Finds the next available transmit mailbox.
 */
int can_find_free_tx_mailbox(void);

/**
 * Transmits the given message on the given mailbox.
 */
int can_tx_with_mailbox(int mailbox, can_message_t *msg);



#endif /* CANBUS_PRIVATE_H_ */
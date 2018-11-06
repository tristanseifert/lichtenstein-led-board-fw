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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/// size of the RX buffer
#define kCANRxBufferSize			8
/// size of the CAN bus driver stack (words)
#define kCANStackSize				100

/**
 * Internal state of the CAN bus driver
 */
typedef struct {
	/// receive buffer
	can_message_t rxBuffer[kCANRxBufferSize];

	/// number of dropped messages
	unsigned int numDroppedMessages;

	/// FreeRTOS task handle
	TaskHandle_t task;
	/// task control block
	StaticTask_t taskTCB;
	/// stack for task
	StackType_t taskStack[kCANStackSize];

	/// receive buffer semaphore, used to block until frames receive
	SemaphoreHandle_t rxSemaphore;
	/// receive buffer semaphore struct
	StaticSemaphore_t rxSemaphoreStruct;
} can_state_t;

/**
 * Entry point for the CAN bus task.
 *
 * This task primarily handles receiving frames.
 */
void can_task(void);



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

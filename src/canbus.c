/*
 * canbus.c
 *
 * CAN_RX:	PA11
 * CAN_TX:	PA12
 *
 * BTR register value calculated with http://www.bittiming.can-wiki.info/
 *
 * TODO: implement reception (lol)
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "canbus.h"
#include "canbus_private.h"

#include "lichtenstein.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/**
 * Queue of received messages
 */
#define kRxBufferSize	8

static canbus_message_t rxBuffer[kRxBufferSize];

/// set when a message is received yet the queue is full
bool droppedMessages = false;


/**
 * Initializes the CAN peripheral.
 */
void canbus_init(void) {
	// clear the receive buffer
	memset(&rxBuffer, 0, sizeof(rxBuffer));

	// enable GPIO/SYSCFG clocks, then remap PA11/PA12 to the pins
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	// configure PA11 and PA12 as high speed alternate function outputs
	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12);

	// enable CAN clock and reset
	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	RCC->APB1RSTR |= RCC_APB1RSTR_CANRST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;

	// enter initialization mode
	CAN->MCR |= CAN_MCR_INRQ;
	while(!(CAN->MSR & CAN_MSR_INAK)) {}

	// exit sleep mode
	CAN->MCR &= (uint32_t) ~CAN_MCR_SLEEP;

	/*
	 * Configure CAN master control:
	 *
	 * - Automatic bus-off management
	 * - Automatic wake-up mode
	 * - Discard messages if FIFO is full
	 *
	 */
	CAN->MCR |= CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_RFLM;

	// configure CAN bit timing for 125kbps nominal
	CAN->BTR = 0x001c0017;
}

/**
 * Starts the CAN driver.
 */
void canbus_start(void) {
	// exit initialization mode; wait for INAK clear
	CAN->MCR &= (uint32_t) ~CAN_MCR_INRQ;
	while((CAN->MSR & CAN_MSR_INAK)) {}

	// we should be good now, wait for the interrupts to come in
}


/**
 * Stops the CAN driver.
 */
void canbus_stop(void) {
	// re-enter initialization mode
	CAN->MCR |= CAN_MCR_INRQ;
	while(!(CAN->MSR & CAN_MSR_INAK)) {}
}



/**
 * Configures the exact identifier filter.
 *
 * This is done with filter bank 0. We configure it in mask mode such that we
 * get the message regardless of the RTR bit.
 *
 * Filtered messages are dropped into FIFO0.
 *
 * @note Only the low 29 bits are considered.
 */
void canbus_filter_exact(uint32_t identifier) {
	// enable filter initialization mode, deactivate filter 0
	CAN->FMR |= CAN_FMR_FINIT;
	CAN->FA1R &= (uint32_t) ~CAN_FA1R_FACT0;


	// filter is configured as a single 32-bit filter in mask mode
	CAN->FS1R |= CAN_FS1R_FSC0;
	CAN->FM1R &= (uint32_t) ~CAN_FM1R_FBM0;

	// set the mask (high 29 bits, then IDE is 1)
	CAN->sFilterRegister[0].FR2 = 0xFFFFFFFC;
	// copy low 29 bits of address, shift left 3, then add IDE bit (0x04)
	CAN->sFilterRegister[0].FR1 = ((identifier & 0x1FFFFFFF) << 3) | 0x04;

	// Use FIFO 0
	CAN->FFA1R &= (uint32_t) ~CAN_FFA1R_FFA0;

	// activate filter, then disable filter init mode
	CAN->FA1R |= CAN_FA1R_FACT0;
	CAN->FMR &= (uint32_t) ~CAN_FMR_FINIT;
}

/**
 * Configures the mask identifier filter.
 *
 * @note Only the low 29 bits of the identifier and mask are considered.
 */
void canbus_filter_mask(uint32_t mask, uint32_t identifier) {
	// enable filter initialization mode, deactivate filter 1
	CAN->FMR |= CAN_FMR_FINIT;
	CAN->FA1R &= (uint32_t) ~CAN_FA1R_FACT1;


	// filter is configured as a single 32-bit filter in mask mode
	CAN->FS1R |= CAN_FS1R_FSC0;
	CAN->FM1R &= (uint32_t) ~CAN_FM1R_FBM0;

	// set the mask (high 29 bits, then IDE is 1)
	CAN->sFilterRegister[0].FR2 = ((mask & 0x1FFFFFFF) << 3) | 0x04;
	// copy low 29 bits of address, shift left 3, then add IDE bit (0x04)
	CAN->sFilterRegister[0].FR1 = ((identifier & 0x1FFFFFFF) << 3) | 0x04;

	// Use FIFO 0
	CAN->FFA1R &= (uint32_t) ~CAN_FFA1R_FFA0;

	// activate filter, then disable filter init mode
	CAN->FA1R |= CAN_FA1R_FACT1;
	CAN->FMR &= (uint32_t) ~CAN_FMR_FINIT;
}


/**
 * Are there any messages waiting to be read?
 */
bool canbus_messages_available(void) {
	// check the entire receive queue
	for(int i = 0; i < kRxBufferSize; i++) {
		// is this message valid?
		if(rxBuffer[i].valid) {
			// then yes, there are messages waiting
			return true;
		}
	}

	// if we get here, no messages available
	return false;
}

/**
 * Were messages dropped since the last invocation of this function?
 */
bool canbus_messages_dropped(void) {
	// get state and reset it
	bool state = droppedMessages;
	droppedMessages = false;

	return state;
}

/**
 * Copies the oldest message to the specified buffer, then removes it from the
 * internal queue.
 */
int canbus_get_last_message(canbus_message_t *msg) {
	// buffer cannot be null
	if(msg == NULL) {
		return -1;
	}

	// find an available message
	for(int i = 0; i < kRxBufferSize; i++) {
		// is this message valid?
		if(rxBuffer[i].valid) {
			// copy message
			memcpy(msg, &rxBuffer[i], sizeof(canbus_message_t));

			// mark that slot as available, return index of message
			rxBuffer[i].valid = 0;

			return i;
		}
	}

	// no messages were available :(
	return -1;
}



/**
 * Transmits the given message.
 */
int canbus_transmit_message(canbus_message_t *msg) {
	// find a free transmit mailbox
	int box = canbus_find_free_tx_mailbox();

	if(box < 0) {
		return -1;
	}

	// attempt to transmit message into that mailbox
	return canbus_tx_with_mailbox(box, msg);
}



/**
 * Finds the next available transmit mailbox.
 *
 * Currently, we only use mailbox 0.
 */
int canbus_find_free_tx_mailbox(void) {
	while(true) {
		// is mailbox 0 empty?
		if((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			return 0;
		}
		// is mailbox 1 empty?
		else if((CAN->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
			return 1;
		}
		// is mailbox 2 empty?
		else if((CAN->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
			return 2;
		}

		// TODO: timeout in case the mailboxes are fucked
	}

	// no mailbox could be found that's free
	return -1;
}

/**
 * Transmits the given message on the given mailbox.
 */
int canbus_tx_with_mailbox(int mailbox, canbus_message_t *msg) {
	uint32_t reqAckFlag, txOkFlag;

	// get the right flags
	switch(mailbox) {
	case 0:
		reqAckFlag = CAN_TSR_RQCP0;
		txOkFlag = CAN_TSR_TXOK0;
		break;
	case 1:
		reqAckFlag = CAN_TSR_RQCP1;
		txOkFlag = CAN_TSR_TXOK1;
		break;
	case 2:
		reqAckFlag = CAN_TSR_RQCP2;
		txOkFlag = CAN_TSR_TXOK2;
		break;
	}

	// set the address into the mailbox; use identifier extension, data frame
	CAN->sTxMailBox[mailbox].TIR = ((msg->identifier & 0x1FFFFFFFUL) << 3) | 0x04;

	// set data length
	CAN->sTxMailBox[mailbox].TDTR = msg->length;

	// copy data
	memcpy((uint32_t *) &CAN->sTxMailBox[mailbox].TDLR, &msg->data[0], 4);
	memcpy((uint32_t *) &CAN->sTxMailBox[mailbox].TDHR, &msg->data[4], 4);

	// request transmission of mailbox 0, and wait for request acknowledgement
	CAN->sTxMailBox[mailbox].TIR |= 0x00000001;
	while(!(CAN->TSR & reqAckFlag)) {}

	// wait for the message to have been transmitted successfully
	// TODO: timeout if CAN physical layer got fucked
	while(!(CAN->TSR & txOkFlag)) {}
	return 0;
}

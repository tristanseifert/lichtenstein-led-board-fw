/*
 * canbus.c
 *
 * On the STM32F042 based hardware:
 * CAN_RX:	PA11
 * CAN_TX:	PA12
 *
 * On the STM32F072 Nucleo64 board:
 * CAN_RX:	PB8
 * CAN_TX:	PB9
 *
 * BTR register value calculated with http://www.bittiming.can-wiki.info/
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

static can_message_t rxBuffer[kRxBufferSize];

/// set when a message is received yet the queue is full
bool droppedMessages = false;


/**
 * Initializes the CAN peripheral.
 */
void can_init(void) {
	// clear the receive buffer
	memset(&rxBuffer, 0, sizeof(rxBuffer));

#ifdef STM32F042
	// enable GPIO, SYSCFG clocks, then remap PA11/PA12 to the pins
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	// configure PA11 and PA12 as high speed alternate function outputs
	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12);

	// configure PA11 and PA12 alternate functions
	GPIOB->AFR[1] |= 0x04 << (3 * 4);
	GPIOB->AFR[1] |= 0x04 << (4 * 4);
#endif
#ifdef STM32F072
	// enable GPIO clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// configure PB8 and PB9 as high speed alternate function outputs
	GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);

	// configure PB8 and PB9 alternate functions
	GPIOB->AFR[1] |= 0x04 << (0 * 4);
	GPIOB->AFR[1] |= 0x04 << (1 * 4);
#endif

	// enable CAN clock and reset
	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	RCC->APB1RSTR |= RCC_APB1RSTR_CANRST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;

	// exit sleep mode
	CAN->MCR &= (uint32_t) ~CAN_MCR_SLEEP;

	// enter initialization mode
	CAN->MCR |= CAN_MCR_INRQ;
	while(!(CAN->MSR & CAN_MSR_INAK)) {}

	/*
	 * Configure CAN master control:
	 *
	 * - Automatic bus-off management
	 * - Automatic wake-up mode
	 * - Discard messages if FIFO is full
	 * - No automatic retransmissions
	 *
	 */
	CAN->MCR |= CAN_MCR_ABOM | CAN_MCR_AWUM/* | CAN_MCR_RFLM *//*| CAN_MCR_NART*/;

	/**
	 * Enable CAN interrupts:
	 *
	 * - Error interrupt
	 * - Bus off interrupt
	 * - Error passive interrupt
	 * - FIFO full
	 * - FIFO message pending
	 */
	CAN->IER |= CAN_IER_ERRIE | CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_FFIE0 | CAN_IER_FMPIE0;

	// unmask interrupts
	NVIC_EnableIRQ(CEC_CAN_IRQn);
	NVIC_SetPriority(CEC_CAN_IRQn, 2);

	// configure CAN bit timing for 125kbps nominal
	CAN->BTR = 0x001c0017;
}

/**
 * Starts the CAN driver.
 */
void can_start(void) {
	// exit initialization mode; wait for INAK clear
	CAN->MCR &= (uint32_t) ~CAN_MCR_INRQ;
	while((CAN->MSR & CAN_MSR_INAK)) {}

	// we should be good now, wait for the interrupts to come in
}


/**
 * Stops the CAN driver.
 */
void can_stop(void) {
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
int can_filter_exact(unsigned int bank, uint32_t identifier) {
	return can_filter_mask(bank, 0x1FFFFFFF, identifier);
}

/**
 * Configures the mask identifier filter.
 *
 * @param bank The bank to configure: this should be [1, 14].
 *
 * @note Only the low 29 bits of the identifier and mask are considered.
 */
int can_filter_mask(unsigned int _bank, uint32_t mask, uint32_t identifier) {
	// make bank zero based, validate it, then get the bit flag for config regs
	unsigned int bank = _bank - 1;

	if(bank > 13) {
//		LOG("invalid bank: %u\n", bank);
		return kErrInvalidArgs;
	}

	uint32_t bankFlag = 1UL << (bank);

	// enable filter initialization mode, deactivate filter
	CAN->FMR |= CAN_FMR_FINIT;
	CAN->FA1R &= (uint32_t) ~bankFlag;


	// filter is configured as a single 32-bit filter in mask mode
	CAN->FS1R |= bankFlag;
	CAN->FM1R &= (uint32_t) ~bankFlag;

	// set the mask (high 29 bits, then IDE is 1)
	CAN->sFilterRegister[bank].FR2 = ((mask & 0x1FFFFFFF) << 3) | 0x04;
	// copy low 29 bits of address, shift left 3, then add IDE bit (0x04)
	CAN->sFilterRegister[bank].FR1 = ((identifier & 0x1FFFFFFF) << 3) | 0x04;

	// Use FIFO 0
	CAN->FFA1R &= (uint32_t) ~bankFlag;

	// activate filter, then disable filter init mode
	CAN->FA1R |= bankFlag;
	CAN->FMR &= (uint32_t) ~CAN_FMR_FINIT;

	// success
	return kErrSuccess;
}


/**
 * Are there any messages waiting to be read?
 */
bool can_messages_available(void) {
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
bool can_messages_dropped(void) {
	// get state and reset it
	bool state = droppedMessages;
	droppedMessages = false;

	return state;
}

/**
 * Copies the oldest message to the specified buffer, then removes it from the
 * internal queue.
 */
int can_get_last_message(can_message_t *msg) {
	// buffer cannot be null
	if(msg == NULL) {
		return kErrInvalidArgs;
	}

	// find an available message
	for(int i = 0; i < kRxBufferSize; i++) {
		// is this message valid?
		if(rxBuffer[i].valid) {
			// copy message
			memcpy(msg, &rxBuffer[i], sizeof(can_message_t));

			// mark that slot as available, return index of message
//			rxBuffer[i].valid = 0;
			memset(&rxBuffer[i], 0, sizeof(can_message_t));

			return i;
		}
	}

	// no messages were available :(
	return kErrCanNoRxFramesPending;
}



/**
 * Transmits the given message.
 */
int can_transmit_message(can_message_t *msg) {
	// find a free transmit mailbox
	int box = can_find_free_tx_mailbox();

	if(box < 0) {
		return kErrCanNoFreeMailbox;
	}

	// attempt to transmit message into that mailbox
	return can_tx_with_mailbox(box, msg);
}



/**
 * Finds the next available transmit mailbox.
 *
 * Currently, we only use mailbox 0.
 */
int can_find_free_tx_mailbox(void) {
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
	return kErrCanNoFreeMailbox;
}

/**
 * Transmits the given message on the given mailbox.
 */
int can_tx_with_mailbox(int mailbox, can_message_t *msg) {
	uint32_t reqAckFlag = 0, txOkFlag = 0;

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
	uint32_t TIR = 0x04;
	TIR |= (msg->identifier & 0x1FFFFFFFUL) << 3;

	if(msg->rtr) {
		TIR |= 0x02;
	}

	CAN->sTxMailBox[mailbox].TIR = TIR;

	// set data length
	CAN->sTxMailBox[mailbox].TDTR = msg->length;

	// copy low byte of data
	uint32_t data;

	data = (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | (msg->data[0] << 0);
	CAN->sTxMailBox[mailbox].TDLR = data;

	data = (msg->data[7] << 24) | (msg->data[6] << 16) | (msg->data[5] << 8) | (msg->data[4] << 0);
	CAN->sTxMailBox[mailbox].TDHR = data;

	// request transmission of mailbox 0, and wait for request acknowledgement
	CAN->sTxMailBox[mailbox].TIR |= 0x00000001;
	while(!(CAN->TSR & reqAckFlag)) {}

	// wait for the message to have been transmitted successfully
	// TODO: timeout if CAN physical layer got fucked
	while(!(CAN->TSR & txOkFlag)) {}
	return kErrSuccess;
}



/**
 * CAN IRQ handler
 */
void CEC_CAN_IRQHandler(void) {
	// was this a CAN IRQ?
//	if(SYSCFG->IT_LINE_SR[30] & SYSCFG_ITLINE30_SR_CAN) {
	if(1) {
		uint32_t masterIrq = CAN->MSR;

		// is this an error interrupt?
		if(masterIrq & CAN_MSR_ERRI) {
			// TODO: handle error interrupts
			LOG("CAN error: %x\n", CAN->ESR);

			// acknowledge error interrupt
			CAN->MSR &= (uint32_t) ~CAN_MSR_ERRI;
		}

		// do we have any pending messages in the FIFOs? repeat while we do
		uint32_t fifo0_pending = (CAN->RF0R & CAN_RF0R_FMP0);
		uint32_t fifo1_pending = (CAN->RF1R & CAN_RF1R_FMP1);

		while(fifo0_pending || fifo1_pending) {
//		if(fifo0_pending || fifo1_pending) {
			// is there any pending messages in FIFO 0?
			if(fifo0_pending) {
				can_read_fifo(0);
			}
			// are there any pending messages in FIFO 1?
			if(fifo1_pending) {
				can_read_fifo(1);
			}

			// check for overruns
			can_check_fifo_overrun();

			// re-check fifo status
			fifo0_pending = (CAN->RF0R & CAN_RF0R_FMP0);
			fifo1_pending = (CAN->RF1R & CAN_RF1R_FMP1);
		}
	}
}

/**
 * Reads a message from the specified FIFO.
 */
void can_read_fifo(int fifo) {
	// find a free slot in the receive buffer
	int freeRxSlot = -1;

	for(int i = 0; i < kRxBufferSize; i++) {
		// is this slot free?
		if(!rxBuffer[i].valid) {
			// yup, copy index and leave
			freeRxSlot = i;
			break;
		}
	}

	// if no free slot was found, return. we will probably loose this message
	if(freeRxSlot == -1) {
		LOG_PUTS("discarded CAN message");
		return;
	}

	// get the identifier of the message
	can_message_t *msg = &rxBuffer[freeRxSlot];

	msg->valid = 1;
	msg->rtr = (CAN->sFIFOMailBox[fifo].RIR & 0x02) ? 1 : 0;
	msg->identifier = (CAN->sFIFOMailBox[fifo].RIR & 0xFFFFFFF8) >> 3;

	// copy the data out of the register
	msg->length = CAN->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_DLC;

	uint32_t data;

	// read low word of data
	data = CAN->sFIFOMailBox[fifo].RDLR;

	msg->data[0] = (uint8_t) ((data & 0x000000FF) >> 0);
	msg->data[1] = (uint8_t) ((data & 0x0000FF00) >> 8);
	msg->data[2] = (uint8_t) ((data & 0x00FF0000) >> 16);
	msg->data[3] = (uint8_t) ((data & 0xFF000000) >> 24);

	// read high word of data
	data = CAN->sFIFOMailBox[fifo].RDHR;

	msg->data[4] = (uint8_t) ((data & 0x000000FF) >> 0);
	msg->data[5] = (uint8_t) ((data & 0x0000FF00) >> 8);
	msg->data[6] = (uint8_t) ((data & 0x00FF0000) >> 16);
	msg->data[7] = (uint8_t) ((data & 0xFF000000) >> 24);

	// release message from the FIFO
	if(fifo == 0) {
		CAN->RF0R |= CAN_RF0R_RFOM0;

		// wait for the mailbox to be released
//		while(CAN->RF0R & CAN_RF0R_RFOM0) {}
	} else if(fifo == 1) {
		CAN->RF1R |= CAN_RF1R_RFOM1;

		// wait for the mailbox to be released
//		while(CAN->RF1R & CAN_RF1R_RFOM1) {}
	}

	// log
	/*LOG("received to slot %u: id %x, len %u, %x %x\n", freeRxSlot,
			msg->identifier, msg->length,
			CAN->sFIFOMailBox[fifo].RDLR,
			CAN->sFIFOMailBox[fifo].RDHR);*/
}

/**
 * Checks whether the receive FIFOs were overrun.
 */
void can_check_fifo_overrun(void) {
	// was FIFO0 overrun?
	if(CAN->RF0R & CAN_RF0R_FOVR0) {
		droppedMessages = true;

		CAN->RF0R &= (uint32_t) ~CAN_RF0R_FOVR0;
	}
	// Was FIFO1 overrun?
	if(CAN->RF1R & CAN_RF1R_FOVR1) {
		droppedMessages = true;

		CAN->RF1R &= (uint32_t) ~CAN_RF1R_FOVR1;
	}
}

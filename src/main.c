/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lichtenstein.h"

#include "hw/output_mux.h"
#include "hw/differential_rx.h"
#include "hw/ws2811_generator.h"
#include "periph/spi.h"
#include "periph/canbus.h"

#include "hw/spi_flash.h"
#include "hw/nvram.h"

#include "cannabus/cannabus.h"

#include "gitcommit.h"

/**
 * Performs initialization of all hardware.
 */
static void init_hardware(void) {
	// first, initialize the hardware
//	mux_init();
//	diffrx_init();
//	ws2811_init();
	spi_init();
	can_init();

	// then, init flash and read the config
//	spiflash_init();
//	nvram_init();
}



/**
 * CANnabus callback: initializes CAN bus.
 */
static int cannabus_cb_can_init(void) {
	can_start();
	return 0;
}
/**
 * CANnabus callback: configures a mask-based filter.
 */
static int cannabus_cb_can_config_filter(unsigned int filter, uint32_t mask, uint32_t identifier) {
	return can_filter_mask(filter, mask, identifier);
}
/**
 * CANnabus callback: are there any messages waiting?
 */
static bool cannabus_cb_can_rx_waiting(void) {
	return can_messages_available();
}
/**
 * CANnabus callback: receives a message from CAN peripheral.
 */
static int cannabus_cb_can_rx_message(cannabus_can_frame_t *frame) {
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

	LOG("CANnabus rx from %x, %u bytes\n", rawFrame.identifier, rawFrame.length);

	// success!
	return kErrSuccess;
}
/**
 * CANnabus callback: transmits a message.
 */
static int cannabus_cb_can_tx_message(cannabus_can_frame_t *frame) {
	int err;

	LOG("CANnabus tx to %x, %u bytes\n", frame->identifier, frame->data_len);

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
 * Handles a CANnabus operation that isn't handled internally by the CANnabus
 * stack.
 */
static int cannabus_cb_handle_operation(cannabus_operation_t *op) {
	LOG("cannabus op: reg %x, rtr %d\n", op->reg, op->rtr);

	// TODO: implement, lol
	return kErrCannabusUnimplemented;
}

/**
 * Initializes CANnabus communication.
 */
static void init_cannabus(void) {
	int err;

	// get the node id from nvram
	nvram_t *nvram = nvram_get();
	uint16_t busId = nvram->cannabusId;

	// get testing bus id for nucleo board
#ifdef STM32F072
	busId = 0xDEAD;
#endif

	// build the list of functions
	cannabus_callbacks_t cb = {
		.can_init = cannabus_cb_can_init,
		.can_config_filter = cannabus_cb_can_config_filter,
		.can_rx_waiting = cannabus_cb_can_rx_waiting,
		.can_rx_message = cannabus_cb_can_rx_message,
		.can_tx_message = cannabus_cb_can_tx_message,

		.handle_operation = cannabus_cb_handle_operation,
	};

	// initialize bus
	err = cannabus_init(busId, 0x0B, &cb);

	if(err < kErrSuccess) {
		LOG("cannabus init failed: %d\n", err);
	}
}



/**
 * Application entry point
 */
int main(int argc __attribute__((__unused__)), char* argv[]__attribute__((__unused__))) {
	int err;

	// blinky (PA5)
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER5_0;

	int led = 0;

	// initialize trace
	trace_initialize();
	LOG("lichtenstein-led-fw %s\n", GIT_INFO);

#ifdef STM32F042
	LOG_PUTS("hw: STM32F042");
#endif
#ifdef STM32F072
	LOG_PUTS("hw: STM32F072");
#endif

	/*while(1) {
		// set LED status
		led = !led;

		if(led) {
			GPIOA->ODR |= GPIO_ODR_5;
		} else {
			GPIOA->ODR &= ~GPIO_ODR_5;
		}

		// sleep
		for(volatile int i = 0; i < 300000; i++) {}
	}*/


	// initialize hardware/peripherals
	init_hardware();

	// initialize CANnabus
	init_cannabus();

	// reset the outputs, then enable differential driver
	ws2811_send_pixel(300, kWS2811PixelTypeRGBW, 0x00000000);

	diffrx_set_state(kDiffRxEnabled);

	mux_set_state(kMux0, kMuxStateDifferentialReceiver);
	mux_set_state(kMux1, kMuxStateDifferentialReceiver);

	// enter main loop
	while(1) {
/*		// transmit periodic test messages
		can_message_t msg;
		memset(&msg, 0, sizeof(msg));

		msg.identifier = 0x0420BEEF;
		msg.rtr = 0;
		msg.length = 8;

		msg.data[0] = 'I';
		msg.data[1] = 'm';
		msg.data[2] = '2';
		msg.data[3] = 'H';
		msg.data[4] = 'i';
		msg.data[5] = 'g';
		msg.data[6] = 'h';
		msg.data[7] = 'e';

		err = can_transmit_message(&msg);

		if(err < kErrSuccess) {
			LOG("can_transmit_message: %d", err);
		}*/

		// process waiting CANnabus messages
		err = cannabus_process();

		if(err < 0) {
			LOG("cannabus_process failed: %d", err);
		}

		// set LED status
		led = !led;

		if(led) {
			GPIOA->ODR |= GPIO_ODR_5;
		} else {
			GPIOA->ODR &= ~GPIO_ODR_5;
		}

		// wait
		for(volatile int i = 0; i < 800000; i++) {}

		// wait for an interrupt
//		__WFI();
	}

	// never should get here
	return 0;
}



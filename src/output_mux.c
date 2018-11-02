/*
 * output_mux.c
 *
 * OUT0_MUX is on PA3
 * OUT1_MUX is on PB8
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "output_mux.h"

#include "lichtenstein.h"

/**
 * Initializes the output mux interfaces.
 *
 * This will set up the GPIO pins, then drive them so that the output of the
 * test output.
 */
void mux_init(void) {
	// enable GPIO clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	// configure OUT0_MUX
	GPIOA->MODER |= GPIO_MODER_MODER3_0;

	mux_set_state(kMux0, kMuxStateTestGenerator);

	// configure OUT1_MUX
	GPIOB->MODER |= GPIO_MODER_MODER8_0;

	mux_set_state(kMux1, kMuxStateTestGenerator);
}

/**
 * Sets the state of the specified multiplexer.
 *
 * Low selects the differential driver, a high selects the test generator.
 */
void mux_set_state(mux_t mux, mux_state_t state) {
	switch(mux) {
	case kMux0:
		if(state == kMuxStateDifferentialReceiver) {
			GPIOA->ODR &= (uint16_t) ~GPIO_ODR_3;
		} else if(state == kMuxStateTestGenerator) {
			GPIOA->ODR |= GPIO_ODR_3;
		}
		break;

	case kMux1:
		if(state == kMuxStateDifferentialReceiver) {
			GPIOB->ODR &= (uint16_t) ~GPIO_ODR_8;
		} else if(state == kMuxStateTestGenerator) {
			GPIOB->ODR |= GPIO_ODR_8;
		}
		break;
	}
}

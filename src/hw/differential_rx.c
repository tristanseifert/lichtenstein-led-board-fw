/*
 * differential.c
 *
 * Controls the SN75175. Its enable line is on PA7, shared with SPI_MOSI;
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "differential_rx.h"

#include "lichtenstein.h"

/// last state of the receiver. on powerup, we want it disabled
static diffrx_state_t gLastState = kDiffRxDisabled;

/**
 * Initializes the differential receiver.
 *
 * This is also called by the SPI driver when the SPI bus is no longer needed,
 * since the pin is shared with SPI_MOSI.
 */
void diffrx_init(void) {
	// enable GPIO clock and configure as output
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER7_0;

	// set the last receiver state (or default if first reset)
	diffrx_set_state(gLastState);
}

/**
 * Sets the state of the differential receiver.
 */
void diffrx_set_state(diffrx_state_t state) {
	// set the last state
	gLastState = state;

	// update GPIO: the signal is active high
	switch(state) {
		case kDiffRxDisabled:
			GPIOA->ODR &= (uint16_t) ~GPIO_ODR_7;
			break;

		case kDiffRxEnabled:
			GPIOA->ODR |= GPIO_ODR_7;
			break;
	}
}

/**
 * Returns the state of the differential receiver.
 */
diffrx_state_t diffrx_get_state(void) {
	return gLastState;
}

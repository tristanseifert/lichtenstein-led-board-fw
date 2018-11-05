/*
 * output_mux.c
 *
 * On STM32F042:
 * OUT0_MUX is on PA3
 * OUT1_MUX is on PB8
 *
 * On STM32F072:
 * OUT0_MUX is on PB3
 * OUT1_MUX is on PA10
 *
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
	// enable GPIO clocks and configure GPIO
#ifdef STM32F042
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	GPIOA->MODER |= GPIO_MODER_MODER3_0;
	GPIOB->MODER |= GPIO_MODER_MODER8_0;
#endif
#ifdef STM32F072
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	GPIOA->MODER |= GPIO_MODER_MODER10_0;
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
#endif

	// set outputs to test generator
	mux_set_state(kMux0, kMuxStateTestGenerator);
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
#ifdef STM32F042
		if(state == kMuxStateDifferentialReceiver) {
			GPIOA->ODR &= (uint16_t) ~GPIO_ODR_3;
		} else if(state == kMuxStateTestGenerator) {
			GPIOA->ODR |= GPIO_ODR_3;
		}
#endif
#ifdef STM32F072
		if(state == kMuxStateDifferentialReceiver) {
			GPIOB->ODR &= (uint16_t) ~GPIO_ODR_3;
		} else if(state == kMuxStateTestGenerator) {
			GPIOB->ODR |= GPIO_ODR_3;
		}
#endif
		break;

	case kMux1:
#ifdef STM32F042
		if(state == kMuxStateDifferentialReceiver) {
			GPIOB->ODR &= (uint16_t) ~GPIO_ODR_8;
		} else if(state == kMuxStateTestGenerator) {
			GPIOB->ODR |= GPIO_ODR_8;
		}
#endif
#ifdef STM32F072
		if(state == kMuxStateDifferentialReceiver) {
			GPIOA->ODR &= (uint16_t) ~GPIO_ODR_10;
		} else if(state == kMuxStateTestGenerator) {
			GPIOA->ODR |= GPIO_ODR_10;
		}
#endif
		break;
	}
}

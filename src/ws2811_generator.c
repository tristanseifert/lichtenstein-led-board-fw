/*
 * ws2811_generator.c
 *
 * The LEDOUT_TEST pin is on PB1.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "ws2811_generator.h"

#include "lichtenstein.h"

/**
 * Initializes the test generator.
 */
void ws2811_init(void) {
	// enable GPIO clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// set PB1 as output
	GPIOA->MODER |= GPIO_MODER_MODER1_0;
}

/**
 * Sends n repetitions of the given pixel value.
 *
 * This works for both SK6812RGBW and WS2811-type pixels since they speak the
 * same protocol, so long as the correct type is specified.
 */
void ws2811_send_pixel(int count, ws2811_pixel_type type, uint32_t pixel) {
	// TODO: implement by bit-banging. probably need asm

	// send a reset pulse

	// done!
}

#include <stdint.h>

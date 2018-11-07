/*
 * ws2811_generator.c
 *
 * The LEDOUT_TEST pin is on PB1.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "light_ws2812_cortex.h"

#include "lichtenstein.h"

#include <stdint.h>
#include <string.h>
#include "ws2812_generator.h"

static void ws2812_do_send(int count, uint8_t *buffer, int bufferLen);

/**
 * Initializes the test generator.
 */
void ws2812_init(void) {
	// enable GPIO clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// set PB1 as output
	GPIOA->MODER |= GPIO_MODER_MODER1_0;
}

/**
 * Sends n repetitions of the given pixel value.
 *
 * This works for both SK6812RGBW and WS2812-type pixels since they speak the
 * same protocol, so long as the correct type is specified.
 *
 * Pixel data is in the format of 0xRRGGBBWW.
 */
void ws2812_send_pixel(int count, ws2812_pixel_type type, uint32_t pixel) {
	// build data array
	uint8_t data[4];
	int dataLen = 3;

	memset(&data, 0, sizeof(data));

	switch(type) {
	case kWS2812PixelTypeRGBW:
		data[3] = (uint8_t) ((pixel & 0x000000FF) >>  0);
		dataLen = 4;
		// no break

	case kWS2812PixelTypeRGB:
		data[0] = (uint8_t) ((pixel & 0xFF000000) >> 24);
		data[1] = (uint8_t) ((pixel & 0x00FF0000) >> 16);
		data[2] = (uint8_t) ((pixel & 0x0000FF00) >>  8);
		break;
	}

	// send the data
	ws2812_do_send(count, (uint8_t *) &data, dataLen);

	// send a reset pulse (make output low)
	GPIOB->BRR |= GPIO_BRR_BR_1;

	// done!
}

/**
 * Performs the send of the data. This function is in RAM so it can call into
 *  the WS2812 send function directly.
 */
__attribute__((__section__(".data"))) void ws2812_do_send(int count, uint8_t *buffer, int bufferLen) {
	for(int i = 0; i < count; i++) {
		ws2812_sendarray(buffer, bufferLen);
	}
}

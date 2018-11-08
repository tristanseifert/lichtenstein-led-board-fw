/*
 * ws2811_generator.c
 *
 * The LEDOUT_TEST pin is on PB1 on the STM32F042 hardware.
 * The LEDOUT_TEST pin is on PC8 on the STM32F072 hardware.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "ws2812_generator.h"

#include "lichtenstein.h"

#include "FreeRTOS.h"

#include <stdint.h>
#include <string.h>

extern void ws2812_out_generic(unsigned int numLeds, uint32_t data, const unsigned int numBits);

/**
 * Initializes the test generator.
 */
void ws2812_init(void) {
	// configure GPIO
#ifdef STM32F042
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOA->MODER |= GPIO_MODER_MODER1_0;
#endif
#ifdef STM32F072
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
#endif
}

/**
 * Sends n repetitions of the given pixel value.
 *
 * This works for both SK6812RGBW and WS2812-type pixels since they speak the
 * same protocol, so long as the correct type is specified.
 *
 * Pixel data is in the format of 0xRRGGBBWW.
 *
 * @note Interrupts are disabled while data is output. This can lead to pretty
 * big interrupt latencies (~20ms, worst case with 300 LEDs).
 */
void ws2812_send_pixel(unsigned int count, ws2812_pixel_type type, uint32_t pixel) {
	uint32_t temp;

	// do output pls
	switch(type) {
	case kWS2812PixelTypeRGBW:
		// LED format is GGRRBBWW
		temp = ((pixel & 0x00FF0000) << 8) |  ((pixel & 0xFF000000) >> 8) |
			   ((pixel & 0x0000FF00)) | ((pixel & 0x000000FF));

		vPortEnterCritical();
		ws2812_out_generic(count, temp, 32);
		vPortExitCritical();
		break;

	case kWS2812PixelTypeRGB:
		// LED format is GGRRBB
		temp = ((pixel & 0x00FF0000) << 8) |  ((pixel & 0xFF000000) >> 8) |
			   ((pixel & 0x0000FF00));

		vPortEnterCritical();
		ws2812_out_generic(count, temp, 24);
		vPortExitCritical();
		break;
	}

	// send a reset pulse (make output low)
#ifdef STM32F042
	GPIOB->BRR |= GPIO_BRR_BR_1;
#endif
#ifdef STM32F072
	GPIOC->BRR |= GPIO_BRR_BR_8;
#endif

	// done!
}

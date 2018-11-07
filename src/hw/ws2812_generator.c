/*
 * ws2811_generator.c
 *
 * The LEDOUT_TEST pin is on PB1 on the STM32F042 hardware.
 * The LEDOUT_TEST pin is on PC8 on the STM32F072 hardware.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "lichtenstein.h"

#include <stdint.h>
#include <string.h>
#include "ws2812_generator.h"

// TODO: implement 3 byte send
void ws2812_send_4bytes(int numLeds, uint32_t data);

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
 */
void ws2812_send_pixel(int count, ws2812_pixel_type type, uint32_t pixel) {
	// build data array
	uint8_t data[4];
	int dataLen = 3;

	memset(&data, 0, sizeof(data));

	switch(type) {
	case kWS2812PixelTypeRGBW:
		ws2812_send_4bytes(count, pixel);
		break;

	case kWS2812PixelTypeRGB:
		// TODO: implement
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

/**
 * Performs the send of the data. This function is in RAM so it executes with no
 * wait states.
 */
__attribute__((__section__(".data"))) void ws2812_send_4bytes(int numLeds, uint32_t _data) {
	// prepare registers and bit fields
	volatile uint32_t *gpioReg;
	uint32_t setBitmask, clearBitmask;
	uint32_t counter = 0;

#ifdef STM32F042
	gpioReg = ((uint32_t *) &GPIOB->BSRR);
	setBitmask = GPIO_BSRR_BS_1;
	clearBitmask = GPIO_BSRR_BR_1;
#endif
#ifdef STM32F072
	gpioReg = ((uint32_t *) &GPIOC->BSRR);
	setBitmask = GPIO_BSRR_BS_8;
	clearBitmask = GPIO_BSRR_BR_8;
#endif

	// repeat for the number of LEDs we have
	for(int i = 0; i < numLeds; i++) {
		// send 32 bits of data
		uint32_t bit = (uint32_t) (1 << 31);
		uint32_t data = _data;

		asm volatile(
			// load counter
			"movs	%[numBits], #31\n"

			// declare loop
			"loop%=:\n"

			// at the start, the output is high
			"str		%[set], [%[gpio]]\n"

			// is this a 1 or a 0 bit?
			"tst		%[bit], %[data]\n"
			"bne		one%=\n"

			// it's a 0 bit. high for 350ns (16), low for 900ns (43)
			// wait 14 (tst = 1, bne = 1, str = 1) cycles
			".rept	13\n"
			"nop\n"
			".endr\n"
			// set the output low
			"str		%[clear], [%[gpio]]\n"

			// wait 32 (42 - 3 for branch, -1 for sub, -1 for lsr, -4 for test at top) cycles
			".rept	23\n"
//			".rept	33\n"
			"nop\n"
			".endr\n"
//			"b		next%=\n"
			"b		next_wait%=\n"

			// it's a 1 bit. high for 900ns (43), low for 350ns (16)
			"one%=:\n"
			// wait 38 cycles (43 - 1 for str, -4 for timing)
			".rept	37\n"
			"nop\n"
			".endr\n"
			// set the output low
			"str		%[clear], [%[gpio]]\n"

			// the nops are jumped to by the 0 bits to save space
			"next_wait%=:\n"

			// wait 8 cycles (16 - 3 for bne, 1 for sub, 1 for lsr, -2 for test at top)
			".rept	10\n"
			"nop\n"
			".endr\n"

			// handle next bit
			"next%=:\n"

			// shift the bit test mask right one for the next bit to output
			"lsr		%[bit], #1\n"

			// are there any more bits to output?
			"sub		%[numBits], #1\n"
			"bne		loop%=\n"
				: [numBits] "+r" (counter),
				  [bit] "+r" (bit)
				: [gpio] "r" (gpioReg),
				  [set] "r" (setBitmask),
				  [clear] "r" (clearBitmask),
				  [data] "r" (data)
				: "cc"
		);
	}
}

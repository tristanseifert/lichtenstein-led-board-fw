/*
 * ws2812_generator.S
 *
 * Assembly language routines (placed in RAM) to output WS2812 waveforms.
 *
 *  Created on: Nov 7, 2018
 *      Author: tristan
 */

/**
 * ws2812_out_generic(unsigned int numLeds, uint32_t data, const unsigned int numBits)
 *
 * Outputs `numBits` of `data` (starting with the MSB) `numLeds` times, in the
 * WS2812 data format.
 *
 * Inputs:
 *	r0: numLeds
 *	r1: data
 *	r2: numBits
 *
 * Clobbers:
 *	r0: Outer loop counter
 *	r1: Data (not modified)
 *	r2: Number of bits (not modified)
 *	r3: Inner loop counter
 *	r4: Bit test mask
 *	r5: GPIO port
 *	r6: GPIO set mask
 *	r7: GPIO clear mask
 */
 //	.section .data
	.global	ws2812_out_generic
	.type	fn, %function
	.align	4


ws2812_out_generic:
	// back up registers
	push		{r4, r5, r6, r7, lr}

	// get the GPIO port address (BSRR, offset 0x18)
//	ldr		r3, gpioReg
//	ldr		r5, [r3]
	ldr		r5, gpioReg

	// get the GPIO masks
//	ldr		r3, gpioSet
//	ldr		r6, [r3]
	ldr		r6, gpioSet

//	ldr		r3, gpioClear
//	ldr		r7, [r3]
	ldr		r7, gpioClear

	// outer loop start
ws2812_out_generic_outer:
	// load the bitmask
//	ldr		r3, bitmask
//	ldr		r4, [r3]
	ldr		r4, bitmask

	// load inner loop counter (number of bits)
	mov		r3, r2
	mov		r3, #31

	// inner loop start
ws2812_out_generic_inner:
	// set output high
	str		r6, [r5]

	// is this a 1 bit or a 0 bit?
	tst		r4, r1
	bne		ws2812_out_generic_inner_one

	// it's a 0 bit. high for 350ns (16), low for 900ns (43)
	// wait 14 (tst = 1, bne = 1, str = 1) cycles
.rept	14
	nop
.endr

	// set output low
	str		r7, [r5]

	// wait 32 (42 - 3 for branch, -1 for sub, -1 for lsr, -4 for test at top) cycles
.rept	21
//.rept	33
	nop
.endr
	//b		next%=
	b		ws2812_out_generic_inner_next_wait

	// it's a 1 bit. high for 900ns (43), low for 350ns (16)
ws2812_out_generic_inner_one:
	// wait 38 cycles (43 - 1 for str, -4 for timing)
.rept	37
	nop
.endr
	// set the output low
	str		r7, [r5]

	// jumped by the 0 bit handler to save some space
ws2812_out_generic_inner_next_wait:
.rept	9
	nop
.endr

	// handle next bit
ws2812_out_generic_inner_next:
	// shift the bit test mask now
	lsr		r4, #1

	// decrement inner loop counter
	sub		r3, #1
	bne		ws2812_out_generic_inner

	// wait a little to fix timing for the first bit of the next byte
	dmb
	dmb

	// decrement outer loop counter
	sub		r0, #1
	bne		ws2812_out_generic_outer

	// restore registers and return
	pop		{r4, r5, r6, r7, pc}
	nop

	.align	4

// define GPIO reg and masks
#ifdef STM32F042
gpioReg: // GPIOB->BSRR (GPIOB base 0x48000400)
	.word	0x48000418
gpioSet: // GPIO_BSRR_BS_1
	.word	0x00000002
gpioClear: //GPIO_BSRR_BR_1
	.word	0x00020000
#endif
#ifdef STM32F072
gpioReg: // GPIOC->BSRR (GPIOC base 0x48000800)
	.word	0x48000818
gpioSet: // GPIO_BSRR_BS_8
	.word	0x00000100
gpioClear: //GPIO_BSRR_BR_8
	.word	0x01000000
#endif

// starting bitmask
bitmask:
	.word	0x80000000

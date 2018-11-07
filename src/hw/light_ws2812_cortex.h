/*
 * light weight WS2812 lib - ARM Cortex M0/M0+ version
 *
 * Created: 07.07.2013
 *  Author: Tim (cpldcpu@gmail.com)
 */

#ifndef LIGHT_WS2812_H_
#define LIGHT_WS2812_H_


#include "stm32f0xx.h"

// port to use to set bit
#define ws2812_port_set ((uint32_t*) &GPIOB->BSRR)
// port to use to clear bit
#define ws2812_port_clr ((uint32_t*) &GPIOB->BRR)

// bitmask to set data output pin
#define ws2812_mask_set  GPIO_BSRR_BR_1
// bitmask to clear data output pin
#define ws2812_mask_clr  GPIO_BRR_BR_1


///////////////////////////////////////////////////////////////////////
// User defined area: Define CPU clock speed
//
// The current implementation of the sendarray routine uses cycle accurate
// active waiting. The routine is automatically adjusted according to
// the clockspeed defined below. Only values between 8 Mhz and 60 Mhz
// are allowable.
//
// Important: The timing calculation assumes that there are no waitstates
// for code memory access. If there are waitstates you may have to reduce
// the value below until you get acceptable timing. It is highly recommended
// to use this library only on devices without flash waitstates and
// predictable code execution timing.
///////////////////////////////////////////////////////////////////////
#define ws2812_cpuclk 48000000


#if (ws2812_cpuclk<8000000)
	#error "Minimum clockspeed for ARM ws2812 library is 8 Mhz!"
#endif

#if (ws2812_cpuclk>60000000)
	#error "Maximum clockspeed for ARM ws2812 library is 60 Mhz!"
#endif


/**
 * Sends `length` bytes of data on the WS2812 bus.
 */
void ws2812_sendarray(uint8_t *ledarray, int length);

#endif /* LIGHT_WS2812_H_ */

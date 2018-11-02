#include "cmsis_device.h"
#include "diag/Trace.h"

#include "lichtenstein.h"

// system core clock
extern uint32_t SystemCoreClock;

/**
 * Initialize UART2 with TX on PA2. Output is 8N1 115200.
 */
void trace_initialize(void) {
	// enable GPIO clock for port A and SYSCFG clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	// configure PA2 as alternate function out, push/pull (default on reset)
	GPIOA->MODER |= GPIO_MODER_MODER2_1;

	// enable clock for UART2 and reset it
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

	// set baud rate (USART2 clock is SYSCLK)
	RCC->CFGR3 |= (RCC_CFGR3_USART2SW_0) & RCC_CFGR3_USART2SW;

	USART2->BRR = (uint16_t) (SystemCoreClock / 115200);

	// RX, TX enable
	USART2->CR1 |= USART_CR1_RE | USART_CR1_TE;

	// enable USART
	USART2->CR1 |= USART_CR1_UE;
}


/**
 * Writes the given string to UART2.
 */
ssize_t trace_write (const char* buf, size_t nbyte) {
	// write bytes to the UART until done
	for(size_t i = 0; i < nbyte; i++) {
		// is the UART ready to accept more data?
		while(USART2->ISR & USART_ISR_BUSY) {}

		// write the byte of data
		USART2->TDR = (uint16_t) *buf++;
	}

	// return number of bytes written
	return (ssize_t) nbyte;
}

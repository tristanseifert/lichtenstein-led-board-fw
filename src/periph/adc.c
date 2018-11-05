/*
 * adc.c
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */
#include "adc.h"
#include "adc_private.h"

#include "lichtenstein.h"

#include <string.h>

/// ADC state
static adc_state_t gState;



/**
 * Initializes the ADC.
 */
void adc_init(void) {
	// initialize state: 4 conversions
	memset(&gState, 0, sizeof(gState));

	gState.numConversions = 4;

	// enable GPIOs in analog mode
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0) |
			(GPIO_MODER_MODER1_1 | GPIO_MODER_MODER1_0);

	// enable ADC clock and reset it
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;

	// enable the ADC temeprature sensor (ch16) and VREFINT (ch17)
	ADC->CCR &= ~(ADC_CCR_VBATEN | ADC_CCR_TSEN | ADC_CCR_VREFEN);
	ADC->CCR |= (ADC_CCR_TSEN | ADC_CCR_VREFEN);

	// begin an ADC calibration
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL) {}

	// enable auto-off, right align, override DR
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF | ADC_CFGR1_OVRMOD;

	// use internal ADC clock (CKMODE = 0b00)
	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;

	// sampling time of 239.5 ADC clock cycles
	ADC1->SMPR &= ~ADC_SMPR_SMP;
	ADC1->SMPR |= (ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2);

	// enable "conversion done" interrupts
	ADC1->IER |= ADC_IER_EOCIE;

	NVIC_EnableIRQ(ADC1_IRQn);
	NVIC_SetPriority(ADC1_IRQn, 3);

	// power on the ADC
	ADC1->CR |= ADC_CR_ADEN;
}

/**
 * Kicks off a measurement in the background.
 */
void adc_measure_begin(void) {
	// sample channels 0, 1, 16, 17
//	ADC1->CHSELR &= (uint32_t) ~0x7FFFF;
	ADC1->CHSELR = 0x30003;

	// begin conversion
	ADC1->CR |= ADC_CR_ADSTART;

	// we could wait here but we don't
}


/**
 * Copies the measurement for the two channels.
 *
 * @param out An array of two integers.
 */
void adc_get_measure(int *out) {
	// just copy the last measurements out
	memcpy(out, &gState.lastMeasure, sizeof(gState.lastMeasure));
}



/**
 * ADC interrupt handler
 */
#ifdef STM32F042
	void ADC1_IRQHandler(void) {
#endif
#ifdef STM32F072
	void ADC1_COMP_IRQHandler(void) {
#endif
	uint32_t isr = ADC1->ISR;

	// acknowledge all interrupts
	ADC1->ISR |= (ADC_ISR_AWD | ADC_ISR_OVR | ADC_ISR_EOSEQ |
			ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_ADRDY);

	// ISR is sometimes zero for some reason
	if(isr == 0) {

	}
	// was this an end of transfer irq?
	else if(isr & ADC_ISR_EOC) {
		adc_read_data();
	}
	// unhandled interrupt :(
	else {
		LOG("unhandled adc irq %x", isr);
	}
}

/**
 * Reads a single conversion out of the ADC.
 */
void adc_read_data(void) {
	// read value out
	uint32_t val = ADC1->DR;
//	LOG("ADC: %u\n", val);

	gState.lastMeasure[gState.conversions] = val;

	// disable conversion if all channels read
	if(++gState.conversions == gState.numConversions) {
		ADC1->CR |= ADC_CR_ADDIS;

		// reset counter as well
		gState.conversions = 0;

//		LOG_PUTS("adc stopped");
	}
}

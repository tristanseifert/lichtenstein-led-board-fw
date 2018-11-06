/*
 * adc.c
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */
#include "adc.h"
#include "adc_private.h"

#include "lichtenstein.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/// how many channels to sample
static const size_t kNumChannels = 4;
/// the channels we want to sample
static const unsigned int kChannelsToSample[] = {
	0, 1, 16, 17
};

/// ADC state
static adc_state_t gState;



/**
 * Initializes the ADC.
 */
void adc_init(void) {
	// initialize state: 4 conversions
	memset(&gState, 0, sizeof(gState));

	// enable GPIOs in analog mode
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0) |
			(GPIO_MODER_MODER1_1 | GPIO_MODER_MODER1_0);

	// enable ADC clock and reset it
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;

	// enable the ADC temperature sensor (ch16) and VREFINT (ch17)
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

	// power on the ADC
	ADC1->CR |= ADC_CR_ADEN;
}

/**
 * Kicks off a measurement in the background.
 */
int adc_measure(void) {
	int err = kErrSuccess;

	// sample channels
	for(size_t i = 0; i < kNumChannels; i++) {
		// set the correct bit
		uint32_t bit = (uint32_t) (1 << kChannelsToSample[i]);

		// set the channel, then start conversion
		ADC1->CHSELR = bit;

		ADC1->CR |= ADC_CR_ADSTART;

		// wait for ADC reading to complete
		err = adc_wait_end_conversion();

		if(err < kErrSuccess) {
			return err;
		}

		// copy out the reading
		gState.lastMeasure[i] = ADC1->DR;
	}

	// success!
	return err;
}

/**
 * Waits for the current ADC conversion to complete.
 */
int adc_wait_end_conversion(void) {
	// count how long it takes (5000 is about enough here)
	// we could go lower (1 ADC clk ~= 4 sysclk, so 1500 should suffice)
	volatile unsigned int timeout = 5000;

	do {
		// is the ADC still busy?
		if(ADC1->ISR & ADC_ISR_EOC) {
			// acknowledge the flag
			ADC1->ISR |= ADC_ISR_EOC;
			return kErrSuccess;
		}
	} while(timeout-- != 0);

	// if we get here, the request timed out
	return kErrADCConversionTimedOut;
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
 * Normalizes an ADC measurement from a raw ADC value into a fixed-point voltage
 * value.
 *
 * The value is in the form of a 16.16 fixed point value.
 *
 * This assumes that VDDA is 3.3V.
 */
uint32_t adc_normalize_value(unsigned int adc) {
	return adc;
}

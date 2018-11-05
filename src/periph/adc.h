/*
 * adc.h
 *
 * Interface to the on-chip analog/digital converter.
 *
 * This driver only configures the first two channels on PA0 and PA1, plus
 * the temperature sensor and internal voltage reference.
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */

#ifndef PERIPH_ADC_H_
#define PERIPH_ADC_H_

/**
 * Initializes the ADC.
 */
void adc_init(void);

/**
 * Kicks off a measurement in the background.
 */
void adc_measure_begin(void);

/**
 * Copies the measurement for the four channels.
 *
 * @param out An array of four integers.
 */
void adc_get_measure(int *out);



#endif /* PERIPH_ADC_H_ */

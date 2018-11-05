/*
 * adc_private.h
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */

#ifndef PERIPH_ADC_PRIVATE_H_
#define PERIPH_ADC_PRIVATE_H_

/**
 * Internal ADC state
 */
typedef struct {
	/// conversion counter
	int conversions;
	/// total number of conversions
	int numConversions;

	/// last measurement value
	unsigned int lastMeasure[4];
} adc_state_t;



/**
 * Reads a single conversion out of the ADC.
 */
void adc_read_data(void);

#endif /* PERIPH_ADC_PRIVATE_H_ */

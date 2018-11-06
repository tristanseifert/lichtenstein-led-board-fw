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
	/// last measurement value
	unsigned int lastMeasure[4];
} adc_state_t;


/**
 * Waits for the current ADC conversion to complete.
 */
int adc_wait_end_conversion(void);

#endif /* PERIPH_ADC_PRIVATE_H_ */

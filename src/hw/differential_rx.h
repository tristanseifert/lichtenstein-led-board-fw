/*
 * differential.h
 *
 * Controls the differential receiver.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef DIFFERENTIAL_RX_H_
#define DIFFERENTIAL_RX_H_

/**
 * States for the differential receiver: this corresponds to the enable input
 * on the SN75175.
 */
typedef enum {
	kDiffRxDisabled,
	kDiffRxEnabled
} diffrx_state_t;



/**
 * Initializes the differential receiver.
 */
void diffrx_init(void);

/**
 * Sets the state of the differential receiver.
 */
void diffrx_set_state(diffrx_state_t state);

/**
 * Returns the state of the differential receiver.
 */
diffrx_state_t diffrx_get_state(void);

#endif /* DIFFERENTIAL_RX_H_ */

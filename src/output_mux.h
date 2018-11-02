/*
 * output_mux.h
 *
 * Functions to control the state of the output multiplexers.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef OUTPUT_MUX_H_
#define OUTPUT_MUX_H_

/**
 * Available multiplexers.
 */
typedef enum {
	kMux0,
	kMux1
} mux_t;

/**
 * Possible settings for an output mux.
 */
typedef enum {
	kMuxStateDifferentialReceiver,
	kMuxStateTestGenerator
} mux_state_t;



/**
 * Initializes the output mux interfaces.
 */
void mux_init(void);

/**
 * Sets the state of the specified multiplexer.
 */
void mux_set_state(mux_t mux, mux_state_t state);



#endif /* OUTPUT_MUX_H_ */

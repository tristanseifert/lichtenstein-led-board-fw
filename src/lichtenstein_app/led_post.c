/*
 * led_post.c
 *
 *  Created on: Nov 8, 2018
 *      Author: tristan
 */
#include "led_post.h"

#include "lichtenstein.h"

#include "../hw/ws2812_generator.h"
#include "../hw/output_mux.h"
#include "../hw/differential_rx.h"

/**
 * Performs the LED self-test for both channels.
 */
void lichtenstein_led_post(void) {
	// enable test generator outputs
	diffrx_set_state(kDiffRxEnabled);

	mux_set_state(kMux0, kMuxStateTestGenerator);
	mux_set_state(kMux1, kMuxStateTestGenerator);

	// TODO: get config from NVRAM

	// reset all LEDs
	ws2812_send_pixel(300, kWS2812PixelTypeRGBW, 0x00000000);

	// enable differential receiver and put that on output
	diffrx_set_state(kDiffRxEnabled);

	mux_set_state(kMux0, kMuxStateDifferentialReceiver);
	mux_set_state(kMux1, kMuxStateDifferentialReceiver);

}

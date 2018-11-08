/*
 * led_post.h
 *
 * Power on self-test for LEDs. This will set them all to R/G/B/W, if enabled in
 * the NVRAM config, then clear them.
 *
 *  Created on: Nov 8, 2018
 *      Author: tristan
 */

#ifndef LICHTENSTEIN_APP_LED_POST_H_
#define LICHTENSTEIN_APP_LED_POST_H_

/**
 * Performs the LED self-test for both channels.
 */
void lichtenstein_led_post(void);

#endif /* LICHTENSTEIN_APP_LED_POST_H_ */

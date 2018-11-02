/*
 * ws2811_generator.h
 *
 * Provides an interface to generate some very simple test patterns on the
 * LEDOUT_TEST pin.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include <stdint.h>

#ifndef WS2811_GENERATOR_H_
#define WS2811_GENERATOR_H_

/**
 * Pixel types for output data format.
 *
 * There are really just two formats: WS2811 (3 bytes shifted out per pixel) and
 * SK6812 (4 bytes shifted out per pixel)
 */
typedef enum {
	kWS2811PixelTypeRGB,
	kWS2811PixelTypeRGBW,
} ws2811_pixel_type;



/**
 * Initializes the test generator.
 */
void ws2811_init(void);

/**
 * Sends n repetitions of the given pixel value.
 *
 * This works for both SK6812RGBW and WS2811-type pixels since they speak the
 * same protocol, so long as the correct type is specified.
 */
void ws2811_send_pixel(int count, ws2811_pixel_type type, uint32_t pixel);

#endif /* WS2811_GENERATOR_H_ */

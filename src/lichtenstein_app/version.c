/*
 * version.c
 *
 * Defines the version of the firmware.
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */
#include "version.h"

#include <stdint.h>

// for broken compilers
#ifndef COMPILE_TIME
#define COMPILE_TIME				-1
#endif



/**
 * This is the firmware version: it's placed in the version field at the end of
 * the flash.
 *
 * The high byte is the major version, low byte is minor version.
 */
const version_t kLichtensteinVersion __attribute__((__section__(".version"))) = {
	.version = 0x0010,
	.buildTime = COMPILE_TIME
};

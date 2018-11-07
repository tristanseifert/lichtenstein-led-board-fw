/*
 * version.h
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */

#ifndef LICHTENSTEIN_APP_VERSION_H_
#define LICHTENSTEIN_APP_VERSION_H_

#include <stdint.h>

/**
 * Structure placed at the end of flash to identify this firmware.
 */
typedef struct {
	/// Lichtenstein version
	uint16_t version;

	/// build timestamp
	uint32_t buildTime;
} __attribute__((__packed__)) version_t;

extern const version_t kLichtensteinVersion;

#endif /* LICHTENSTEIN_APP_VERSION_H_ */

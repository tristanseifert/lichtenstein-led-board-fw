/*
 * nvram.h
 *
 * Provides an NVRAM-like interface to the flash. A single page is set aside as
 * a variable storage area (see nvram_t below).
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */

#ifndef NVRAM_H_
#define NVRAM_H_

#include <stdint.h>

/**
 * NVRAM structure
 */
typedef struct {
	// hardware revision
	uint16_t hwVersion;
	// software version
	uint16_t swVersion;

	// CANnabus device id
	uint16_t cannabusId;

	// CRC-16 checksum. this must be the last value
	uint16_t checksum;
} nvram_t __attribute__((__packed__));



/**
 * Initializes NVRAM. This will read the values back from flash, and initialize
 * them if the checksum of the read back values is invalid or flash hasn't been
 * initialized.
 */
void nvram_init(void);

/**
 * Reads NVRAM data from flash.
 */
int nvram_read(void);

/**
 * Writes NVRAM data back to flash after it's been updated.
 */
int nvram_write(void);

/**
 * Gets the shadow copy of the NVRAM.
 */
nvram_t *nvram_get(void);


#endif /* NVRAM_H_ */

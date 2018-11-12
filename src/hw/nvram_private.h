/*
 * nvram_private.h
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */

#ifndef NVRAM_PRIVATE_H_
#define NVRAM_PRIVATE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**
 * Validates whether the contents of the shadow NVRAM are valid.
 */
bool nvram_is_valid(void);

/**
 * Initializes the shadow copy with sane defaults, then writes it to flash.
 */
int nvram_reset_data(void);

/**
 * Returns the CRC16 of the shadow copy.
 */
uint16_t nvram_shadow_crc16(void);

#endif /* NVRAM_PRIVATE_H_ */

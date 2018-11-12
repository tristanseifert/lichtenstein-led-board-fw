/*
 * crc.h
 *
 * Provides basic CRC routines.
 *
 *  Created on: Nov 11, 2018
 *      Author: tristan
 */

#ifndef UTIL_CRC_H_
#define UTIL_CRC_H_

#include <stdint.h>
#include <stddef.h>

/**
 * Calculates the CRC over a block of data. If the entire block is available,
 * set `currentCRC` to 0xFFFF.
 */
uint16_t crc16(void *data, size_t length, uint16_t currentCRC);


#endif /* UTIL_CRC_H_ */

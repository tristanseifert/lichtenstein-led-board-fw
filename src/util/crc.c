/*
 * crc.c
 *
 *  Created on: Nov 11, 2018
 *      Author: tristan
 */
#include "crc.h"

#include "lichtenstein.h"

#include <stdint.h>
#include <stddef.h>

/**
 * Calculates the CRC over a block of data. If the entire block is available,
 * set `currentCRC` to 0xFFFF.
 *
 * from http://people.cs.umu.se/isak/snippets/crc-16.c
 */
uint16_t crc16(void *_data, size_t len, uint16_t currentCRC) {
	// CCITT CRC16 polynomial (0x1021, but we use the reverse)
	const uint16_t poly = 0x8408;

    unsigned int data, i;
    unsigned int crc = currentCRC;

    // handle zero length data
    if (len == 0) {
    	return (uint16_t) ~crc;
    }

    // get data and start the loop
    uint8_t *data_p = (uint8_t *) _data;

	do {
		for (i = 0, data = (unsigned int) 0xff & *data_p++; i < 8; i++, data >>= 1) {
			if ((crc & 0x0001) ^ (data & 0x0001)) {
				crc = (crc >> 1) ^ poly;
			} else  {
				crc >>= 1;
			}
		}
	} while(--len);

    crc = ~crc;
    data = crc;
    crc = (crc << 8) | (data >> 8 & 0xff);

    return (uint16_t) crc;
}

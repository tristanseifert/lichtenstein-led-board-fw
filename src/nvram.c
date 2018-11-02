/*
 * nvram.c
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */
#include "nvram.h"
#include "nvram_private.h"

#include "spi_flash.h"

#include "lichtenstein.h"

#include <string.h>

// location of NVRAM in the flash's security pages
const uint32_t kNVRAMSecurityPageAddress = 0x00000000;

// shadow copy of NVRAM
static nvram_t gNVRAM;



/**
 * Initializes NVRAM. This will read the values back from flash, and initialize
 * them if the checksum of the read back values is invalid or flash hasn't been
 * initialized.
 */
void nvram_init(void) {
	int err;

	// clear local NVRAM copy
	memset(&gNVRAM, 0, sizeof(gNVRAM));

	// attempt to read NVRAM
	err = nvram_read();

	if(err < 0) {
		LOG("error reading nvram %d", err);
		return;
	}

	// are the NVRAM contents valid?
	if(!nvram_is_valid()) {
		// if not, initialize it
		err = nvram_reset_data();

		if(err < 0) {
			LOG("error initializing nvram contents %d", err);
			return;
		}
	}

	// print data
	LOG("HW vers %0x, SW vers %0x", gNVRAM.hwVersion, gNVRAM.swVersion);
	LOG("CANnabus ID %0x", gNVRAM.cannabusId);
}

/**
 * Reads NVRAM data from flash.
 */
int nvram_read(void) {
	return spiflash_read_security(sizeof(nvram_t), &gNVRAM, kNVRAMSecurityPageAddress);
}

/**
 * Writes NVRAM data back to flash after it's been updated.
 */
int nvram_write(void) {
	int err;

	// first, erase the page
	err = spiflash_erase_security(kNVRAMSecurityPageAddress);
	if(err < 0) {
		return err;
	}

	// now, program it
	err = spiflash_write_security(sizeof(gNVRAM), &gNVRAM, kNVRAMSecurityPageAddress);
	return err;
}

/**
 * Gets the shadow copy of the NVRAM.
 */
nvram_t *nvram_get(void) {
	return &gNVRAM;
}


/**
 * Validates whether the contents of the shadow NVRAM are valid.
 */
bool nvram_is_valid(void) {
	return (gNVRAM.checksum == nvram_shadow_crc16());
}

/**
 * Initializes the shadow copy with sane defaults, then writes it to flash.
 */
int nvram_reset_data(void) {
	int err;

	// clear it
	memset(&gNVRAM, 0, sizeof(gNVRAM));

	// set hwversion and swversion
	gNVRAM.hwVersion = 0x001F;
	gNVRAM.swVersion = 0x0001;

	// generate cannabus address from device unique id (starts at 0x1FFFF7AC)
	uint16_t cannabusId = nvram_crc16((void *) 0x1FFFF7AC, 12);
	gNVRAM.cannabusId = cannabusId;

	// assign checksum
	gNVRAM.checksum = nvram_shadow_crc16();

	// write the nvram back to the flash
	err = nvram_write();
	return err;
}



/**
 * Calculates the CRC16 over the given block of data.
 *
 * from http://people.cs.umu.se/isak/snippets/crc-16.c
 */
uint16_t nvram_crc16(void *_data, size_t len) {
	// CCITT CRC16 polynomial (0x1021, but we use the reverse)
	const uint16_t poly = 0x8408;

    unsigned int data, i;
    unsigned int crc = 0xffff;

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

/**
 * Returns the CRC16 of the shadow copy.
 */
uint16_t nvram_shadow_crc16(void) {
	return nvram_crc16(&gNVRAM, sizeof(nvram_t) - sizeof(uint16_t));
}

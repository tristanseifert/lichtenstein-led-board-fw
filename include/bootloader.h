/*
 * bootloader.h
 *
 * Provides an interface to the basic bootloader on the Lichtenstein devices.
 *
 * The loader occupies the first 0x1000 bytes of flash. A structure describing
 * the loader's API is located in the last 0x40 bytes.
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */

#ifndef LOADER_H_
#define LOADER_H_

#include <stdint.h>
#include <stddef.h>

/**
 * Callbacks used by the bootloader to access the flash
 */
typedef struct {
	/// Opens the flash for reading/writing.
	int (*flash_open)(void);
	/// Closes the flash.
	int (*flash_close)(void);

	/// Reads from the flash.
	int (*flash_read)(uint32_t, size_t, void *);
	/// Erases an area of flash.
	int (*flash_erase)(uint32_t, size_t);
	/// Writes to the flash. Requires the page is erased first.
	int (*flash_write)(uint32_t, size_t, void *);
} bootloader_flash_callbacks_t;



/**
 * A bootloader info struct, containing information about each of the firmwares
 * stored in the flash.
 */
typedef struct {
	/// Total number of firmware images in flash.
	uint8_t totalFirmwares;

	/// Which firmware is currently running?
	uint8_t currentFirmware;
	/// Which firmware is the failsafe firmware?
	uint8_t failsafeFirmware;

	/// Info about one of 8 firmware images
	struct {
		/// Firmware version (0xFFFF == unused)
		uint16_t version;
		/// How many startup failures were there?
		uint8_t startFails;
		/// How many startup successes were there?
		uint8_t startSuccesses;
	} fwInfo[8];
} __attribute__((__packed__)) bootloader_info_t;

/**
 * Functions and information provided by the bootloader in ROM.
 */
typedef struct {
	/// Version of the loader
	uint32_t version;

	/// Marks this firmware as "good."
	int (*mark_fw_good)(bootloader_flash_callbacks_t *);
	/// Reads out the loader information block from flash.
	int (*read_loader_info)(bootloader_flash_callbacks_t *, bootloader_info_t *);
} __attribute__((__packed__)) bootloader_interface_t;

#endif /* LOADER_H_ */

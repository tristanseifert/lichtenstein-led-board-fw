/*
 * spi_flash.c
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "spi_flash.h"
#include "spi_flash_private.h"

#include "../periph/spi.h"
#include "differential_rx.h"

#include "lichtenstein.h"

/**
 * Initializes the SPI flash: reads vendor info.
 */
void spiflash_init(void) {
	int err;

	// wait for flash to be idle
	spiflash_wait_for_idle();

	// start an SPI transaction to read manufacturer/chip id
	uint8_t idBuffer[4];
	uint8_t idCommand[1] = {0x9F};

	spi_begin();
	err = spiflash_command(&idCommand, sizeof(idCommand), &idBuffer, sizeof(idBuffer));
	spi_end();

	// handle errors, then log id info
	if(err < 0) {
		LOG("error reading flash id: %d", err);
		return;
	}

	LOG("manufacturer %x, id %x %x", idBuffer[0], idBuffer[1], idBuffer[2]);
}

/**
 * Reads n bytes from the flash, starting at the specified address.
 */
int spiflash_read(size_t nBytes, void *buf, uint32_t address) {
	return spiflash_read_internal(0x0B, nBytes, buf, address);
}
/**
 * Reads n bytes from the flash's security register. The register is specified
 * by bits 9-8 of the address.
 */
int spiflash_read_security(size_t nBytes, void *buf, uint32_t address) {
	return spiflash_read_internal(0x48, nBytes, buf, address);
}

/**
 * Reads n bytes from the flash, starting at the specified address. The specific
 * read command byte to use is specified.
 */
int spiflash_read_internal(uint8_t command, size_t nBytes, void *buf, uint32_t address) {
	int err;

	// validate parameters
	if(buf == NULL) {
		return -1;
	}

	// wait for the flash to be idle
	spiflash_wait_for_idle();

	// set up command (with the dummy byte)
	uint8_t readCommand[5] = {command, 0, 0, 0, 0};

	readCommand[2] = (address & 0x00FF0000) >> 16;
	readCommand[3] = (address & 0x0000FF00) >> 8;
	readCommand[4] = (address & 0x000000FF) >> 0;

	// execute the command
	spi_begin();
	err = spiflash_command(&readCommand, sizeof(readCommand), buf, nBytes);
	spi_end();

	return err;
}



/**
 * Writes n bytes to the flash, starting at the specified address. The pages
 * attempted to be programmed must have been erased previously.
 *
 * @note Only a maximum of 256 bytes can be written at once.
 *
 * @note Data is written within a single 256 byte page only: if the address is
 * not at a page boundary, and more than the number of bytes remaining in the
 * page are written, writing wraps to the start of the page.
 */
int spiflash_write(size_t nBytes, void *buf, uint32_t address) {
	return spiflash_write_page_internal(0x02, nBytes, buf, address);
}
/**
 * Writes n (at most 256) bytes to the address in the security register space of
 * the flash.
 */
int spiflash_write_security(size_t nBytes, void *buf, uint32_t address) {
	return spiflash_write_page_internal(0x42, nBytes, buf, address);
}

/**
 * Writes n bytes to the flash, starting at the specified address. The pages
 * attempted to be programmed must have been erased previously. This will write
 * up to one page (256 bytes) of data.
 *
 * The specific write command to use can be specified.
 */
int spiflash_write_page_internal(uint8_t command, size_t nBytes, void *buf, uint32_t address) {
	int err;

	// validate inputs
	if(nBytes > 256) {
		return -1;
	} else if(buf == NULL) {
		return -1;
	}

	// wait for the flash to be idle
	spiflash_wait_for_idle();

	// enable writing
	err = spiflash_write_enable();

	if(err < 0) {
		return err;
	}

	// send the write command
	uint8_t writeCmd[4] = {command, 0, 0, 0};
	writeCmd[2] = (address & 0x00FF0000) >> 16;
	writeCmd[3] = (address & 0x0000FF00) >> 8;
	writeCmd[4] = (address & 0x000000FF) >> 0;

	spi_begin();
	err = spiflash_command(&writeCmd, sizeof(writeCmd), NULL, 0);

	if(err < 0) {
		spi_end();
		return err;
	}

	// now, send the data
	err = spiflash_command(buf, nBytes, NULL, 0);
	spi_end();

	if(err < 0) {
		return err;
	}

	// disable writing
	err = spiflash_write_disable();

	return err;
}



/**
 * Erases n bytes starting at the specified address, in the most efficient way
 * possible.
 *
 * TODO: make this smart lol
 */
int spiflash_erase(size_t nBytes, uint32_t address) {
	int err = 0;

	// align address to a 4K block
	address &= 0x00FFF000;

	// erase however many 4K blocks we need to
	int numBlocks = ((int) nBytes) / 0x1000;

	if(numBlocks == 0) {
		return 0;
	}

	// begin erasing
	for(int i = 0; i < numBlocks; i++) {
		// call through to the erase block function
		err = spiflash_erase_block_internal(0x20, address);

		// handle errors by leaving the loop
		if(err < 0) {
			break;
		}

		// increment address
		address += 0x1000;
	}

	// return error code
	return err;
}
/**
 * Erases an entire security register.
 */
int spiflash_erase_security(uint32_t address) {
	address &= 0x3FFF;
	return spiflash_erase_block_internal(0x44, address);
}

/**
 * Erases a single page at the given address, using the specified command.
 *
 * @note Unlike the other internal functions, this function DOES NOT disable
 * write protection by itself. You need to do that outside of this function.
 */
int spiflash_erase_block_internal(uint8_t command, uint32_t address) {
	int err;

	// wait for flash to be idle
	spiflash_wait_for_idle();

	// enable writing
	err = spiflash_write_enable();

	if(err < 0) {
		return err;
	}

	// set up the block erase command
	uint8_t eraseCmd[4] = {command, 0, 0, 0};
	eraseCmd[2] = (address & 0x00FF0000) >> 16;
	eraseCmd[3] = (address & 0x0000FF00) >> 8;
	eraseCmd[4] = (address & 0x000000FF) >> 0;

	// execute command
	spi_begin();
	err = spiflash_command(&eraseCmd, sizeof(eraseCmd), NULL, 0);
	spi_end();

	if(err < 0) {
		return err;
	}

	// disable writing again
	err = spiflash_write_disable();
	return err;
}



/**
 * Writes a command to the chip, then reads zero or more bytes of response.
 */
int spiflash_command(void *_command, size_t commandLen, void *_response, size_t responseLen) {
	int bytesRead = 0;

	// write the command
	uint8_t *command = (uint8_t *) _command;

	for(size_t i = 0; i < commandLen; i++) {
		spi_io(*command++);
	}

	// should we read a response?
	if(responseLen != 0) {
		uint8_t *response = (uint8_t *) _response;

		// figure out if the diff receiver is enabled (ff) or not (00)
		uint8_t writeByte = 0x00;

		if(diffrx_get_state() == kDiffRxEnabled) {
			writeByte = 0xFF;
		}

		for(size_t i = 0; i < responseLen; i++) {
			// write either 00 or FF depending on differential driver state
			*response++ = spi_io(writeByte);
		}
	}

	// return the number of bytes we read
	return bytesRead;
}



/**
 * Reads out the status register.
 */
int spiflash_get_status(uint16_t *status) {
	int err;

	uint8_t temp;
	uint8_t command[1];

	// clear the status register
	if(status == NULL) {
		return -1;
	}

	*status = 0;

	// read the first byte of the register
	command[0] = 0x05;

	spi_begin();
	err = spiflash_command(&command, sizeof(command), &temp, sizeof(temp));
	spi_end();

	if(err < 0) {
		return err;
	}

	*status |= (uint16_t) (temp << 8);

	// read the second byte of the register
	command[0] = 0x35;

	spi_begin();
	err = spiflash_command(&command, sizeof(command), &temp, sizeof(temp));
	spi_end();

	if(err < 0) {
		return err;
	}

	*status |= (uint16_t) temp;

	// if we get here, it was successful
	return 0;
}

/**
 * Checks if the flash is busy with an internal operation.
 *
 * We have no real good way to report error from this function, so if an error
 * occurs, we break into the debugger and return false.
 */
bool spiflash_is_busy(void) {
	int err;
	uint8_t temp;

	// read first byte of the status register
	uint8_t command[1] = {0x05};

	spi_begin();
	err = spiflash_command(&command, sizeof(command), &temp, sizeof(temp));
	spi_end();

	if(err < 0) {
#ifdef DEBUG
		asm volatile ("bkpt 0");
#endif
		return false;
	}

	// return busy flag
	return (temp & 0x01);
}



/**
 * Waits until the flash is no longer busy.
 */
void spiflash_wait_for_idle(void) {
	bool busy;

	// check the status register, waiting if it's still busy
	do {
		// read the status register
		busy = spiflash_is_busy();

		// waste a little time if flash is still busy
		if(busy) {
			for(volatile int x = 0; x < 100000; x++) {}
		}
	} while(busy);
}



/**
 * Disables software write protection in the flash.
 */
int spiflash_write_enable(void) {
	int err;

	// write the single byte command
	uint8_t command[1] = {0x06};

	spi_begin();
	err = spiflash_command(&command, sizeof(command), NULL, 0);
	spi_end();

	// return the error code of the command routine
	return err;
}

/**
 * Enables software write protection in the flash.
 */
int spiflash_write_disable(void) {
	int err;

	// write the single byte command
	uint8_t command[1] = {0x04};

	spi_begin();
	err = spiflash_command(&command, sizeof(command), NULL, 0);
	spi_end();

	// return the error code of the command routine
	return err;
}

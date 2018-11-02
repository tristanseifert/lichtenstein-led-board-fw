/*
 * spi_flash_private.h
 *
 * Private helper methods for accessing the SPI flash. This mostly relates to
 * making sending of commands easier.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef SPI_FLASH_PRIVATE_H_
#define SPI_FLASH_PRIVATE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * Reads n bytes from the flash, starting at the specified address. The specific
 * read command byte to use is specified.
 */
int spiflash_read_internal(uint8_t command, size_t nBytes, void *buf, uint32_t address);

/**
 * Writes n bytes to the flash, starting at the specified address. The pages
 * attempted to be programmed must have been erased previously. This will write
 * up to one page (256 bytes) of data.
 *
 * The specific write command to use can be specified.
 */
int spiflash_write_page_internal(uint8_t command, size_t nBytes, void *buf, uint32_t address);

/**
 * Erases a single page at the given address, using the specified command.
 */
int spiflash_erase_block_internal(uint8_t command, uint32_t address);



/**
 * Writes a command to the chip, then reads zero or more bytes of response.
 */
int spiflash_command(void *command, size_t commandLen, void *response, size_t responseLen);



/**
 * Checks if the flash is busy with an internal operation.
 */
bool spiflash_is_busy(void);

/**
 * Waits until the flash is no longer busy.
 */
void spiflash_wait_for_idle(void);

/**
 * Reads out the status register.
 */
int spiflash_get_status(uint16_t *status);



/**
 * Disables software write protection in the flash.
 */
int spiflash_write_enable(void);

/**
 * Enables software write protection in the flash.
 */
int spiflash_write_disable(void);



#endif /* SPI_FLASH_PRIVATE_H_ */

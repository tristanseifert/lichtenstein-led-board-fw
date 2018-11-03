/*
 * spi_flash.h
 *
 * Provides a basic interface to the SPI flash: in this case, that would be a
 * AT25SF041 4Mbit flash.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

#include <stddef.h>
#include <stdint.h>

/**
 * Initializes the SPI flash: reads vendor info.
 */
void spiflash_init(void);

/**
 * Reads n bytes from the flash, starting at the specified address.
 */
int spiflash_read(size_t nBytes, void *buf, uint32_t address);
/**
 * Reads n bytes from the flash's security register. The register is specified
 * by bits 9-8 of the address.
 */
int spiflash_read_security(size_t nBytes, void *buf, uint32_t address);

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
int spiflash_write(size_t nBytes, void *buf, uint32_t address);
/**
 * Writes n (at most 256) bytes to the address in the security register space of
 * the flash.
 */
int spiflash_write_security(size_t nBytes, void *buf, uint32_t address);

/**
 * Erases n bytes starting at the specified address, in the most efficient way
 * possible.
 */
int spiflash_erase(size_t nBytes, uint32_t address);
/**
 * Erases an entire security register.
 */
int spiflash_erase_security(uint32_t address);

#endif /* SPI_FLASH_H_ */

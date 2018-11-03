/*
 * spi.h
 *
 * Provides routines for accessing the connected SPI peripheral.
 *
 * All pins except for SPI_MOSI are configured: this pin is configured only
 * when the driver is used because it is used for other purposes normally.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

/**
 * Initializes the SPI peripheral.
 */
void spi_init(void);



/**
 * Begins an SPI transaction.
 */
void spi_begin(void);

/**
 * Ends an SPI transaction.
 */
void spi_end(void);



/**
 * Writes a single byte to the SPI, then returns the byte read.
 */
uint8_t spi_io(uint8_t out);


#endif /* SPI_H_ */

/*
 * spi.c
 *
 * SPI_CS:		PA4
 * SPI_SCK:		PA5
 * SPI_MISO:	PA6
 * SPI_MOSI:	PA7
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "spi.h"
#include "../hw/differential_rx.h"

#include "lichtenstein.h"

#include <stdint.h>

/**
 * Initializes the SPI peripheral.
 */
void spi_init(void) {
	// enable GPIO clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// configure CS, SCK, MISO as AF in high speed mode
	GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6);

	// enable SPI clock and reset it
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

	// configure SPI with fPCLK / 8 clock, master, mode 0, software /CS management, CS high
	SPI1->CR1 |= SPI_CR1_BR_1 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

	// 8 bit data size, generate slave select
	SPI1->CR2 |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2) | SPI_CR2_SSOE;
}



/**
 * Begins an SPI transaction.
 *
 * This configures SPI_MOSI as an alternate function output.
 */
void spi_begin(void) {
	// configure MOSI (PA7)
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;

	// assert /CS and enable SPI
	SPI1->CR1 &= (uint32_t) ~SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_SPE;
}

/**
 * Ends an SPI transaction.
 *
 * This returns SPI_MOSI to the normal function of differential receiver select
 * and is done by just re-initializing that peripheral.
 */
void spi_end(void) {
	// de-assert CS and disable SPI
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 &= (uint32_t) ~SPI_CR1_SPE;

	// re-initialize differential rx
	diffrx_init();
}



/**
 * Writes a single byte to the SPI, then returns the byte read.
 */
uint8_t spi_io(uint8_t out) {
	// TX fifo empty?
	while(!(SPI1->SR & SPI_SR_TXE)) {}

	// write a byte
	SPI1->DR = out;

	// wait for RX fifo to not be empty
	while(!(SPI1->SR & SPI_SR_RXNE)) {}

	return (uint8_t) SPI1->DR;
}

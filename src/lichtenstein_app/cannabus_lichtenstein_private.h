/*
 * cannabus_lichtenstein_private.h
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */

#ifndef LICHTENSTEIN_APP_CANNABUS_LICHTENSTEIN_PRIVATE_H_
#define LICHTENSTEIN_APP_CANNABUS_LICHTENSTEIN_PRIVATE_H_

#include "../cannabus/cannabus.h"

/**
 * Gets ADC data, converts it, and sends it. (Reg 0x010)
 *
 * Allowed accesses: RTR
 *
 * The register is divided as follows:
 * - Bytes 1-2:	Voltage
 * - Bytes 2-3: Current
 * - Bytes 4-5: Temperature (Celsius)
 * - Bytes 6-7: Raw ADC reference voltage
 *
 * All converted values are presented in 8.8 fixed point form.
 */
int lichtenstein_cannabus_adc(cannabus_operation_t *op);

/**
 * Handles a write to the MUX control register. (Reg 0x011)
 *
 * Allowed accesses: Write
 *
 * The register is divided as follows:
 * - Byte 1:		Output 0 mux state
 * - Byte 2:		Output 1 mux state
 *
 * Mux state is 0x00 selects the received differential signal, and 0x01 selects
 * the test generator as output.
 */
int lichtenstein_cannabus_muxctrl(cannabus_operation_t *op);

/**
 * Handles a write to the differential receiver control register. (Reg 0x012)
 *
 * Allowed accesses: Write
 *
 * The register is divided as follows:
 * - Byte 1:		Differential driver state (0 = disabled, 1 = enabled)
 */
int lichtenstein_cannabus_diffrxctrl(cannabus_operation_t *op);

/**
 * Handles a write to the test generator control register. (Reg 0x013)
 *
 * Allowed accesses: Write
 *
 * Writing to this register immediately starts the test generator.
 *
 * The register is divided as follows:
 * - Bytes 1-2:	Number of LEDs for which to output data
 * - Bytes 3-6:	RGB(W) data to output
 * - Byte 7:		LED type (0 = WS2812b, 1 = SK6812)
 * - Byte 8:		Reserved (should be 0)
 */
int lichtenstein_cannabus_testgen(cannabus_operation_t *op);

#endif /* LICHTENSTEIN_APP_CANNABUS_LICHTENSTEIN_PRIVATE_H_ */

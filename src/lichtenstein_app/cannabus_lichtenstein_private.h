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
 */
int lichtenstein_cannabus_adc(void);

/**
 * Handles a write to the MUX control register. (Reg 0x011)
 */
int lichtenstein_cannabus_muxctrl(cannabus_operation_t *op);

/**
 * Handles a write to the differential receiver control register. (Reg 0x012)
 */
int lichtenstein_cannabus_diffrxctrl(cannabus_operation_t *op);

#endif /* LICHTENSTEIN_APP_CANNABUS_LICHTENSTEIN_PRIVATE_H_ */

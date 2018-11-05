/*
 * cannabus_lichtenstein.c
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */
#include "cannabus_lichtenstein.h"
#include "cannabus_lichtenstein_private.h"

#include "../cannabus/cannabus.h"

#include "../periph/adc.h"

#include "lichtenstein.h"

/**
 * Handles a CANnabus operation that isn't handled internally by the CANnabus
 * stack.
 */
int lichtenstein_cannabus_cb(cannabus_operation_t *op) {
	// handle requests (RTR = 1)
	if(op->rtr) {
		// read ADC state (reg = 0x10)
		if(op->reg == 0x010) {
			return lichtenstein_cannabus_adc();
		}
	}
	// handle register writes (RTR = 0)
	else {
		// TODO: handle
	}

	// if we fall down here, whatever operation was not handled
	LOG("unhandled CANnabus op: reg %x, rtr %d\n", op->reg, op->rtr);

	return kErrCannabusUnimplemented;
}


/**
 * Gets ADC data, converts it, and sends it. (Reg 0x010)
 */
int lichtenstein_cannabus_adc(void) {
	// get ADC data
	unsigned int adcData[4];
	adc_get_measure(&adcData);

	LOG("V = %u, I = %u, temp = %u, ref = %u", adcData[0], adcData[1], adcData[2], adcData[3]);

	// TODO: convert to meaningful data

	// TODO: send response
	return kErrSuccess;
}

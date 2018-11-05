/*
 * cannabus_lichtenstein.c
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */
#include "cannabus_lichtenstein.h"
#include "cannabus_lichtenstein_private.h"

#include "../cannabus/cannabus.h"

#include "../hw/output_mux.h"
#include "../hw/differential_rx.h"

#include "../periph/adc.h"

#include "lichtenstein.h"

#include <string.h>

/**
 * Handles a CANnabus operation that isn't handled internally by the CANnabus
 * stack.
 */
int lichtenstein_cannabus_cb(cannabus_operation_t *op) {
	// handle requests (RTR = 1)
	if(op->rtr) {
		// read ADC state (reg = 0x010)
		if(op->reg == 0x010) {
			return lichtenstein_cannabus_adc();
		}
	}
	// handle register writes (RTR = 0)
	else {
		// set mux state (reg = 0x011)
		if(op->reg == 0x011) {
			return lichtenstein_cannabus_muxctrl(op);
		}
		// set differential driver state (reg = 0x012)
		else if(op->reg == 0x012) {
			return lichtenstein_canbus_diffrxctrl(op);
		}
	}

	// if we fall down here, whatever operation was not handled
	LOG("unhandled CANnabus op: reg %x, rtr %d\n", op->reg, op->rtr);

	return kErrCannabusUnimplemented;
}


/**
 * Gets ADC data, converts it, and sends it. (Reg 0x010)
 */
int lichtenstein_cannabus_adc(void) {
	uint32_t temp;

	// get ADC data
	unsigned int adcData[4];
	adc_get_measure(&adcData);

	LOG("V = %u, I = %u, temp = %u, ref = %u", adcData[0], adcData[1], adcData[2], adcData[3]);

	// convert to 16 bit quantities
	uint16_t convertedData[4];
	memset(&convertedData, 0, sizeof(convertedData));

	// convert voltage: it's 1/2 of the input voltage
	temp = adc_normalize_value(adcData[0]);
	convertedData[0] = (uint16_t) temp;

	// convert current:
	temp = adc_normalize_value(adcData[1]);
	convertedData[1] = (uint16_t) temp;

	// convert temperature
	temp = adc_normalize_value(adcData[2]);
	convertedData[2] = (uint16_t) temp;

	// send raw vRef value
	convertedData[3] = (uint16_t) adcData[3];

	// send response
	cannabus_operation_t op;
	memset(&op, 0, sizeof(op));

	op.reg = 0x010;
	op.data_len = 8;

	memcpy(&op.data, &convertedData, 8);

	// send frame
	return cannabus_send_op(&op);
}


/**
 * Handles a write to the MUX control register. (Reg 0x011)
 */
int lichtenstein_cannabus_muxctrl(cannabus_operation_t *op) {
	// we must get two bytes of data
	if(op->data_len != 2) {
		return kErrCannabusInvalidFrameSize;
	}

//	LOG("mux status: %x %x\n", op->data[0], op->data[1]);

	// mux 0 state: zero is differential receiver, any other value is test gen
	if(op->data[0]) {
		mux_set_state(kMux0, kMuxStateTestGenerator);
	} else {
		mux_set_state(kMux0, kMuxStateDifferentialReceiver);
	}

	// mux 1 state: zero is differential receiver, any other value is test gen
	if(op->data[1]) {
		mux_set_state(kMux1, kMuxStateTestGenerator);
	} else {
		mux_set_state(kMux1, kMuxStateDifferentialReceiver);
	}

	// success
	return kErrSuccess;
}


/**
 * Handles a write to the differential receiver control register. (Reg 0x012)
 */
int lichtenstein_canbus_diffrxctrl(cannabus_operation_t *op) {
	// we must get a single byte of data
	if(op->data_len != 1) {
		return kErrCannabusInvalidFrameSize;
	}

	// if 0 driver is disabled, enabled otherwise
	if(op->data[0]) {
		diffrx_set_state(kDiffRxEnabled);
	} else {
		diffrx_set_state(kDiffRxDisabled);
	}

	// success
	return kErrSuccess;
}

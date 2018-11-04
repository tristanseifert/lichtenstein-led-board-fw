/*
 * cannabus.c
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */
#include "cannabus.h"
#include "cannabus_private.h"

#include "lichtenstein.h"

#include <string.h>

/// CANnabus version implemented by this driver
const uint8_t kCannabusVersion = 0x08;

/// Internal state of the CANnabus driver
static cannabus_state_t gState;



/**
 * Initializes the CANnabus and sets this node's address.
 */
int cannabus_init(cannabus_addr_t addr, uint8_t deviceType, cannabus_callbacks_t *callbacks) {
	int err;

	// first, clear all state and copy callbacks
	memset(&gState, 0, sizeof(gState));
	memcpy(&gState.callbacks, callbacks, sizeof(cannabus_callbacks_t));

	// set filter for the broadcast address
	err = gState.callbacks.can_config_filter(1, 0x07FFF800, (0xFFFF << 11));

	if(err < kErrSuccess) {
		LOG("couldn't set broadcast filter: %d\n", err);
		return err;
	}

	// then, set the node address and device type
	err = cannabus_set_address(addr);

	if(err < kErrSuccess) {
		LOG("couldn't assign id %x: %d\n", addr, err);
		return err;
	}

	gState.deviceType = deviceType;

	// start the CAN bus
	err = gState.callbacks.can_init();
	return err;
}

/**
 * Changes the node's address.
 */
int cannabus_set_address(cannabus_addr_t addr) {
	int err;

	// ensure it's not the broadcast address
	if(addr == 0xFFFF) {
		return kErrInvalidArgs;
	}

	// set address in state
	gState.address = addr;
	LOG("CANnabus address: %x\n", addr);

	// update the filters on the CAN peripheral
	err = gState.callbacks.can_config_filter(2, 0x07FFF800, (uint32_t) (addr << 11));
	return err;
}



/**
 * Gets any waiting messages from the CAN bus driver and processes them.
 *
 * @returns A negative error code, or the number of messages processed.
 */
int cannabus_process(void) {
	int err, messages = 0;

	// continue as long as we have messages waiting
	while(gState.callbacks.can_rx_waiting()) {
		cannabus_can_frame_t frame;
		cannabus_operation_t op;

		// dequeue a message
		err = gState.callbacks.can_rx_message(&frame);

		if(err < kErrSuccess) {
			return err;
		}

		// convert this message into a CANnabus operation
		err = cannabus_conv_frame_to_op(&frame, &op);

		if(err < kErrSuccess) {
			return err;
		}

		// handle the message internally if possible
		if(cannabus_is_op_internal(&op)) {
			err = cannabus_internal_op(&op);
		}
		// otherwise, call into application code
		else {
			err = gState.callbacks.handle_operation(&op);
		}

		if(err < kErrSuccess) {
			return err;
		}

		// increment the counter
		messages++;
	}

	// return number of processed messages
	return messages;
}



/**
 * Sends the given operation on the bus.
 */
int cannabus_send_op(cannabus_operation_t *op) {
	int err;
	cannabus_can_frame_t frame;

	// convert the operation to a CAN frame
	err = cannabus_conv_op_to_frame(op, &frame);

	if(err < kErrSuccess) {
		return err;
	}

	// now, transmit the frame
	return gState.callbacks.can_tx_message(&frame);
}



/**
 * Converts a received CAN frame into a CANnabus operation.
 */
int cannabus_conv_frame_to_op(cannabus_can_frame_t *frame, cannabus_operation_t *op) {
	// extract relevant fields from identifier
	uint16_t nodeId = (uint16_t) (frame->identifier >> 11) & 0xFFFF;
	uint16_t reg = (uint16_t) (frame->identifier & 0x7FF);
//	uint8_t priority = (frame->identifier >> 27) & 0x03;

	// check this frame is destined for us? insurance against fucked filters
	if(nodeId != gState.address && nodeId != 0xFFFF) {
		return kErrCannabusNodeIdMismatch;
	}

	// copy fields from the CAN frame
	op->broadcast = (nodeId == 0xFFFF) ? 1 : 0;
	op->data_len = frame->data_len;
	op->reg = reg;
	op->rtr = frame->rtr;

	memcpy(&op->data, &frame->data, 8);

	// message was ok
	return kErrSuccess;
}

/**
 * Converts a CANnabus operation into a CAN frame to be transmitted.
 *
 * TODO: handle priority here lmfao
 */
int cannabus_conv_op_to_frame(cannabus_operation_t *op, cannabus_can_frame_t *frame) {
	// build the identifier
	uint32_t identifier = (uint32_t) ((op->reg & 0x7FF) | (gState.address << 11));

	// copy parameters to frame
	frame->identifier = identifier;
	frame->rtr = op->rtr;
	frame->data_len = op->data_len;

	memcpy(&frame->data, &op->data, 8);

	return kErrSuccess;
}



/**
 * Is this CANnabus operation internal, e.g. is it something specified in the
 * CANnabus protocol that all nodes interpret, thus something we handle internal
 * to the protocol handler?
 */
bool cannabus_is_op_internal(cannabus_operation_t *op) {
	// CANnabus protocol handles 0x0000 - 0x0003 rn
	if(op->reg <= 0x0003) {
		return true;
	}

	// message isn't handled internally otherwise
	return false;
}

/**
 * Handles an internal CANnabus operation.
 */
int cannabus_internal_op(cannabus_operation_t *op) {
	switch(op->reg) {
		// device id register
		case 0x000:
			return cannabus_internal_reg_deviceid(op);


		// unknown registers
		default:
			return kErrCannabusUnimplemented;
	}
}

/**
 * Handles reads/writes to the Device ID (0x0000) register.
 */
int cannabus_internal_reg_deviceid(cannabus_operation_t *_op) {
	int err = kErrCannabusUnimplemented;

	// is this a broadcast frame?
	if(_op->broadcast) {
		// if so, respond with our device id register
		err = cannabus_internal_reg_deviceid_respond(_op);
	} else {
		// if it's an RTR frame, send the device id register
		if(_op->rtr) {
			err = cannabus_internal_reg_deviceid_respond(_op);
		}
		// otherwise, process a write of the register data
		else {
			// make sure that we received at least two bytes
			if(_op->data_len < 2) {
				return kErrCannabusInvalidFrameSize;
			}

			// the only value that can be written is the device id
			cannabus_addr_t deviceId;

			deviceId = (uint16_t) ((_op->data[0] << 8) | (_op->data[1]));

			err = cannabus_set_address(deviceId);
			LOG("updated device id: %x", deviceId);
		}
	}

	return err;
}


/**
 * Sends the device ID response.
 */
int cannabus_internal_reg_deviceid_respond(cannabus_operation_t *_op) {
	int err;

	// build a CAN frame to transmit
	cannabus_operation_t op;

	op.broadcast = 0;
	op.reg = 0x000;
	op.rtr = 0;
	op.data_len = 8;

	// clear data field
	memset(&op.data, 0, 8);

	// device id
	op.data[0] = (uint8_t) ((gState.address & 0xFF00) >> 8);
	op.data[1] = (uint8_t) (gState.address & 0x00FF);

	// supported CANnabus version and device type
	op.data[2] = kCannabusVersion;
	op.data[3] = gState.deviceType;

	// TODO: handle firmware update capabilities field

	// transmit the frame
	err = cannabus_send_op(&op);

	// done
	return err;
}

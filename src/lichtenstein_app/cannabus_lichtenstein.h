/*
 * cannabus_lichtenstein.h
 *
 * Lichtenstein app specific CANnabus packet handler
 *
 *  Created on: Nov 4, 2018
 *      Author: tristan
 */
#include "../cannabus/cannabus.h"

#ifndef CANNABUS_LICHTENSTEIN_H_
#define CANNABUS_LICHTENSTEIN_H_


/**
 * Handles a CANnabus operation that isn't handled internally by the CANnabus
 * stack.
 */
int lichtenstein_cannabus_cb(cannabus_operation_t *op);

#endif /* CANNABUS_LICHTENSTEIN_H_ */

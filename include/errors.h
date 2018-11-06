/*
 * errors.h
 *
 * Error codes used throughout the software.
 *
 *  Created on: Nov 3, 2018
 *      Author: tristan
 */

#ifndef INCLUDE_ERRORS_H_
#define INCLUDE_ERRORS_H_

enum {
	kErrSuccess					= 0,

	// generic errors
	kErrInvalidArgs				= -1000,
	kErrInsufficientResources	= -1001,
	/// could not create a task (xTaskCreate* returned NULL)
	kErrTaskCreationFailed		= -1002,
	/// could not create a semaphore (xSemaphoreCreate* returned NULL)
	kErrSemaphoreCreationFailed	= -1003,

};


#endif /* INCLUDE_ERRORS_H_ */

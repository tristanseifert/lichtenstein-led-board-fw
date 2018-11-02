/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include "lichtenstein.h"

#include "output_mux.h"
#include "differential_rx.h"
#include "ws2811_generator.h"
#include "spi.h"
#include "canbus.h"

#include "gitcommit.h"

int main(int argc __attribute__((__unused__)), char* argv[]__attribute__((__unused__))) {
	// initialize trace
	trace_initialize();

	trace_printf("lichtenstein-led-fw %s", GIT_INFO);

	// initialize hardware
	mux_init();
	diffrx_init();
	ws2811_init();
	spi_init();
	canbus_init();

	// read configuration from flash

	// configure CAN and start it
	canbus_start();

	// reset the outputs, then enable differential driver
	ws2811_send_pixel(300, kWS2811PixelTypeRGBW, 0x00000000);

	diffrx_set_state(kDiffRxEnabled);

	mux_set_state(kMux0, kMuxStateDifferentialReceiver);
	mux_set_state(kMux1, kMuxStateDifferentialReceiver);

  // Infinite loop
  while (1)
    {
       // Add your code here.
    }

  return 0;
}



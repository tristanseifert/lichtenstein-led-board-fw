#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lichtenstein.h"

#include "hw/status.h"
#include "hw/output_mux.h"
#include "hw/differential_rx.h"
#include "hw/ws2811_generator.h"
#include "periph/spi.h"
#include "periph/canbus.h"
#include "periph/adc.h"

#include "hw/spi_flash.h"
#include "hw/nvram.h"

#include "cannabus/cannabus.h"

#include "lichtenstein_app/cannabus_init.h"

#include "gitcommit.h"

/**
 * Performs initialization of all hardware.
 */
static void init_hardware(void) {
#ifdef STM32F042
	// first, initialize the hardware
	status_init();
	mux_init();
	diffrx_init();
	ws2811_init();
	spi_init();
	can_init();
	adc_init();

	// then, init flash and read the config
	spiflash_init();
	nvram_init();
#endif
#ifdef STM32F072
	status_init();
	mux_init();
	diffrx_init();
	spi_init();
	can_init();
	adc_init();
#endif
}



/**
 * Application entry point
 */
int main(int argc __attribute__((__unused__)), char* argv[]__attribute__((__unused__))) {
	int err;

	// initialize trace
	trace_initialize();
	LOG("lichtenstein-led-fw %s\n", GIT_INFO);

#ifdef STM32F042
	LOG_PUTS("hw: STM32F042");
#endif
#ifdef STM32F072
	LOG_PUTS("hw: STM32F072");
#endif

	// initialize hardware/peripherals
	init_hardware();

	// initialize CANnabus
	lichtenstein_cannabus_init();

	// reset the outputs, then enable differential driver
	ws2811_send_pixel(300, kWS2811PixelTypeRGBW, 0x00000000);

	diffrx_set_state(kDiffRxEnabled);

	mux_set_state(kMux0, kMuxStateDifferentialReceiver);
	mux_set_state(kMux1, kMuxStateDifferentialReceiver);

	// start FreeRTOS scheduler. this should not return
	vTaskStartScheduler();

	// enter main loop. status1 toggles when we're busy
	while(1) {
		status_set(kStatusLED1, true);

		// process waiting CANnabus messages
		err = cannabus_process();

		if(err < 0) {
			LOG("cannabus_process failed: %d", err);
		}

		// clear the busy indicator
		status_set(kStatusLED1, false);

		// wait for an interrupt
		__WFI();
	}

	// never should get here
	return 0;
}



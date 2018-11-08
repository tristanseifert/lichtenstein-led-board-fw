#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lichtenstein.h"

#include "hw/status.h"
#include "hw/output_mux.h"
#include "hw/differential_rx.h"
#include "hw/ws2812_generator.h"
#include "periph/spi.h"
#include "periph/canbus.h"
#include "periph/adc.h"

#include "hw/spi_flash.h"
#include "hw/nvram.h"

#include "cannabus/cannabus.h"

#include "lichtenstein_app/version.h"
#include "lichtenstein_app/cannabus_init.h"
#include "lichtenstein_app/led_post.h"

#include "FreeRTOS.h"
#include "task.h"

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
	ws2812_init();
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
	ws2812_init();
	spi_init();
	can_init();
	adc_init();
#endif
}



/**
 * Application entry point
 */
__attribute__((noreturn)) int main(int argc __attribute__((__unused__)), char* argv[]__attribute__((__unused__))) {
#ifdef DEBUG
		// initialize trace
		trace_initialize();
		LOG("lichtenstein-led-fw (0x%x, built %u) %s\n",
				kLichtensteinVersion.version, kLichtensteinVersion.buildTime,
				GIT_INFO);

	#ifdef STM32F042
		LOG_PUTS("hw: STM32F042");
	#endif
	#ifdef STM32F072
		LOG_PUTS("hw: STM32F072");
	#endif
#endif

	// initialize hardware/peripherals
	init_hardware();

	// initialize CANnabus
	lichtenstein_cannabus_init();

	// perform LED POST
	lichtenstein_led_post();

	// start FreeRTOS scheduler. this should not return
	vTaskStartScheduler();
	NVIC_SystemReset();
}



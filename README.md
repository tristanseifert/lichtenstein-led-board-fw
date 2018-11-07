# Lichtenstein LED Board Firmware
Firmware for the Lichtenstein LED boards: a small board that receives two differential WS2812 data streams and outputs them to LED strips.

This board contains a small STM320F042 microcontroller that measures voltage and current, and allows enabling/disabling the differential receiver, as well as selecting each output to be either the received data, or the output of an internal WS2812 waveform generator. All of these features are controlled via CANnabus, a Lichtenstein-specific CAN bus for control of remote nodes.

Firmware upgrades of the device are possible by sending updated firmware over CANnabus. The firmware is written to an external SPI flash, and copied to the internal flash of the system by the bootloader.

To be used in conjunction with the [lichtenstein-led-board-loader](https://github.com/tristanseifert/lichtenstein-led-board-loader) as its bootloader.

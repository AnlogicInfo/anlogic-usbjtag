Description
======

A fully-opensource bit-banging USB-JTAG adapter for Anlogic FPGA debugger.

Dependence
--------------

libopencm3 library, GNU make, GNU ARM-baremetal-toolchain (arm-none-eabi-gcc), dfu-utils.

Hardware requirement and IO assignment
--------------

It requires a STM32F103C8T6 or equal microcontroller(GD32/MM32/BLM32) with 8MHz external crystal.

PA0 - JTAG TDO

PA1 - JTAG TMS

PA2 - JTAG TDI

PA3 - JTAG TMS

PA8  - USB_PULLUP

PA9  - BOOT_ENTRY

PB12 - STATUS_LED with positive polarity


Build
--------------

Enter the src directory, and type "make"

Flash
--------------

This firmware requires a bootloader which called dapboot.

Try my branch https://github.com/rgwan/dapboot, target is "MSC".

And use "make dfu" to flash your microcontroller via USB DFU protocol.

License
--------------

LGPLv3

Contribution
--------------

Zhiyuan Wan 2017

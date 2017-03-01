Description
======

A USB-JTAG debugger for Anlogic platform. For some problem, the source has been removed and the license has been changed.

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

PB11 - STATUS_LED with positive polarity

Flash
--------------

This firmware contains a bootloader, which is opensource from dapboot. The firmware folder contains three files:

bootloader.fwb, The bootloader, you can use STVP or stm32flash to flash into a blank chip.

firmware.fwb, The firmware, you HAVE TO use dfu-utils or anlogic dfu tool to flash into a chip which already contains a bootloader.

flash.fwb, The firmware and the bootloader. You can use STVP or stm32flash to flash it into a blank chip, and it does work, not require you use dfu-utils to flash again.

I'm sorry I can't fully open this stuff's source, but I hope you like it!

License
--------------

EULA

Contribution
--------------

Zhiyuan Wan.

Anlogic Information.

2017

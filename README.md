# tuner-pedal

This file contains the STM32 project for creating a guitar tuner pedal.

Currently the pedal code is running on the STM32L432KC. Pin D3 is connected to the ADC. To read the pedal output open a serial tty terminal with a baud rate of 115200. The pedal will read signals in the range of 0-3.3V with frequencies ranging from 0-1000Hz.

## Running on Mac

```sh
# Record the usb name
ls /dev/tty.usb*

screen /dev/tty.usbXXXXXXXXX 115200
```
Overview
========

The lpc_adc_burst example shows how to use LPC ADC driver with the burst mode.

In this example, the internal temperature sensor is used to created the input analog signal. When user type in any key from the keyboard, the burst mode is enabled. Then the conversion sequence A would be started automatically, till the burst would be disabled in conversion completed ISR. 


Toolchain supported
===================
- IAR embedded Workbench 8.11.3
- Keil MDK 5.23
- MCUXpresso10.1.0
- GCC ARM Embedded 6-2017-q2

Hardware requirements
=====================
- Micro USB cable
- LPCXpresso54608 board
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect a micro USB cable between the host PC and the LPC-Link USB port (J8) on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
Press any key from keyboard and trigger the conversion.
The log below shows example output of the ADC burst example in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ADC burst example.
ADC_DoSelfCalibration() Done.
Configuration Done.
gAdcResultInfoStruct.result        = 672
gAdcResultInfoStruct.channelNumber = 0
gAdcResultInfoStruct.overrunFlag   = 1

gAdcResultInfoStruct.result        = 679
gAdcResultInfoStruct.channelNumber = 0
gAdcResultInfoStruct.overrunFlag   = 1

gAdcResultInfoStruct.result        = 671
gAdcResultInfoStruct.channelNumber = 0
gAdcResultInfoStruct.overrunFlag   = 1

...

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Customization options
=====================


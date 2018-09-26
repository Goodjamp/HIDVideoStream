Overview
========

The lpc_adc_basic example shows how to use LPC ADC driver in the simplest way.

In this example, the internal temperature sensor is used to created the input analog signal. When user type in any key from the keyboard, the software trigger API is called to start the conversion. Then it polls the conversion sequence A's flag till the conversion is completed. When the conversion is completed, just print the conversion result to terminal.



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
The log below shows example output of the ADC basic example in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ADC basic example.
ADC_DoSelfCalibration() Done.
Configuration Done.
adcResultInfoStruct.result        = 675
adcResultInfoStruct.channelNumber = 0
adcResultInfoStruct.overrunFlag   = 0

adcResultInfoStruct.result        = 676
adcResultInfoStruct.channelNumber = 0
adcResultInfoStruct.overrunFlag   = 0

adcResultInfoStruct.result        = 675
adcResultInfoStruct.channelNumber = 0
adcResultInfoStruct.overrunFlag   = 0

...

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Customization options
=====================


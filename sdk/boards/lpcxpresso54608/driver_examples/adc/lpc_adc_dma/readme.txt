Overview
========

The lpc_adc_dma example shows how to use LPC ADC driver with DMA.

In this example, the internal temperature sensor is used to created the input analog signal. When user type in any key from the keyboard, the software trigger API is called to start the conversion. When the ADC conversion is completed, it would trigger the DMA to move the ADC conversion result from ADC conversion data register to user indicated memory. Then the main loop waits for the transfer to be done and print the result to terminal.


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
The log below shows example output of the ADC DMA example in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ADC DMA example.
ADC_DoSelfCalibration() Done.
Configuration Done.
Type in any key to trigger the conversion ...
Conversion word : 0x80022A70
Conversion value: 679

Conversion word : 0x80022930
Conversion value: 659

Conversion word : 0x800228F0
Conversion value: 655

Conversion word : 0x80022920
Conversion value: 658

Conversion word : 0x80022920
Conversion value: 658

...

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Customization options
=====================


Overview
========
Demonstrates the DMIC working with I2S. Audio is converted to samples in the DMIC module.
Then, the data is placed into the I2S buffer. Last, it is read from the I2S buffer and sent
to the CODEC, then the audio data will be output to Lineout of CODEC.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
- Earphone/headset
- Jumper wire

Board settings
==============
This example shows how to route the DMIC PCM data to I2S(Flexcomm7),So the i2s_data(Flexcomm7)pin 
should connect to the input pin(DACDAT) of CODEC, so the following jumper should have:

1.Connect J9-20 to J9-10
2.Remove the resistor JS8
3.Connect an earphone or speaker to line out


Prepare the Demo
================
This demo was written on a LPCXpresso 54608 board, PCB rev A

1.  Connect a micro USB cable between the host PC and the LPC-Link USB port (J8) on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.

Running the demo
================
1.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

The following lines are printed to the serial terminal when the demo program is executed.

Configure I2C

Configure WM8904 codec

Configure I2S

2. This example transfers data from DMIC to Codec. Connect headphone/earphone on audio out of the board.
   Speak on DMIC and you can listen voice on audio out.
Customization options
=====================


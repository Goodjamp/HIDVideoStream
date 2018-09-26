Overview
========
CMSIS-Driver defines generic peripheral driver interfaces for middleware making it reusable across a wide 
range of supported microcontroller devices. The API connects microcontroller peripherals with middleware 
that implements for example communication stacks, file systems, or graphic user interfaces. 
More information and usage methord please refer to http://www.keil.com/pack/doc/cmsis/Driver/html/index.html.

The cmsis_spi_interrupt_transfer example shows how to use CMSIS SPI driver in interrupt way:

In this example , one spi instance used as master and another spi instance used as slave in the same board.

1. SPI master send/received data to/from SPI slave in interrupt transfer . (SPI Slave using interrupt to receive/send the data)


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
Populate jumper JP6.
Connect SPI9 pins to SPI3 pins:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
INSTANCE0(SPI9)     CONNECTS TO         INSTANCE1(SPI3)
Pin Name   Board Location     Pin Name  Board Location
MISO       J9 pin 11          MISO      J14 pin 3
MOSI       J9 pin 13          MOSI      J14 pin 2
SCK        J9 pin 9           SCK       J14 pin 4
PCS0       J9 pin 15          PCS0      J14 pin 1

Make sure JP-6 is fitted to use the PMOD SPI port.
Also in JP8 pin 1-3 and 7-9 should be shorted.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Prepare the Demo
================
1.  Connect a micro USB cable between the PC host and the LPC-Link USB port (J8) on the board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Reset the SoC and run the project.

Running the demo
================
When the example runs successfully, you can see the similar information from the terminal as below.

SPI CMSIS interrupt transfer start.
This example use one spi instance as master and another as slave on one board.
Master and slave are both use interrupt way.
Please make sure you make the correct line connection. Basically, the connection is: 
SPI_master -- SPI_slave   
   CLK      --    CLK  
   PCS      --    PCS  
   MOSI     --    MOSI 
   MISO     --    MISO 
This is SPI_SlaveSignalEvent_t
This is SPI_MasterSignalEvent_t
 
SPI transfer all data matched! 


Customization options
=====================


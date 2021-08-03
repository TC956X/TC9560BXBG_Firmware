Release Date: 05 April 2018
Release Version: 1.2
===============================================================================

Introduction:
=============
The folder contains a Keil project, for HSIC FW.

Requirements:
============
1. You will need to have Keil nVision IDE Version 5 (Version 4 should be OK. TAEC tested with Version 5). A valid Keil 
   licence is required, as the firmware size is larger than 32k. 
2. Keil ULink-me JTAG connector is used.

Procedures
==========
1. Build the project using Keil IDE.
2. Download it to TC9560BXBG SRAM and run it.
3. Open UART/Serial console with 115200 baud rate, 8data bits, 1stops bits no flow control.

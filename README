Introduction
============

This fork fixes some problems and adds the functionality to control a GPIO extender over I2C to control filter selection.
This is a work in progress.

Plese note that the code in this fork and the one from ted051 is not compatible with the version hosted by Pavel.
Therefore follow the instructions below to install and update the base image to a working version.

Currently the code running on the STM32 is untouched and has many bugs.
Further known bugs are popcorn sound during reception depending on selected FFT type.

Installation
============

Basic Environment
-----------------

Commands to install and run this program on Red Pitaya:

Download and unpack onto a fat32 formatted SD card:
from  http://pavel-demin.github.io/red-pitaya-notes/sdr-transceiver-emb/

Embedded Receiver
-----------------

Update to use code from this repository

copy sdr-transceiver-emb.c into apps/sdr_transceiver_emb

copy start.sh into the topmost directory of the card

insert the card, connect with ssh

commands:                     // comment
	cd apps
	cd s*emb                 // -> sdr_transceiver_emb
	rw
	apk add make gcc patch   //only first time, activate make 
	apk add make patch       //only first time, activate patch and gcc
	lbu commit -d            //only first time
	rm sdr-transceiver-emb
	make 
	ro 
	reboot

HW Connection
=============

I2S on Red Pitaya
-----------------

The I2S interface is sharing pins with the ALEX interface. So, the two can’t be used simultaneously.
The supported I2S audio codecs are TLV320AIC23B and WM8731. The I2S audio
codecs should be clocked with a 12.288 MHz oscillator crystal.

In terms of the codec board's signals the connection should look like the following:

E1:
DIO4_N - SCK -- E1/12
DIO5_N - MISO -- E1/14
DIO6_N - MOSI -- E1/16
DIO7_N - ADCL -- E1/18

The codec board's I2C interface should be connected to the I2C pins of the
extension connector E2.

E2:
SDA -- E2/10
SCL -- E2/09

UART STM32 -- RedPitaya
-----------------------

STM32
RX = CN4 / Pin1
TX = CN4 / Pin2
GND = CN7 / Pin7		

RedPitaya
RX = E2 Pin 8
TX = E2 Pin 7
GND = E2 Pin 12		

Low-Pass filter
---------------

For Low-pass filter switching the PCA9555 GPIO extender is used at I2C address 0x21.
It's connected on the same bus as the I2S (E2 connector)
First four outputs are used as follows:
0x00, 1.8 MHz, 3dB 2.5MHz
0x01, 3.5 MHz, 3dB 5MHz
0x02, 7.0 MHz, 3dB 8.6MHz
0x03, 10 MHz, 3dB 15.5MHz
0x05, 18 MHz, 3dB 25MHz
0x08, 28 MHz, 3dB 35MHz
0x09, 50MHz, 3dB 55MHz
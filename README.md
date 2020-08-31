Introduction
============

This fork fixes some problems and adds the functionality to control a GPIO extender over I2C to control filter selection.
This is a work in progress.

Plese note that the code in this fork and the one from ted051 is not compatible with the version hosted by Pavel.
Therefore follow the instructions below to install and update the base image. Actually one needs to copy sdr-transceiver-emb.c to the RedPitaya and rebuild as described below.

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

Insert the card, connect with ssh

```console
root@bar:~$ cd apps/sdr-transceiver-emb
root@bar:~$ rw
root@bar:~$ apk add make gcc patch   #only first time, activate make, gcc and patch
root@bar:~$ lbu commit -d            #only first time
root@bar:~$ rm sdr-transceiver-emb
root@bar:~$ make 
root@bar:~$ ro 
root@bar:~$ reboot
```

HW Connection
=============

I2S (Red Pitaya)
-----------------

The I2S interface is sharing pins with the ALEX interface. So, the two can’t be used simultaneously.
The supported I2S audio codecs are TLV320AIC23B and WM8731. The I2S audio
codecs should be clocked with a 12.288 MHz oscillator crystal.

In terms of the codec board's signals the connection should look like the following:

E1:
* DIO4_N - SCK -- E1/12
* DIO5_N - MISO -- E1/14
* DIO6_N - MOSI -- E1/16
* DIO7_N - ADCL -- E1/18

The codec board's I2C interface should be connected to the I2C pins of the
extension connector E2.

E2:
* SDA -- E2/10
* SCL -- E2/09

Low-Pass filter (Red Pitaya)
----------------------------

This is a new feature. For Low-pass filter switching the PCA9555 GPIO extender is used with I2C address 0x21.
It's connected to the same bus as the I2S (E2 connector)
First four outputs are used as follows:
* 0x00, 1.8 MHz, 3dB 2.5MHz
* 0x01, 3.5 MHz, 3dB 5MHz
* 0x02, 7.0 MHz, 3dB 8.6MHz
* 0x03, 10 MHz, 3dB 15.5MHz
* 0x05, 18 MHz, 3dB 25MHz
* 0x08, 28 MHz, 3dB 35MHz
* 0x09, 50MHz, 3dB 55MHz
This configuration fits the Hermes filter which can be acquired from the bay.

UART STM32 -- RedPitaya
-----------------------
The communication between RedPitaya and STM32 is based on UART. Baud = 115200, Stopbits = 1, Parity = None	

STM32
* RX = CN4 / Pin1
* TX = CN4 / Pin2
* GND = CN7 / Pin7		

RedPitaya
* RX = E2 Pin 8
* TX = E2 Pin 7
* GND = E2 Pin 12		

Attenuator (STM32)
------------------
An RX/TX attenuator can be attached to the STM32 board and is controlled with SPI.

|PE4306 Pin       |		STM32 Pin |
|-----------------|---------------|
|Pin1 = DATA 	  |   CN4/8|
|Pin2 = +3V  	  |	 CN6/4|
|Pin3 = Ground 	  |	 CN6/6|
|Pin4 = Clock 	  |	 CN4/3|
|Pin5 = LatchEnable |	 CN4/5|

PTT (STM32)
-----------
PTT is connected to the STM32.
* PTT = CN7/1

Stockton Coupler (STM32)
------------------------
To monitor the forward/reflected power the analog signals from the coupler can be connected to the STM32.

* Power forward	CN5/4
* Power reflected	CN5/3

Software
========

Communication Protocol
----------------------

The protocol is used in both communication directions. Used communication protocol structure is as follows:

```
| Code, 1Byte | Data, 4Byte | CRC8, 1Byte |
```
The CRC check is currently ignored.

Codes used in the Protocol:
|Code       |		Direction |   Data & Description |
|-----------------|-----------|-----------|
|1 	  |   STM32>RedPitaya   |   uint32 0x006DDD00 = 7200000 Hz, RX Frequency |
|2 	  |   STM32>RedPitaya   |   uint32 0x006DDD00 = 7200000 Hz, TX Frequency |
|3 	  |   STM32>RedPitaya   |   uint32 0x00 ... 0x09, Speaker Volume |
|4 	  |   STM32>RedPitaya   |   uint32 0x00 ... 0x09, Microphone Volume |
|5 	  |   STM32>RedPitaya   |   uint32 0x00 ... 0x09, Filter Bandwidth |
|6 	  |   STM32>RedPitaya   |   uint32 0x00 ... 0x06 ("CWL"	"CWU" "LSB" "USB" "AM" "FM" "DIGI" "SAM"), Demodulator Mode |
|7 	  |   STM32>RedPitaya   |   uint32 0x00 ... 0x04 ("Off" "LONG" "SLOW" "MED"	"FAST"), AGC Mode |
|8 	  |   STM32>RedPitaya   |   uint32 0x00 or 0x01, TX On |
|9 	  |   STM32>RedPitaya   |   uint32 0x00, Exit Main |
|10 	  |   STM32>RedPitaya   |   Not implemented, Get Build Number |
|11 	  |   STM32>RedPitaya   |   CW Threshold High |
|12 	  |   STM32>RedPitaya   |   CW Threshold Low |
|13 	  |   STM32>RedPitaya   |   uint32 0x00 or 0x01, Auto Notch filter |
|14 	  |   STM32>RedPitaya   |   uint32 0x00 or 0x01, Noise Blanker |
|15 	  |   STM32>RedPitaya   |   uint32 0x00 or 0x01, Spectral NR |
|16 	  |   STM32>RedPitaya   |   uint32 0x00 or 0x01, LMS NR |
|17 	  |   STM32>RedPitaya   |   FFT (1=RF, 2=AF, 0=off), detector_mode, avg_mode, FFT_length, FFT Parameter |
|85 	  |   RedPitaya>STM32   |   uint32 0x00 ... 0x0578 (0= 0 dBm, 1400 = -140 dBm (min)), Sinal Strength |
|86 	  |   RedPitaya>STM32   |   CW Keyup |
|89 	  |   RedPitaya>STM32   |   CW Keydown |
|82 	  |   RedPitaya>STM32   |   CW Amplitude |
|100 	  |   RedPitaya>STM32   |   FFT - Data (4Samples) |
|86 	  |   RedPitaya>STM32   |   Begin of line FFT data |
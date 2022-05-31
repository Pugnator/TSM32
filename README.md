# STM32 TSM for Harleys

Hi! You're here because of your broken TSM or may be you want to get rid of it and still have your dashboard working. Or may be you want to have some custom turn modes for your **Sportie**.
It was created for my Harley Sportster 2004 but I bet it will work with Harleys of the same generation. Refer to  [Sportsterpedia](http://sportsterpedia.com/doku.php) for more information.

> **Note:** The **TSM32** is still work in progress. I have almost no time to investigate the proto Harley uses and use it as is. In search of DIY guys who needs such a device too.


## Description

It's a custom PCB, firmware for *STM32F103* and 3D enclosure model to build your own TSM. Firmware is to be build with GCC.


##  What works

* Starter control
* Blinker on and off, long press to do 3 blinks (kinda overtake mode)
* Hazard mode
* MPU9250 support
* J1850 read mostly works


## What doesn't work yet

* Auto turn off
* J1850 transfer is not tested yet
* J1850 check engine reset

## How to build

    Just issue **make** command in the root folder.

## PCB board

PCB, gerber files and schematic are located in a corresponding folders.

## Enclosure

STL files to print with 3D printer are also available. I used 0.3mm layers and PETG to print it. 


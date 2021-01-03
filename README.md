# GoPro Control

This project contains the firmware for controlling a GoPro using MAVLINK

Feature list
====

-- 

How to build
====
 
Make sure the esp-idf (SDK) is installed and ready for use. This code is based on version 4.1 of the SDK.
 
`idf.py menuconfig`
This starts the configuration menu. Apply the correct settings for your development environment
 
`idf.py build`
Compile the firmware

`idf.py -p <serial device> flash`
Install firmware on the device
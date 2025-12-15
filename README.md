# HEATSeeker — Smart Glasses + Remote Rescue Robot

**Project Website:** https://rueyday.github.io/HEATSeeker/  
**Video Demo:** https://www.youtube.com/watch?v=UyH7BJMXlRg  

**Contributors:** Jungeun Seo · Sophie Yang · Ruey Day · Yi Keen Lim  
**Course:** EECS 373 (Embedded Systems), University of Michigan

## Repository Overview

This repository contains the firmware and embedded code for HEATSeeker — a thermal-guided search-and-rescue robot with head-mounted smart glasses.

## What Each Folder Is

### `diff_drive/`  
Firmware for the rescue robot’s differential-drive system.  
Includes:
- Motor control routines  
- Thermal camera capture  
- XBee transmit/receive logic  

### `joystick/`  
Handheld controller firmware:
- STM32 joystick + rotary knob
- Mode buttons
- XBee command transmitter

### `tiny_board/`  
HUD glasses firmware:
- LCD thermal visualization
- Battery + mode indicators
- XBee receive logic

### `python/`  
Supporting Python code for offline visualization, logging, or testing

## Requirements

### Tools
- **STM32CubeIDE**
- **ST-Link**
- USB drivers for your device

### Hardware
- STM32 boards (one per subsystem)
- XBee radios
- AMG8833 thermal sensor (robot)
- Joystick + rotary knob (controller)
- LCD/OLED (HUD)
- Motors + driver (robot)

## Summary

This code is a basic opensource firmware for 3-axis brushless camera gimbals, such as those shipping with SimpleBGC32 firmware, but completely independent of the original SimpleBGC32 or STorM32 firmware.

It is still in a relatively early stage although it's some way beyond a Proof of Concept now.

## Licensing

3-Clause BSD license.

Some code informed, advised or drafted by AI.

## Hardware compatibility

* STM32 F3 or F1 series MCUs only.
* 3 axes required.
* 1x or 2x IMU (main and "frame"), Invensene MPU-6xxx series or NT-IMU.
* Encoders are required on all axes.  Early encoderless support available with two IMUs.
* Angles other than 90 degrees between joint axes *are* supported and are seamlessly detected during calibration.  No assumptions are made about joint order or angles or IMU mounting angles, but structure must be rigid.
* Supported encoders are: only AS5600 for onboard encoder, or any model (don't care) when connected to SBGC32_I2C_Drv extension boards or NT motor+encoder boards.

* Bluetooth remote works, tested with BaseCam RM-1

CAN communication is *not* supported.  NT bus early support.

## Software compatibility

Is it possible to go back to the original firmware? *Yes*, it is.

SimpleBGC32 GUI communication is *not* supported, neither is STorM32 GUI.  They're doable but many of the settings will be different due to different maths algorithms used, and many functionalities are likely never being implemented, such as scripting, variables, MAVLink, extra motors, RC virtual channels, external reference data, filesystem etc.

The SimpleBGC serial API is only supported to the (small) extent needed for bluetooth remote support (tested with the RM-1 model) and the 2-hand base controls.  OpenBGC implements its own extensions to the SimpleBGC serial API for setup, control, monitoring.

A relatively complete GUI companion app is present at utils/app.py.

## OpenBGC -- SimpleBGC32 opensource replacement

This repository hosts a basic firmware for camera gimbals, specifically for the gimbals that run SimpleBGC32 firmware created by BasecamElectronics and some STorM32 boards by OlliW (see next section).

There are many commercial models that use the SimpleBGC32 firmware and are manufactured by others.  They use either the electronic boards from BaseCam / SimpleBGC32 (the main controller board, the motor driver+encoder extension boards, etc.) or custom boards based on the SimpleBGC32 schematics, probably with proper licensing from BasecamElectronics.

This is the case with the excellent PilotFly H2 gimbal, which this code was first developed on.  Very robust design and seemingly full firmware compatibility with other models and manufacturers as well as with DIY gimbals using SimpleBGC32 controllers or their clones.  PilotFly H2 uses a [tiny, round controller PCB](board-photos/pilotfly-h2-controller-front.jpg) which is not offered by BasecamElectronics but matches their schematics and the gimbal overall connection layout matches the SimpleBGC32 recommended encoder-enabled modular design.

There may be hardware features of these boards that are not used by OpenBGC.  Right now there are IMU drivers with an AHRS algorithm, encoder reading, LED control, some autodetect/autocalibrate/self-test and bootloader/flashing management, various BLDC motor drivers, RC channel and MODE button inputs.  It is easily possible to go back to original firmware but make sure to back all your settings up.  This firmware attempts to write its settings to the end of the memory block so as to not interfere with the original firmware's storage, but no guarantees.

## STorM32 opensource replacement

The old STorM32 v1.3x and STorM32-NT v1.3x boards are supported.  Here, you do need to back up your STorM32 firmware settings as they will likely be overwritten by OpenBGC.

Later (up to v4.3) and earlier STorM32 boards using the same STM32F103RC MCU are trivial to add although, unlike with SimpleBGC32, the pinouts differ between board versions.  Some of the board features, such as IMUs and motor drivers, are runtime-configurable from the GUI.  But some, like I2C / UART / RC pins, are currently hardcoded in [src/hw.h](./src/hw.h) in the firmware code so a rebuild *will* be needed.

## Motivation

The official firmware from BasecamElectronics is honestly very well made, robust, flexible, configurable.  The GUI is excellent too.  Apparently the team behind it was occasionally even taking requests/suggestions from forums.  However no one can expect them to support every user's custom one-off need, and their forums have been closed for over a year at the time of writing and no other way to contact them.

In my case I need to get just a tiny bit of extra precision from the attitude tracking because my camera is set at ~1400mm focal length, i.e. very high zoom, for sky photography (astrophotography).  The official firmware's tracking precision is not bad, probably close to the hardware's limitations.  But there's a couple of algorithmic ideas that I want to try to see if I can get an improvement.

## Installation and usage

Until better documentation is available refer to these walk-through videos:

* [OpenBGC setup part 1: flashing](https://www.youtube.com/watch?v=vO-vN2ZL8Ac)
* [OpenBGC setup part 2: calibration](https://www.youtube.com/watch?v=L7Ofs9QW8jE)

## Development

If anyone ventures to make improvements to this code, I recommend you communicate your intent early by perhaps opening a GitHub issue for coordination.

## Summary

This code is a basic firmware for SimpleBGC32 camera gimbals.

Before I go any further, this code is in its early stage.  Current status, at the initial commit, is that it reads and processes the sensor values but doesn't drive the motors.  I wanted to start change tracking early.  The code has minimal drivers for all the peripherals and all of the physical electrical connections figured out for the PiltoFly H2 gimbal (likely very similar to BaseCam reference design), it has some mathematical algorithms implemented but doesn't drive the motors.  It also doesn't do Bluetooth.

## Licensing

3-Clause BSD license.

Some code informed, advised or drafted by AI.

## Compatible hardware

TODO

Invensense MPU6050 IMU (main, or main and frame).

STMF3 MCU.

Encoders are required on all axes.  No-encoder setups are not supported due to the math choices made.

Supported encoders are: only AS5600 for onboard encoder, or any model (don't care) when connected to SBGC32_I2C_Drv extension boards.

CAN communication is *not* supported.

Angles other than 90 degrees between joint axes *are* supported, no assumptions are made.

## Software compatibility

Is it possible to go back to the original firmware? *Yes*.

SimpleBGC32 GUI communication is *not* supported.  It's doable but many of the settings will be different due to different maths algorithms used, and many functionalities are likely never being implemented, such as scripting, variables, MAVLink, RC channels, etc.  Currently settings are hardcoded.

## OpenBGC -- SimpleBGC32 opensource firmware

This repository hosts a basic firmware for camera gimbals, specifically for the gimbals that run SimpleBGC32 gimbal firmware created by BasecamElectronics.  There are many models that use that firmware manufactured by others but using either the electronic boards from SimpleBGC32 (the main controller board, the motor driver+encoder extension boards, etc.) or custom boards based on the SimpleBGC32 schematics, probably with proper licensing.

This is the case with the excellent PilotFly H2 gimbal, which this code is currently tested on.  Very robust design and seemingly full firmware compatibility with other models and manufacturers as well as with DIY gimbals using SimpleBGC32 controllers or their clones.  PilotFly H2 uses a tiny, round controller PCB which is not offered by BasecamElectronics but matches their schematics and the gimbal overall connection layout matches the SimpleBGC32 recommended encoder-enabled modular design.

This code is not at a Proof of Concept stage yet.  This README will be updated when that milestone is reached.  Most code here is currently a least-effort minimum to get something working.  Right now there are IMU drivers with an AHRS algorithm, encoder reading, LED control, some autodetect/autocalibrate/self-test and bootloader/flashing management (but no reflashing from GUI yet).  It is easily possible to go back to original firmware but make sure back all your settings up.

## Motivation

The official firmware from BasecamElectronics is honestly very well made, robust, flexible, configurable.  The GUI is excellent too.  Apparently the team behind it was occasionally even taking requests/suggestions from forums.  However no one can expect them to support every user's custom one-off need, and their forums have been closed for ~a year at the time of writing.

In my case I need to get just a tiny more bit of extra precision from the orientation angle tracking because my camera is set at ~1400mm focal length, i.e. very high zoom, for sky photography.  The official firmware's tracking precision is not bad, probably close to the hardware's limitations.  But there's a couple of algorithmic ideas that I wanted to try to see if I can get an improvement.

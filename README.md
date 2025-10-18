## Summary

This code is a basic opensource firmware for SimpleBGC32 camera gimbals.

Before I go any further, this code is in its early stage.  Current status, at the initial commit, is that it reads and processes the sensor values but doesn't drive the motors.  I wanted to start change tracking early.  The code has minimal drivers for all the peripherals and all of the physical electrical connections figured out for the PiltoFly H2 gimbal (likely very similar to BaseCam reference design), it has some mathematical algorithms implemented but doesn't drive the motors.  It also doesn't do Bluetooth.

## Licensing

3-Clause BSD license.

Some code informed, advised or drafted by AI.

## Compatible hardware

TODO

STMF3 MCU.

3 axes.

2x Invensense MPU6050 IMU (main and "frame").

Encoders are required on all axes.  No-encoder setups are not supported due to the math choices made.

Angles other than 90 degrees between joint axes *are* supported, no assumptions are made.

Long version: We can get by relatively easily without encoders if we have two IMUs *and* perpendicular rotation axes.  We can also get by without the frame IMU (main IMU only) if we have encoders on all axes, no need for them to be perpendicular in this case.  But any other setup will require you to get very creative with the math code.

Supported encoders are: only AS5600 for onboard encoder, or any model (don't care) when connected to SBGC32_I2C_Drv extension boards.

CAN communication is *not* supported.

## Software compatibility

Is it possible to go back to the original firmware? *Yes*, it is.

SimpleBGC32 GUI communication is *not* supported.  It's doable but many of the settings will be different due to different maths algorithms used, and many functionalities are likely never being implemented, such as scripting, variables, MAVLink, RC channels, etc.  Currently settings are hardcoded.

The bluetooth remote support would be nice to have and may require us to add some amount of serial API compatibility.

## OpenBGC -- SimpleBGC32 opensource firmware

This repository hosts a basic firmware for camera gimbals, specifically for the gimbals that run SimpleBGC32 gimbal firmware created by BasecamElectronics.  There are many models that use that firmware and are manufactured by others.  They use either the electronic boards from SimpleBGC32 (the main controller board, the motor driver+encoder extension boards, etc.) or custom boards based on the SimpleBGC32 schematics, probably with proper licensing from BasecamElectronics.

This is the case with the excellent PilotFly H2 gimbal, which this code is currently tested on.  Very robust design and seemingly full firmware compatibility with other models and manufacturers as well as with DIY gimbals using SimpleBGC32 controllers or their clones.  PilotFly H2 uses a tiny, round controller PCB which is not offered by BasecamElectronics but matches their schematics and the gimbal overall connection layout matches the SimpleBGC32 recommended encoder-enabled modular design.

This code is not at a Proof of Concept stage yet.  This README will be updated when that milestone is reached.  Most code here is currently a least-effort minimum to get something working.  Right now there are IMU drivers with an AHRS algorithm, encoder reading, LED control, some autodetect/autocalibrate/self-test and bootloader/flashing management (but no reflashing from GUI yet), rough motor driver support.  It is easily possible to go back to original firmware but make sure to back all your settings up.

## Motivation

The official firmware from BasecamElectronics is honestly very well made, robust, flexible, configurable.  The GUI is excellent too.  Apparently the team behind it was occasionally even taking requests/suggestions from forums.  However no one can expect them to support every user's custom one-off need, and their forums have been closed for ~a year at the time of writing and no other way to contact them.

In my case I need to get just a tiny more bit of extra precision from the orientation angle tracking because my camera is set at ~1400mm focal length, i.e. very high zoom, for sky photography (astrophotography).  The official firmware's tracking precision is not bad, probably close to the hardware's limitations.  But there's a couple of algorithmic ideas that I want to try to see if I can get an improvement.

## STorM32

While it is my goal to eventually support some STorM32 gimbals (3-axis, 2 IMUs w/ or w/out encoders or 1 IMU w/ encoders) there's absolutely no compatibility at this time.

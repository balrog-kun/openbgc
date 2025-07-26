all: compile
compile build: .pio/build/simplebgc32_regular/firmware.bin
.pio/build/simplebgc32_regular/firmware.bin: src/*.c src/*.cpp src/*.h
	pio run

# Bootloader detects the baudrate from our first byte sent to it so we decide the baudrate between 1200 and 115200
bl_baudrate = 115200

upload: .pio/build/simplebgc32_regular/firmware.bin /dev/ttyUSB0 stm32ld
	# First place a jumper at FLASH pads on PilotFly H2 board then connect USB cable
	# The 1 at the end is to start the new binary immediately, drop last param to avoid this
	./stm32ld /dev/ttyUSB0 $(bl_baudrate) .pio/build/simplebgc32_regular/firmware.bin 1
	# Show serial port, ^a q to exit
	picocom -b 115200 /dev/ttyUSB0
boot: /dev/ttyUSB0 stm32ld
	# Start the firmware if board is still in bootloader
	./stm32ld /dev/ttyUSB0 $(bl_baudrate) 0 1
	# Show serial port, ^a q to exit
	picocom -b 115200 /dev/ttyUSB0
com: /dev/ttyUSB0
	# Show serial port, ^a q to exit
	picocom -b 115200 /dev/ttyUSB0

# This is mainly for documentation and a bit of a hack
stm32ld: utils/stm32ld.patch
	[ -d stm32ld.git ] || git clone https://github.com/jsnyder/stm32ld.git stm32ld.git
	cd stm32ld.git && (patch --forward -p1 < ../utils/stm32ld.patch; make)
	ln -s stm32ld.git/stm32ld $@

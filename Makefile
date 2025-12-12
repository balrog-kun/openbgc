ifeq ($(V), 1)
	pio_verbose=-v
endif

all: compile
compile build: .pio/build/simplebgc32_regular/firmware.bin
.pio/build/simplebgc32_regular/firmware.bin: src/*.c src/*.cpp src/*.h src/TwoWire.h
	pio run $(pio_verbose)

# Until FlexWire stops doing this or we switch to a different library, work around FlexWire shipping
# its own Wire.h and overriding the TwoWire identifier (https://github.com/felias-fogg/FlexWire/issues/10)
src/TwoWire.h:
	ln -s ~/.platformio/packages/framework-arduinoststm32/libraries/Wire/src/Wire.h $@

# Bootloader detects the baudrate from our first byte sent to it so we decide the baudrate between 1200 and 115200
bl_baudrate = 115200

#port = /dev/rfcomm0 # bluetooth -- works with picocom but not with stm32ld
port = /dev/ttyUSB0

upload: .pio/build/simplebgc32_regular/firmware.bin $(port) stm32ld
	# First place a jumper at FLASH pads on PilotFly H2 board then connect USB cable
	# The 1 at the end is to start the new binary immediately, drop last param to avoid this
	./stm32ld $(port) $(bl_baudrate) .pio/build/simplebgc32_regular/firmware.bin 1
	# Show serial port, ^a q to exit
	picocom -b 115200 $(port)
boot: $(port) stm32ld
	# Start the firmware if board is still in bootloader
	./stm32ld $(port) $(bl_baudrate) 0 1
	# Show serial port, ^a q to exit
	picocom -b 115200 $(port)
com: $(port)
	# Show serial port, ^a q to exit
	picocom -b 115200 $(port)
bl: $(port)
	# Use the official serial-api command to reset firmware to bootloader,
	# command a 5s delay to avoid the bootloader seeing any extra serial input,
	# should be compatible with both the original firmware and openbgc
	#
	# V1 protocol:
	picocom -b 115200 -t $$(bash -c "echo -ne '\x3e\x33\x03\x36\x01\x88\x13\x9c'") -X $(port)
	# V2:
	#picocom -b 115200 -t $$(bash -c "echo -ne '\x24\x33\x03\x36\x01\x88\x13\x62\x1c'") -r -X $(port)

# This is mainly for documentation and a bit of a hack
stm32ld.git:
	git clone https://github.com/jsnyder/stm32ld.git stm32ld.git
stm32ld.git/patched: stm32ld.git utils/stm32ld.patch
	# Could unpatch but meh
	[ -e $@ ] || (cd stm32ld.git && patch --forward -p1 < ../utils/stm32ld.patch) && cp utils/stm32ld.patch $@ && touch $@
stm32ld.git/stm32ld: stm32ld.git/patched
	make -C stm32ld.git && touch $@
stm32ld: stm32ld.git/stm32ld
	ln -sf stm32ld.git/stm32ld $@

tags:
	ctags src/*.{c,h,cpp}
gdb: .pio/build/simplebgc32_regular/firmware.elf
	gdb-multiarch .pio/build/simplebgc32_regular/firmware.elf

utils/param_map.py: utils/build-param-map-py.txt src/param-defs.c.inc src/util.h
	cpp utils/build-param-map-py.txt -o $@
utils/param_defs.py: .pio/build/simplebgc32_regular/firmware.elf utils/param_map.py utils/build-param-defs.py
	gdb-multiarch .pio/build/simplebgc32_regular/firmware.elf -x utils/param_map.py -x utils/build-param-defs.py

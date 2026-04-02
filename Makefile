# The rules in this file are a bit of a mix.  First are the development rules,
# second the rules relevant to making a release and at the end are the user
# rules (installation etc.) to run from inside an unpacked release directory.

ifeq ($(V), 1)
	pio_verbose := -v
endif

BOARDS := simplebgc32_regular storm32_stm32f1

# Select the board we're currently working with.  Override this on the command
# line or here.  Mainly affects upload since we always build for all boards.
ifeq ($(DEV_BOARD),)
	DEV_BOARD := simplebgc32_regular
	#DEV_BOARD := storm32_stm32f1
endif

REL_VER := $(strip $(shell [ -e rel-version ] && cat rel-version))
ARCH := $(shell uname -m)

ifeq ($(REL_VER),)
	FIRMWARE_ELF := .pio/build/$(DEV_BOARD)/firmware.elf
	FIRMWARE_BIN := .pio/build/$(DEV_BOARD)/firmware.bin
	FIRMWARE_BIN_ALL := $(patsubst %, .pio/build/%/firmware.bin, $(BOARDS))
	STM32LD_BIN := stm32ld.git/stm32ld
else
	FIRMWARE_BIN := openbgc-$(REL_VER)-$(DEV_BOARD).bin
	FIRMWARE_BIN_ALL := $(patsubst %, openbgc-$(REL_VER)-%.bin, $(BOARDS))
	STM32LD_BIN := tools/stm32ld-$(ARCH)
endif

all: compile
compile build: $(FIRMWARE_BIN)
ifeq ($(REL_VER),)
$(FIRMWARE_BIN_ALL): src/*.c src/*.cpp src/*.h src/TwoWire.h
	pio run $(pio_verbose)
endif

# Until FlexWire stops doing this or we switch to a different library, work around FlexWire shipping
# its own Wire.h and overriding the TwoWire identifier (https://github.com/felias-fogg/FlexWire/issues/10)
src/TwoWire.h:
	ln -s ~/.platformio/packages/framework-arduinoststm32/libraries/Wire/src/Wire.h $@

# Bootloader detects the baudrate from our first byte sent to it so we decide the baudrate between 1200 and 115200
bl_baudrate = 115200

#port = /dev/rfcomm0 # bluetooth -- works with picocom but not with stm32ld
port = /dev/ttyUSB0

upload: $(FIRMWARE_BIN) $(port) stm32ld
	# PilotFly H2: Place a jumper between FLASH pads on main board then connect USB cable
	# Boards with switches: reset with the FLASH or BOOT0 button pressed
	# The 1 at the end is to start the new binary immediately, drop last param to avoid this
	./stm32ld $(port) $(bl_baudrate) $(FIRMWARE_BIN) 1
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
stm32ld: $(STM32LD_BIN)
	ln -sf $(STM32LD_BIN) $@

tags:
	ctags src/*.{c,h,cpp}
gdb: $(FIRMWARE_ELF)
	gdb-multiarch $(FIRMWARE_ELF)

utils/param_map.py: utils/build-param-map-py.txt src/param-defs.c.inc src/util.h
	cpp utils/build-param-map-py.txt -o $@
utils/param_defs.py: $(FIRMWARE_ELF) utils/param_map.py utils/param_utils.py utils/build-param-defs.py
	gdb-multiarch $(FIRMWARE_ELF) -x utils/param_map.py -x utils/param_utils.py -x utils/build-param-defs.py

utils/sbgcserialapi:
	git clone https://github.com/balrog-kun/sbgcserialapi.git $@
client: utils/param_defs.py utils/sbgcserialapi utils/param-tool.py
	@echo Ready.  Copy utils/99-iio.rules to /etc/udev/rules.d/ to use laptop accelerometers/gyros in the GUI app
	# also needs udevadm control --reload-rules && udevadm trigger -c add

# VER expected to be set on command line
release-tag:
	git tag -a $(VER) -m "Release $(VER)"

# Find last tag only if the target is "release"
ifneq ($(filter release,$(MAKECMDGOALS)),)
    SHELL := /bin/bash
    VER := $(strip $(shell git describe --tags --abbrev=0))
endif
release: openbgc-$(VER).tar.gz

openbgc-%.tar.gz: $(FIRMWARE_BIN_ALL) client stm32ld
	$(eval DIR := openbgc-$*)
	rm -rf $(DIR)
	mkdir $(DIR)
	mkdir $(DIR)/tools
	mkdir $(DIR)/client
	mkdir $(DIR)/client/sbgcserialapi
	echo $* > $(DIR)/rel-version
	for board in $(BOARDS); do \
		cp .pio/build/$$board/firmware.bin $(DIR)/openbgc-$*-$$board.bin; \
	done
	cp Makefile $(DIR)/
	cp $(STM32LD_BIN) $(DIR)/tools/stm32ld-$(ARCH)
	cp utils/param_{defs,map,utils}.py utils/app{,_widgets,_iio,_calib_tabs,_hw_tab}.py $(DIR)/client/
	cp utils/param-tool.py $(DIR)/client/ # TODO: use install?
	cp utils/sbgcserialapi/*.py $(DIR)/client/sbgcserialapi/
	chmod 0755 $(DIR)/client/{app,param-tool}.py
	cp utils/99-iio.rules $(DIR)/tools/
	tar -cvzf $@ $(DIR)

ifneq ($(REL_VER),)
install-deps-debian: # Ubuntu, Debian
	sudo apt-get install python3-construct python3-pyqt6 python3-opengl python3-pyudev
install-deps-redhat: # Fedora, CentOS, CentOS Stream etc.
	sudo dnf install python3-construct python3-pyqt6 python3-pyopengl python3-pyudev
install-udev-file: tools/99-iio.rules
	sudo cp tools/99-iio.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger -c add
run-app: client/*.py client/sbgcserialapi/*.py
	client/app.py
endif

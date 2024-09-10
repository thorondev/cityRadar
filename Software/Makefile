# Attention for editors: You have to use tab-characters for command indentation!
# vim: set noexpandtab:

DEVICE_TTY      ?= /dev/ttyACM1

BAUDRATE        := 115200
PROCESSOR       := teensy40
BOARD           := teensy:avr:$(PROCESSOR):usb=serial2

BUILD_DIR_ARD   := build/

.PHONY: default
default: build

.PHONY: help
help:
	@echo "Help:"
	@echo ""
	@echo "$(MAKE) build         - just build"
	@echo "$(MAKE) deploy        - build and upload to teensy"
	@echo "$(MAKE) deployNoBuild - just upload to teensy"
	@echo "$(MAKE) monitor       - monitor $(DEVICE_TTY)"

.PHONY: build
build:
	arduino-cli compile --jobs 8 --build-path $(BUILD_DIR_ARD) --build-cache-path /tmp/arduino --fqbn $(BOARD) -v .

.PHONY: deploy
deploy: build
	teensy_loader_cli -v --mcu=teensy40 -w build/Software.ino.hex
.PHONY: deployNoBuild
deployNoBuild:
	teensy_loader_cli -v --mcu=teensy40 -w build/Software.ino.hex
#arduino-cli upload -p $(DEVICE_TTY) --fqbn $(BOARD) --input-dir $(BUILD_DIR_ARD) .

.PHONY: monitor
monitor:
# get info via arduino-cli board list
	arduino-cli monitor --fqbn teensy:avr:teensy40 --port usb3/3-4/3-4.3/3-4.3.3 --protocol teensy
#arduino-cli monitor -p $(DEVICE_TTY) -c "baudrate=$(BAUDRATE)"

.PHONY: testSetup
testSetup:
	@echo -n "Testing for arduino-cli: " && (which arduino-cli) || (echo "FAILED - Please install arduino-cli" && exit 1)
	@echo "Testing for BOARD platform " && (arduino-cli core list | grep -q "esp32") || (echo "FAILED - Please run arduino-cli core install esp32:esp32" && exit 1)
	@echo "Testing for DebugLog " && (arduino-cli lib list | grep -q "DebugLog") || (echo "FAILED - Please run arduino-cli lib install DEBUGLOG" && exit 1)
	@echo "Setup seems to be ok."


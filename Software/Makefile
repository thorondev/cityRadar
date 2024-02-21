# Attention for editors: You have to use tab-characters for command indentation!
# vim: set noexpandtab:

DEVICE_IP       ?= 192.168.1.121
DEVICE_TTY      ?= /dev/ttyHotWireEsp32

BAUDRATE        := 115200
BOARD           := teensy:avr:teensy41

BUILD_DIR_ARD   := build/

.PHONY: default
default: build

.PHONY: help
help:
	@echo "Help:"
	@echo ""
	@echo "$(MAKE) build        - just build"
	@echo "$(MAKE) deployToIp   - build and upload to $(DEVICE_IP)"
	@echo "$(MAKE) deployToTty  - build and upload to $(DEVICE_TTY)"
	@echo "$(MAKE) monitor      - monitor $(DEVICE_TTY)"

.PHONY: build
build:
	arduino-cli compile --build-path $(BUILD_DIR_ARD) --build-cache-path /tmp/arduino --fqbn $(BOARD) -v .

.PHONY: deployToIp
deployToIp: build
	arduino-cli upload -p $(DEVICE_IP) --fqbn $(BOARD) --input-dir $(BUILD_DIR_ARD) .

.PHONY: deployToTty
deployToTty: build
	arduino-cli upload -p $(DEVICE_TTY) --fqbn $(BOARD) --input-dir $(BUILD_DIR_ARD) .

.PHONY: monitor
monitor:
	arduino-cli monitor help -p $(DEVICE_TTY) -c "baudrate=$(BAUDRATE)"

.PHONY: testSetup
testSetup:
	@echo -n "Testing for arduino-cli: " && (which arduino-cli) || (echo "FAILED - Please install arduino-cli" && exit 1)
	@echo "Testing for BOARD platform " && (arduino-cli core list | grep -q "esp32") || (echo "FAILED - Please run arduino-cli core install esp32:esp32" && exit 1)
	@echo "Testing for DebugLog " && (arduino-cli lib list | grep -q "DebugLog") || (echo "FAILED - Please run arduino-cli lib install DEBUGLOG" && exit 1)
	@echo "Setup seems to be ok."


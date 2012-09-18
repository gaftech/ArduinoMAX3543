# Makefile, using Arduino-Makefile (https://github.com/gaftech/Arduino-Makefile)

# Hardware
BOARD_TAG			= pro328
ARDUINO_PORT		= /dev/ttyUSB0

# Needed libs
ARDUINO_LIBS		= Wire Wire/utility ArduinoMAX3543 ArduinoTestHelpers ArduinoTestSuite

# libs locations
ARDUINO_DIR			= /usr/share/arduino
ARDMK_DIR     		= /home/gabriel/dev/arduino/Arduino-Makefile
AVR_TOOLS_DIR	 	= /usr
ARDUINO_SKETCHBOOK	= /home/gabriel/dev/arduino

# Example flags you can set (see header file for more):
#CPPFLAGS += -DMAX3543_MAXIM_MODE
#            -DMAX3543_IFSEL=MAX3543_IFSEL_ATV \
#			 -DMAX3543_BW=MAX3543_BW_8MHz
include $(ARDMK_DIR)/arduino-mk/Arduino.mk


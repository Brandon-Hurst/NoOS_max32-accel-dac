# Select the example you want to enable by choosing y for enabling and n for disabling
BASIC_EXAMPLE = y

PLATFORM ?= maxim
TARGET ?= max32670

# Select the device you want to enable by choosing y for enabling and n for disabling
# ADXL380 = y
# ADXL382 = n
ADXL355 ?= y
AD5421 ?= y

CFLAGS+= -ggdb -O0

include ../../tools/scripts/generic_variables.mk

include src.mk

include ../../tools/scripts/generic.mk

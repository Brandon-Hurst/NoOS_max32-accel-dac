# Copyright (c) 2024 Brandon Hurst, Analog Devices, Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

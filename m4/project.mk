###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################
# This file can be used to set build configuration
# variables.  These variables are defined in a file called
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Add compiler flags to enable source navigation in ELF File Explorer
PROJ_CFLAGS += -fdump-rtl-expand
PROJ_CFLAGS += -fdump-rtl-dfinish
PROJ_CFLAGS += -fdump-ipa-cgraph
PROJ_CFLAGS += -fstack-usage
PROJ_CFLAGS += -gdwarf-4
PROJ_CFLAGS += -Wno-unused

MXC_OPTIMIZE_CFLAGS = -Og

### Additions to support Building with No-OS ###
LIB_BOARD=0
NO_OS_ROOT = ./no-os

include ./no_os_msdk.mk

# Add new drivers (custom)
VPATH += ./drivers
IPATH += ./drivers

# #Add just the No-Os drivers we want to use
VPATH += ./ad5421
IPATH += ./ad5421
VPATH += $(NO_OS_ROOT)/drivers/accel/adxl38x
IPATH += $(NO_OS_ROOT)/drivers/accel/adxl38x
#################################################

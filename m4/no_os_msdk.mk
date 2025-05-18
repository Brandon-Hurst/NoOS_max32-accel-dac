#Leave it up to the User to point to the No-Os Location in project.mk
NO_OS_ROOT ?= ./no-os
# ifndef NO_OS_ROOT
# $(error NO_OS_ROOT not defined)
# endif

#Require Autosearch. This prevents the makefile from needing to know all of the
#No-Os files
ifeq ($(AUTOSEARCH), 0)
$(error AUTOSEARCH must be 1 for No-Os file inclusion)
endif

#No-Os takes over various ISR Handlers, etc. Can't use a BSP with it
ifneq ($(LIB_BOARD), 0)
$(error Using a board/BSP not compatible with No-Os. Set LIB_BOARD=0)
endif

#Create a variable for the platform directory
NO_OS_PLAT_DIR = $(NO_OS_ROOT)/drivers/platform/maxim/$(TARGET_LC)
NO_OS_PLAT_DIR += $(NO_OS_ROOT)/drivers/platform/maxim/common/

#Check the platform directory actually exists
ifeq ("$(wildcard $(NO_OS_PLAT_DIR))", "")
$(error Platform directory does not exists. Check NO_OS_ROOT and Target are valid)
endif

#Add the API Source
VPATH += $(NO_OS_ROOT)/drivers/api

#Add the target specific source
VPATH += $(NO_OS_PLAT_DIR)

#Add the Utils
VPATH += $(NO_OS_ROOT)/util

#Add the necessary includes
IPATH += $(NO_OS_ROOT)/include
IPATH += $(NO_OS_PLAT_DIR)

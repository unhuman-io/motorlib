SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og -fsingle-precision-constant -O3 
LTO = -flto=auto

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable. 

# Install GCC automatically if GCC_PATH is not specified and it's not already installed.

ifndef GCC_PATH
upgrade_gcc:
	@echo upgrading gcc
	$(SELF_DIR)../scripts/install_gcc.sh

.PHONY: upgrade_gcc

$(SELF_DIR)../gcc/bin/$(PREFIX)gcc:
	$(SELF_DIR)../scripts/install_gcc.sh
endif

GCC_PATH=$(SELF_DIR)../gcc/bin
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
CXX = $(GCC_PATH)/$(PREFIX)g++
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(LTO)

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(LTO)

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CPPFLAGS = $(CFLAGS)

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections,--print-memory-usage -u _printf_float

GCC_VERSION := $(shell $(CC) -dumpversion)
GCC_MAJOR_VERSION := $(word 1, $(subst ., ,$(GCC_VERSION)))

ifeq ($(GCC_MAJOR_VERSION), $(filter $(GCC_MAJOR_VERSION),10 11 12 13))
$(call info_once,gcc version $(GCC_VERSION))
else
$(error gcc version $(GCC_VERSION), 10 - 13 required)
endif

ifeq ($(OS),Windows_NT)
$(call info_once,build is on windows)
RM=powershell -Command rm -r -fo
MKDIR=powershell -Command mkdir -fo
else
RM=rm -rf
MKDIR=mkdir -p
$(call info_once,build is not on windows)
endif
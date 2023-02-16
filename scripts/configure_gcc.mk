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
GCC_PATH=$(dir $(lastword $(MAKEFILE_LIST)))/../gcc/bin
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
CXX = $(GCC_PATH)/$(PREFIX)g++
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
CXX = $(PREFIX)g++
endif
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
LDFLAGS = $(MCU) -specs=rdimon.specs -specs=nosys.specs -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -u _printf_float
	
GCC_VERSION := $(shell $(CC) -dumpversion)
GCC_MAJOR_VERSION := $(word 1, $(subst ., ,$(GCC_VERSION)))

ifeq ($(GCC_MAJOR_VERSION), $(filter $(GCC_MAJOR_VERSION),10 11 12))
$(info gcc version $(GCC_VERSION))
else
$(error gcc version $(GCC_VERSION), 10, 11, or 12 required)
endif

ifeq ($(OS),Windows_NT)
$(info windows)
RM=powershell -Command rm -r -fo
MKDIR=powershell -Command mkdir -fo
else
RM=rm -rf
MKDIR=mkdir -p
$(info not windows)
endif
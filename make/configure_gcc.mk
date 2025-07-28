SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og -cl-single-precision-constant -O3 
LTO = -flto=auto


CC_PATH=$(SELF_DIR)../llvm/bin
CC = $(CC_PATH)/clang
AS = $(CC_PATH)/clang -x assembler-with-cpp
CP = $(CC_PATH)/llvm-objcopy
SZ = $(CC_PATH)/llvm-size
CXX = $(CC_PATH)/clang++
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# fpu
FPU = -mfpu=fpv4-sp-d16

# mcu
MCU = --target=armv7em-none-eabihf $(FPU) -mcpu=cortex-m4

# compile cc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(LTO)

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(LTO) -Wno-vla-cxx-extension -Wno-c99-designator -Wno-c23-extensions

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

CC_VERSION := $(shell $(CC) -dumpversion)
CC_MAJOR_VERSION := $(word 1, $(subst ., ,$(CC_VERSION)))

ifeq ($(CC_MAJOR_VERSION), $(filter $(CC_MAJOR_VERSION),20))
$(call info_once,clang version $(CC_VERSION))
else
$(error clang version $(CC_VERSION), 20 required)
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
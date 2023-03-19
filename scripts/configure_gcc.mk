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
GCC_PATH := $(dir $(lastword $(MAKEFILE_LIST)))../gcc/bin
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

ifdef CLANG
CC = clang
AS = clang -x assembler-with-cpp
CXX = clang++
LD = ld.lld
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

ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(LTO)
# compile gcc flags
ifdef CLANG
SYSROOT := $(GCC_PATH)/../arm-none-eabi
CFLAGS = -target thumbv7em-none-eabi --sysroot $(SYSROOT) $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(LTO)
CFLAGS += -Wno-deprecated-register
else
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(LTO)
endif # CLANG

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CPPFLAGS = $(CFLAGS) 
ifdef CLANG

CPPFLAGS += -I$(SYSROOT)/include/c++/12.2.1/arm-none-eabi/thumb/v7e-m+fp/hard/ -I$(SYSROOT)/include/c++/12.2.1/
endif

# libraries
ifndef CLANG
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=rdimon.specs -specs=nosys.specs -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -u _printf_float
else
LIBS = -lc_nano -lm -lnosys -lrdimon_nano -lstdc++_nano -lg_nano -lgcc
LIBDIR = 
LDFLAGS := --Bstatic --sysroot /opt/gcc-arm-none-eabi/arm-none-eabi/
LDFLAGS += --build-id
LDFLAGS += --gc-sections
LDFLAGS += --Map $(BUILD_DIR)/$(TARGET).map
LDFLAGS += --script $(LDSCRIPT) --build-id=none --gc-sections
LDFLAGS += -Llib -L$(SYSROOT)/lib/thumb/v7e-m+fp/hard/ -L$(SYSROOT)/../lib/gcc/arm-none-eabi/12.2.1/thumb/v7e-m+fp/hard/ $(LIBS)
endif

ifndef CLANG	
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
endif #CLANG
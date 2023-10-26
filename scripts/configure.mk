#######################################
# paths
#######################################
# Requires PARAM_FILE, CONFIG_FILE, and TARGET_MCU defined

# Build path
BUILD_DIR = build
GIT_VERSION := $(shell git describe --long --dirty --always --abbrev=7)
GIT_HASH := $(shell git rev-parse HEAD)

SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

ifndef TARGET_MCU
$(error need to define TARGET_MCU)
endif

$(info target is $(TARGET_MCU))

ifeq "$(TARGET_MCU)" "stm32g474"

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/stm32g4xx_it.c \
Src/system_stm32g4xx.c \
# $(DRIVERS)/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
# $(DRIVERS)/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
# $(DRIVERS)/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
# $(DRIVERS)/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
# $(DRIVERS)/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
# $(DRIVERS)/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \



# ASM sources
ASM_SOURCES =  \
$(SELF_DIR)../peripheral/stm32g4/startup_stm32g474xx.s


# macros for gcc
# AS defines
AS_DEFS = 

# C defines
override C_DEFS +=  \
-DGIT_VERSION=\"$(GIT_VERSION)\" \
-DSTM32G474xx \
-DGIT_HASH=\"$(GIT_HASH)\"
ifdef NOTES
override C_DEFS += \
-DNOTES=\"-$(NOTES)\"
else
override C_DEFS += \
-DNOTES=\"$(shell git branch --show-current)\"
endif
#-DUSE_HAL_DRIVER \

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-I../motorlib/device/stm32g4/Include \
-I$(DRIVERS)/CMSIS/Include

# -IDrivers/CMSIS/Include \
# -I$(DRIVERS)/STM32G4xx_HAL_Driver/Inc \
# -I$(DRIVERS)/STM32G4xx_HAL_Driver/Inc/Legacy \
#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(SELF_DIR)../peripheral/stm32g4/STM32G474RETx_FLASH.ld

ifndef DRIVERS
DRIVERS = ../motorlib/drivers
endif

endif # MCU_TARGET

CPP_SOURCES = \
$(SELF_DIR)../control_fun.cpp\
foc.cpp\
gpio.cpp\
sincos.cpp\
util.cpp\
hall.cpp\
$(SELF_DIR)../peripheral/spi_encoder.cpp\
$(SELF_DIR)../peripheral/stm32g4/usb.cpp\
hrpwm.cpp\
ams_encoder.cpp\
parameter_api.cpp\
./$(CONFIG_FILE)

C_SOURCES += ./$(PARAM_FILE)

ifdef PARAM_OVERRIDE
PARAM_INCLUDE=-include $(PARAM_OVERRIDE)
endif


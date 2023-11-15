#######################################
# paths
#######################################
# Requires PARAM_FILE, CONFIG_FILE, and TARGET_MCU defined

# Build path
BUILD_DIR = build
GIT_VERSION := $(shell git describe --long --dirty --always --abbrev=7)
GIT_HASH := $(shell git rev-parse HEAD)
MOTORLIB_HASH := $(shell git -C $(SELF_DIR) rev-parse HEAD)

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
./$(PARAM_FILE)


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
-DGIT_HASH=\"$(GIT_HASH)\" \
-DMOTORLIB_HASH=\"$(MOTORLIB_HASH)\"
ifdef NOTES
override C_DEFS += \
-DNOTES=\"-$(NOTES)\"
else
override C_DEFS += \
-DNOTES=\"$(shell git branch --show-current)\"
endif

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-I$(SELF_DIR)../device/stm32g4/Include \
-I$(SELF_DIR)../CMSIS/Include \
-I$(SELF_DIR)../peripheral \
-I$(SELF_DIR)../peripheral/stm32g4

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(SELF_DIR)../peripheral/stm32g4/STM32G474RETx_FLASH.ld

endif # MCU_TARGET

CPP_SOURCES = \
$(SELF_DIR)../control_fun.cpp\
$(SELF_DIR)../foc.cpp\
$(SELF_DIR)../gpio.cpp\
$(SELF_DIR)../sincos.cpp\
$(SELF_DIR)../util.cpp\
$(SELF_DIR)../hall.cpp\
$(SELF_DIR)../peripheral/spi_encoder.cpp\
$(SELF_DIR)../peripheral/spi_protocol.cpp\
$(SELF_DIR)../peripheral/spi_mailbox.cpp\
$(SELF_DIR)../peripheral/spi_protocol_states.cpp\
$(SELF_DIR)../peripheral/spi_protocol_commands.cpp\
$(SELF_DIR)../peripheral/stm32g4/usb.cpp\
$(SELF_DIR)../peripheral/stm32g4/spi_slave.cpp\
$(SELF_DIR)../hrpwm.cpp\
$(SELF_DIR)../ams_encoder.cpp\
$(SELF_DIR)../parameter_api.cpp\
./$(CONFIG_FILE)


ifdef PARAM_OVERRIDE
PARAM_INCLUDE=-include $(PARAM_OVERRIDE)
endif


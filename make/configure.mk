#######################################
# paths
#######################################
# Requires PARAM_FILE, CONFIG_FILE, and TARGET_MCU defined

# Build path
BUILD_DIR = build
ifndef GIT_VERSION
GIT_VERSION := $(shell git describe --long --dirty --always --abbrev=7)
endif
ifndef GIT_HASH
GIT_HASH := $(shell git rev-parse HEAD)
endif
MOTORLIB_HASH := $(shell git -C $(SELF_DIR) rev-parse HEAD)
$(shell touch $(SELF_DIR)../param_default.h)

ifndef TARGET_MCU
$(error need to define TARGET_MCU)
endif

$(info target is $(TARGET_MCU))

ifeq "$(TARGET_MCU)" "stm32g474"

######################################
# source
######################################
# C sources
C_SOURCES +=  \
Src/stm32g4xx_it.c \
Src/system_stm32g4xx.c

# ASM sources
ASM_SOURCES += \
$(SELF_DIR)../peripheral/stm32g4/startup_stm32g474xx.s

# macros for gcc
# AS defines
AS_DEFS =

# C defines
override C_DEFS +=  \
-DSTM32G474xx

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES +=  \
-I$(SELF_DIR)../device/stm32g4/Include \
-I$(SELF_DIR)../peripheral/stm32g4

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(SELF_DIR)../peripheral/stm32g4/STM32G474RETx_FLASH.ld

CPP_SOURCES += \
$(SELF_DIR)../peripheral/stm32g4/usb.cpp\
$(SELF_DIR)../peripheral/stm32g4/spi_slave.cpp\
$(SELF_DIR)../peripheral/spi_encoder.cpp\
$(SELF_DIR)../peripheral/spi_protocol.cpp\
$(SELF_DIR)../peripheral/spi_mailbox.cpp\
$(SELF_DIR)../peripheral/spi_protocol_states.cpp\
$(SELF_DIR)../peripheral/spi_protocol_commands.cpp\
$(SELF_DIR)../peripheral/stm32g4/ams_encoder.cpp\
$(SELF_DIR)../peripheral/stm32g4/hrpwm.cpp

endif # MCU_TARGET == stm32g474

override C_DEFS +=  \
-DGIT_VERSION=\"$(GIT_VERSION)\" \
-DGIT_HASH=\"$(GIT_HASH)\" \
-DMOTORLIB_HASH=\"$(MOTORLIB_HASH)\" \
-DCONFIG=\"$(CONFIG)\"
ifdef NOTES
override C_DEFS += \
-DNOTES=\"-$(NOTES)\"
else
override C_DEFS += \
-DNOTES=\"$(shell git branch --show-current)\"
endif

C_INCLUDES +=  \
-IInc \
-I$(SELF_DIR)../CMSIS/Include \
-I$(SELF_DIR)../peripheral

C_SOURCES +=  \
Src/main.c \
./$(PARAM_FILE)

CPP_SOURCES += \
$(SELF_DIR)../control_fun.cpp\
$(SELF_DIR)../foc.cpp\
$(SELF_DIR)../gpio.cpp\
$(SELF_DIR)../sincos.cpp\
$(SELF_DIR)../util.cpp\
$(SELF_DIR)../hall.cpp\
$(SELF_DIR)../parameter_api.cpp\
./$(CONFIG_FILE)


ifdef PARAM_OVERRIDE
PARAM_INCLUDE=-include $(PARAM_OVERRIDE)
endif

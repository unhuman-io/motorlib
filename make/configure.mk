#######################################
# paths
#######################################
# Requires CONFIG_FILE, and TARGET_MCU defined

# Build path
BUILD_DIR = build
ifndef OBOT_VERSION
OBOT_VERSION := $(shell git describe --long --dirty --always --abbrev=7)
endif
ifndef OBOT_HASH
OBOT_HASH := $(shell git rev-parse HEAD)
endif
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
ifndef C_SOURCES
C_SOURCES =  \
$(SELF_DIR)../device/stm32g4/Src/stm32g4xx_it.c \
$(SELF_DIR)../device/stm32g4/Src/system_stm32g4xx.c \
$(SELF_DIR)../device/stm32g4/Src/main.c
endif

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
$(SELF_DIR)../peripheral/spi_encoder.cpp\
$(SELF_DIR)../peripheral/protocol.cpp\
$(SELF_DIR)../peripheral/mailbox.cpp\
$(SELF_DIR)../peripheral/protocol_states_spi.cpp\
$(SELF_DIR)../peripheral/protocol_states_uart.cpp\
$(SELF_DIR)../peripheral/protocol_commands.cpp\
$(SELF_DIR)../peripheral/stm32g4/ams_encoder.cpp\
$(SELF_DIR)../peripheral/stm32g4/hrpwm.cpp\
$(SELF_DIR)../peripheral/stm32g4/clock_config.cpp\
$(SELF_DIR)../peripheral/stm32g4/stm32g4_serial.cpp\
$(SELF_DIR)../peripheral/stm32g4/usb.cpp\
$(SELF_DIR)../peripheral/stm32g4/spi_slave.cpp\
$(SELF_DIR)../peripheral/stm32g4/uart.cpp

endif # MCU_TARGET == stm32g474

override C_DEFS +=  \
-DOBOT_VERSION=\"$(OBOT_VERSION)\" \
-DOBOT_HASH=\"$(OBOT_HASH)\" \
-DMOTORLIB_HASH=\"$(MOTORLIB_HASH)\" \
-DCONFIG=\"$(CONFIG)\"
ifdef NOTES
override C_DEFS += \
-DNOTES=\"-$(NOTES)\"
else
override C_DEFS += \
-DNOTES=\"$(shell git branch --show-current)\"
endif

# Uncomment to disable the watchdog timer
#override C_DEFS += \
-DNO_WATCHDOG

C_INCLUDES +=  \
-I. \
-I$(SELF_DIR)../CMSIS/Include \
-I$(SELF_DIR)../peripheral \
-I$(SELF_DIR)../../lib/protocol \
-I$(SELF_DIR)../obot-protocol \

CPP_SOURCES += \
$(SELF_DIR)../malloc.cpp\
$(SELF_DIR)../control_fun.cpp\
$(SELF_DIR)../foc.cpp\
$(SELF_DIR)../gpio.cpp\
$(SELF_DIR)../sincos.cpp\
$(SELF_DIR)../util.cpp\
$(SELF_DIR)../hall.cpp\
$(SELF_DIR)../parameter_api.cpp\
$(SELF_DIR)../obot-protocol/protocol_parser.cpp\
./$(CONFIG_FILE)

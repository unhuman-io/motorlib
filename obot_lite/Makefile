
TARGET = obot_lite
BUILD_DIR = build
CC = arm-none-eabi-gcc
AS = $(CC) -x assembler-with-cpp
SZ = arm-none-eabi-size
.EXTRA_PREREQS += $(filter-out %.d, $(MAKEFILE_LIST))


all: $(BUILD_DIR)/$(TARGET).elf

SRCS = main.o startup_stm32g474xx.o
OBJS = $(addprefix $(BUILD_DIR)/, $(SRCS))


CFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32G474xx
LDFLAGS += -TSTM32G474RETx_FLASH.ld -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,--gc-sections  -u _printf_float

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) | $(BUILD_DIR)
	@echo "  LD    " $<
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "  CC    " $<
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "  AS    " $<
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR):
	mkdir -p $@

.PHONY: clean

clean:
	rm -rf $(BUILD_DIR)

$(VERBOSE).SILENT:

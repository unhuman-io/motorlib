BUILD_DIR = build
TARGET = stm32g4_bootloader

TARGET_MCU := stm32g474

all:: $(BUILD_DIR)/$(TARGET).elf

include ../../make/configure_gcc.mk

LDSCRIPT = STM32G474RETx_FLASH.ld

OBJECTS = build/bootloader_main.o \
build/bootloader_stm32g474xx.o \


$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	@echo "  CXX    " $<
	$(CXX) -c $(CPPFLAGS) -std=c++17 -Wa,-a,-ad,-alms=$(BUILD_DIR)/lto.lst $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "  AS     " $<
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | $(BUILD_DIR)
	@echo "  LD     " $@
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR):
	$(MKDIR) $@

$(VERBOSE).SILENT:

clean:
	-$(RM) $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

BUILD_DIR = build
TARGET = stm32g4_bootloader

TARGET_MCU := stm32g474

all:: $(BUILD_DIR)/$(TARGET).elf

include ../../make/configure_gcc.mk

LDSCRIPT = STM32G474RETx_FLASH.ld

SOURCES = bootloader_main.cpp \
bootloader_stm32g474xx.s \
interrupt_vectors_g4xx.s \
flash.cpp \
can.cpp \

OBJECTS = $(addprefix $(BUILD_DIR)/, \
	$(addsuffix .o, $(basename $(SOURCES))))

VPATH = $(SELF_DIR)../peripheral/stm32g4

override C_DEFS +=  \
-DSTM32G474xx

C_INCLUDES +=  \
-I$(SELF_DIR)../device/stm32g4/Include \
-I$(SELF_DIR)../CMSIS/Include \
-I$(SELF_DIR)../peripheral \

# ensure flash commands stay in allocated section
$(BUILD_DIR)/flash.o: $(SELF_DIR)../peripheral/stm32g4/flash.cpp | $(BUILD_DIR)
	@echo "  CXX NO LTO" $<
	$(CXX) -c $(filter-out $(LTO), $(CPPFLAGS)) -std=c++17 -Wa,-a,-ad,-alms=$(BUILD_DIR)/lto.lst $< -o $@


$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	@echo "  CXX    " $<
	$(CXX) -c $(CPPFLAGS) -std=c++17 -Wa,-a,-ad,-alms=$(BUILD_DIR)/lto.lst $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "  AS     " $<
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) | $(BUILD_DIR)
	@echo "  LD     " $@
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR):
	$(MKDIR) $@

$(VERBOSE).SILENT:

clean:
	-$(RM) $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

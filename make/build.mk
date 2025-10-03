
.DEFAULT_GOAL = clean_build

clean_build::
	$(MAKE) --no-print-directory clean
	$(MAKE) --no-print-directory all

ifneq "$(OS)" "Windows_NT"
BUILD_TGZ = $(BUILD_DIR)/$(TARGET).tgz
endif

all:: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin $(BUILD_TGZ)


FORCE:

.PHONY: FORCE clean all clean_build scan_build

# always build the config since C_DEFS may be different
$(BUILD_DIR)/$(notdir $(CONFIG_FILE:cpp=o)): FORCE

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR) 
	@echo "  CC    " $<
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	@echo "  CXX    " $<
	$(CXX) -c $(CPPFLAGS) $< -o $@

# ensure flash commands stay in allocated section
$(BUILD_DIR)/flash.o: flash.cpp | $(BUILD_DIR)
	@echo "  CXX NO LTO" $<
	$(CXX) -c $(filter-out $(LTO), $(CPPFLAGS)) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "  AS     " $<
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) build_param calibration
	@echo "  LD     " $@
	$(CXX) $(OBJECTS) $(PARAM_OUT:bin=o) $(CALIBRATION_OUT:bin=o) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "  HEX    " $@
	$(HEX) $< $@
	
	
$(BUILD_DIR):
	$(MKDIR) $@

scan_build:
	scan-build --use-cc $(CC) --use-c++ $(CXX) --use-analyzer $(SELF_DIR)../llvm/bin/clang --analyzer-target=armv7m-none-eabi $(MAKE)

$(VERBOSE).SILENT:

#######################################
# clean up
#######################################
clean:
	-$(RM) $(BUILD_DIR)

#######################################
# dependencies
#######################################
# note this seems like it will be behind one build
-include $(wildcard $(BUILD_DIR)/*.d)


.DEFAULT_GOAL = clean_build

clean_build::
	$(MAKE) clean
	$(MAKE) all

ifneq "$(OS)" "Windows_NT"
BUILD_TGZ = $(BUILD_DIR)/$(TARGET).tgz
endif

all:: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin $(BUILD_TGZ)


FORCE:

.PHONY: FORCE clean all clean_build

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
	@ echo "  CC    " $<
	@ $(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/lto.lst $< -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	@ echo "  CXX    " $<
	@ $(CXX) -c $(CPPFLAGS) -std=c++17 -Wa,-a,-ad,-alms=$(BUILD_DIR)/lto.lst $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@ echo "  AS    " $<
	@ $(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) build_param calibration
	$(CXX) $(OBJECTS) $(PARAM_OUT:bin=o) $(CALIBRATION_OUT:bin=o) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@ $(HEX) $< $@
	
	
$(BUILD_DIR):
	$(MKDIR) $@


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

$(shell > tuneable.txt) 
PLUGIN_DIR = $(SELF_DIR)../gcc_plugin/build
# default action: build all
all::  
	$(MAKE) clean 
	$(MAKE) build_all 

build_all: $(BUILD_DIR)/tuneable.cpp $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin $(BUILD_TGZ)

SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/tuneable.cpp: $(OBJECTS) 
	sed -nE 's/(\S*)\s(\S+)\s(\S+)/api.add_variable("\3", new \1 API\2(\&config::\3));/p' tuneable.txt > tuneable.cpp 

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) $(PARAM_INCLUDE) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -fplugin=$(PLUGIN_DIR)/tuneable-plugin.so -include $(SELF_DIR)../system_log.h -c $(CPPFLAGS) -std=c++11 -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@ >> tuneable.txt

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
include $(SELF_DIR)firmware_installer.mk
	
$(BUILD_DIR):
	$(MKDIR) $@

ifneq "$(OS)" "Windows_NT"
BUILD_TGZ = $(BUILD_DIR)/$(TARGET).tgz
endif

#######################################
# clean up
#######################################
clean:
	-$(RM) $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

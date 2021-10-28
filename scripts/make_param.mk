ifdef PARAM_OVERRIDE
PARAM_INCLUDE=-include $(PARAM_OVERRIDE)
endif

PARAM_BUILD_DIR = $(BUILD_DIR)/$(dir $(PARAM_FILE))

all:: param
param: $(BUILD_DIR)/$(PARAM_FILE:c=bin)

$(BUILD_DIR)/$(PARAM_FILE:c=bin): $(PARAM_FILE) $(PARAM_OVERRIDE) | $(BUILD_DIR) $(PARAM_BUILD_DIR)
	$(CC) $(PARAM_INCLUDE) -c $< -o $(BUILD_DIR)/$(<:c=o)
	$(CP) -O binary -S -j flash_param $(BUILD_DIR)/$(<:c=o) $@ 

$(PARAM_BUILD_DIR):
	$(MKDIR) -p $@
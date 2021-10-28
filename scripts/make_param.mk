ifdef PARAM_OVERRIDE
PARAM_INCLUDE=-include $(PARAM_OVERRIDE)
PARAM_SUFFIX=_$(notdir $(PARAM_OVERRIDE:.h=))
endif

PARAM_BUILD_DIR = $(BUILD_DIR)/$(dir $(PARAM_FILE))
PARAM_OUT = $(BUILD_DIR)/$(PARAM_FILE:.c=)$(PARAM_SUFFIX).bin

all:: param

.PHONY: param

param:
	-$(RM) $(PARAM_BUILD_DIR)
	$(MAKE) build_param

build_param: $(PARAM_OUT)

$(PARAM_OUT): $(PARAM_FILE) $(PARAM_OVERRIDE) | $(BUILD_DIR) $(PARAM_BUILD_DIR)
	$(CC) $(PARAM_INCLUDE) -c $< -o $(BUILD_DIR)/$(<:c=o)
	$(CP) -O binary -S -j flash_param $(BUILD_DIR)/$(<:c=o) $@ 

$(PARAM_BUILD_DIR):
	$(MKDIR) -p $@
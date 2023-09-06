ifdef PARAM_OVERRIDE
PARAM_SUFFIX=$(addprefix _,$(notdir $(PARAM_OVERRIDE:.h=)))
PARAM_OUT = $(foreach suf,$(PARAM_SUFFIX),$(PARAM_BUILD_DIR)/$(notdir $(PARAM_FILE:.c=))$(suf).bin)
else
PARAM_OUT = $(PARAM_BUILD_DIR)/$(notdir $(PARAM_FILE:c=bin))
endif

ifndef PARAM_BUILD_DIR
PARAM_BUILD_DIR = $(BUILD_DIR)/$(dir $(PARAM_FILE))
endif

$(info PARAM_BUILD_DIR: $(PARAM_BUILD_DIR))
$(info $(PARAM_OUT))

all:: param

.PHONY: param

param:
	-$(RM) $(PARAM_BUILD_DIR)
	$(MAKE) build_param

build_param: $(PARAM_OUT)

ifndef PARAM_OVERRIDE
$(PARAM_OUT): $(PARAM_FILE) | $(PARAM_BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $(PARAM_BUILD_DIR)/$(notdir $(<:c=o)) 
	$(CP) -O binary -S -j flash_param $(PARAM_BUILD_DIR)/$(notdir $(<:c=o)) $@ 
endif

# if overrides 
define generateRules
a = $(PARAM_BUILD_DIR)/$(notdir $(PARAM_FILE:.c=))_$(1:.h=)
$(a).bin: $(PARAM_FILE) $(1) | $(PARAM_BUILD_DIR)
	$(CC) -include $(1) -c $(CFLAGS) $(PARAM_FILE) -o $(a).o 
	$(CP) -O binary -S -j flash_param $(a).o $(a).bin
endef

$(foreach ovr, $(PARAM_OVERRIDE), $(eval $(call generateRules, $(ovr))))

$(PARAM_BUILD_DIR):
	$(MKDIR) $@
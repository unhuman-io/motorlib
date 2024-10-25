ifndef OBOT_HASH
OBOT_HASH := $(shell git rev-parse HEAD)
endif
GIT_DEFINE := -DOBOT_HASH=\"$(OBOT_HASH)\"

ifdef PARAM_OVERRIDE
PARAM_SUFFIX=$(addprefix _,$(notdir $(PARAM_OVERRIDE:.h=)))
PARAM_OUT = $(foreach suf,$(PARAM_SUFFIX),$(PARAM_BUILD_DIR)$(notdir $(PARAM_FILE:.c=))$(suf).bin)
else
PARAM_OUT = $(PARAM_BUILD_DIR)$(notdir $(PARAM_FILE:c=bin))
endif

# default cal
CALIBRATION_FILE = $(SELF_DIR)../calibration_example.c
CALIBRATION_OUT = $(PARAM_BUILD_DIR)$(notdir $(CALIBRATION_FILE:c=bin))

ifndef PARAM_BUILD_DIR
PARAM_BUILD_DIR = $(BUILD_DIR)/$(dir $(PARAM_FILE))
endif

$(call info_once,PARAM_BUILD_DIR: $(PARAM_BUILD_DIR))
$(call info_once,$(PARAM_OUT))

.PHONY: param .FORCE

param:
	-$(RM) $(PARAM_BUILD_DIR)
	$(MAKE) --no-print-directory build_param

build_param: $(PARAM_OUT)

calibration: $(CALIBRATION_OUT)

.FORCE:


ifndef PARAM_OVERRIDE
$(PARAM_OUT): $(PARAM_FILE) .FORCE | $(PARAM_BUILD_DIR)
	$(CC) -I param $(filter-out $(LTO), $(CFLAGS)) -c $(GIT_DEFINE) $< -o $(PARAM_BUILD_DIR)/$(notdir $(<:c=o)) 
	$(CP) -O binary -S -j flash_param $(PARAM_BUILD_DIR)/$(notdir $(<:c=o)) $@
	#-dfu-suffix -p 0x100 -v 0x3293 -a $@
endif

$(CALIBRATION_OUT): $(CALIBRATION_FILE) .FORCE | $(PARAM_BUILD_DIR)
	$(CC) -I $(SELF_DIR).. $(filter-out $(LTO), $(CFLAGS)) -c $< -o $(PARAM_BUILD_DIR)/$(notdir $(<:c=o))
	$(CP) -O binary -S -j calibration_data $(PARAM_BUILD_DIR)/$(notdir $(<:c=o)) $@
	#-dfu-suffix -p 0x100 -v 0x3293 -a $@

# if overrides 
define generateRules
a = $(PARAM_BUILD_DIR)/$(notdir $(PARAM_FILE:.c=))_$(1:.h=)
$(a).bin: $(PARAM_FILE) $(1) .FORCE | $(PARAM_BUILD_DIR)
	$(CC) -I param -I cal -include $(1) -c $(GIT_DEFINE) $(PARAM_FILE) -o $(a).o 
	$(CP) -O binary -S -j $(SECTION) $(a).o $(a).bin
	#-dfu-suffix -p 0x100 -v 0x3293 -a $@
endef

$(foreach ovr, $(PARAM_OVERRIDE), $(eval $(call generateRules, $(ovr))))

$(PARAM_BUILD_DIR):
ifneq "$(PARAM_BUILD_DIR)" "build"
	$(MKDIR) $@
endif

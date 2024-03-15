$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@
	-dfu-suffix -p 0x100 -v 0x3293 -a $@
	$(BIN) $< --remove-section flash_param --remove-section calibration_data $(@:.bin=_noparam.bin)
	-dfu-suffix -p 0x100 -v 0x3293 -a $(@:.bin=_noparam.bin)
	# $(BIN) $< --only-section flash_param $(@:.bin=_param_only.bin)
	# -dfu-suffix -p 0x100 -v 0x3293 -a $(@:.bin=_param_only.bin)

define dfu_leave =
tmp=\$$(mktemp) && rm \$$tmp && dfu-util -a0 -s 0x8000000:8:leave -U \$$tmp \$$@ || true; rm \$$tmp
endef
define dfu_param =
dfu-util -a0 -s 0x8060000 -D \$$(dirname \$$0)/$(notdir $(<:.bin=_param_only.bin)) \$$@
endef
define dfu_firmware =
dfu-util -a0 -s 0x8000000 -D \$$(dirname \$$0)/$(notdir $(<:.bin=_noparam.bin)) \$$@
endef
define dfu_calibration =
dfu-util -a0 -s 0x8070000 -D \$$(dirname \$$0)/$(notdir $(CALIBRATION_OUT)) \$$@
endef

$(BUILD_DIR)/%.tgz: $(BUILD_DIR)/%.bin $(PARAM_OUT) $(CALIBRATION_OUT) | $(BUILD_DIR)
	rm -rf $(BUILD_DIR)/$(TARGET)
	mkdir $(BUILD_DIR)/$(TARGET)
	cp $(<:.bin=_noparam.bin) $(BUILD_DIR)/$(TARGET)
	cp $(PARAM_OUT) $(BUILD_DIR)/$(TARGET)/$(notdir $(<:.bin=_param_only.bin))
	cp $(CALIBRATION_OUT) $(BUILD_DIR)/$(TARGET)/
	echo "#!/bin/bash -e\n$(DFU_START_LINE)\n$(dfu_firmware)\n$(dfu_param)\n$(dfu_calibration)\n$(dfu_leave)" > $(BUILD_DIR)/$(TARGET)/load_$(TARGET).sh
	chmod +x $(BUILD_DIR)/$(TARGET)/load_$(TARGET).sh
	echo "#!/bin/bash -e\n$(DFU_START_LINE)\n$(dfu_param)\n$(dfu_leave)" > $(BUILD_DIR)/$(TARGET)/load_$(TARGET)_param.sh
	chmod +x $(BUILD_DIR)/$(TARGET)/load_$(TARGET)_param.sh
	echo "#!/bin/bash -e\n$(DFU_START_LINE)\n$(dfu_calibration)\n$(dfu_leave)" > $(BUILD_DIR)/$(TARGET)/load_$(TARGET)_calibration.sh
	chmod +x $(BUILD_DIR)/$(TARGET)/load_$(TARGET)_calibration.sh
	tar czvf $@ -C $(BUILD_DIR) $(TARGET)

load: $(BUILD_TGZ)
	$(BUILD_DIR)/$(TARGET)/load_$(TARGET).sh

.PHONY: load
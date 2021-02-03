$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@
	dfu-suffix -p 0x100 -v 0x3293 -a $@	
	$(BIN) $< --remove-section flash_param $(@:.bin=_noparam.bin)
	dfu-suffix -p 0x100 -v 0x3293 -a $(@:.bin=_noparam.bin)
	$(BIN) $< --only-section flash_param $(@:.bin=_param_only.bin)
	dfu-suffix -p 0x100 -v 0x3293 -a $(@:.bin=_param_only.bin)


$(BUILD_DIR)/%.tgz: $(BUILD_DIR)/%.bin | $(BUILD_DIR)
	rm -rf $(BUILD_DIR)/$(TARGET)
	mkdir $(BUILD_DIR)/$(TARGET)
	cp $(<:.bin=_noparam.bin) $(BUILD_DIR)/$(TARGET)
	cp $(<:.bin=_param_only.bin) $(BUILD_DIR)/$(TARGET)
	echo "#!/bin/bash\ndfu-util -a0 -s 0x8000000 -D \$$(dirname \$$0)/$(notdir $(<:.bin=_noparam.bin))\ndfu-util -a0 -s 0x8060000:leave -D \$$(dirname \$$0)/$(notdir $(<:.bin=_param_only.bin))" > $(BUILD_DIR)/$(TARGET)/load_$(TARGET).sh
	chmod +x $(BUILD_DIR)/$(TARGET)/load_$(TARGET).sh
	tar czvf $@ -C $(BUILD_DIR) $(TARGET)

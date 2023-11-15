SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

include $(SELF_DIR)configure_gcc.mk
include $(SELF_DIR)configure.mk
include $(SELF_DIR)build.mk
include $(SELF_DIR)make_param.mk
include $(SELF_DIR)firmware_installer.mk
$(info )
.EXTRA_PREREQS += $(filter-out %.d, $(MAKEFILE_LIST)) $(CC)

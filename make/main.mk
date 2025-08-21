SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

ifndef TOP_MAKE
export TOP_MAKE := 1
info_once = $(info $(1))
endif

ifdef USE_LLVM
include $(SELF_DIR)configure_llvm.mk
else
include $(SELF_DIR)configure_gcc.mk
endif
include $(SELF_DIR)configure.mk
include $(SELF_DIR)build.mk
include $(SELF_DIR)make_param.mk
include $(SELF_DIR)firmware_installer.mk
include $(SELF_DIR)help.mk
$(call info_once,)
.EXTRA_PREREQS += $(filter-out %.d, $(MAKEFILE_LIST)) $(CC)

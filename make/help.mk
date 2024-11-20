.PHONY: help

define help_text
usage: make [CONFIG=CONFIG_OPTION] [C_DEFS=C_DEFS_OPTIONS] [-j] [TARGETS]
CONFIG_OPTION:
$(shell ls config | grep ^config_obot_g474_ | sed 's/config_obot_g474_//' | sed 's/.cpp$$//')
$(CONFIG) (default)

C_DEFS_OPTIONS:
	-DNO_WATCHDOG

TARGETS:
	clean_build (default) all clean param load help

endef

export help_text
help:
	$(call info_once,$(help_text))
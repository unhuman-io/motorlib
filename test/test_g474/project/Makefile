MOTORLIB_DIR := ../../..

include ../../../scripts/configure_gcc.mk

C_INCLUDES =  \
-I. \
-I$(MOTORLIB_DIR)/device/stm32g4/Include \
-I$(MOTORLIB_DIR)/drivers/CMSIS/Include

G474_INCLUDES = \
-include $(MOTORLIB_DIR)/peripheral/stm32g4/spi_dma.h

#headers := $(wildcard $(MOTORLIB_DIR)/*.h)
headers := $(MOTORLIB_DIR)/ads1235.h \
$(MOTORLIB_DIR)/ads8339.h \
$(MOTORLIB_DIR)/aksim2_encoder.h \
$(MOTORLIB_DIR)/icpz.h

syntax: $(headers:.h=.syn)

%.syn: %.h
	$(CXX) $(C_INCLUDES) $(G474_INCLUDES) -fsyntax-only $<

.PHONY: syntax

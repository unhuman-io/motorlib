
usage := "USAGE: make NAME=name REV=rev NUM=num [SN=sn]"

ifndef NAME
$(error $(usage))
endif

ifndef REV
$(error $(usage))
endif

ifndef NUM
$(error $(usage))
endif

ifdef SN
SN_LINE := -S $(SN)
endif

srcs = otp.cpp

C_INCLUDES = \
-I../../..

C_DEFS = \
-DNAME=\"$(NAME)\" \
-DREV=\"$(REV)\" \
-DNUM=$(NUM) \

clean_build::
	$(MAKE) clean
	$(MAKE) all
	
all: $(srcs:cpp=bin)

%.o: %.cpp
	$(CXX) $(C_INCLUDES) $(C_DEFS) -c $< $@

%.bin: %.o
	$(CP) -O binary -S -j otp $< $@ 

clean:
	rm -rf *.o *.d *.bin

program: clean_build
	dfu-util -a2 -s 0x1FFF7000:leave -D otp.bin $(SN_LINE)

.PHONY: program clean_build all


TARGET_MCU := stm32g474
include ../../../make/configure_gcc.mk

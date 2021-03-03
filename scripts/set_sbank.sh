#!/bin/bash

dfu-util -a1 -s 0x1FFF7800:16 -D $(dirname $0)/sbank.bin $@

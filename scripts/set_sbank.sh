#!/bin/bash

fname=sbank.bin
if [ $1 == "-c" ]; then
fname=dbank.bin
shift
fi
echo $fname
dfu-util -a1 -s 0x1FFF7800:8 -D $(dirname $0)/$fname $@

#!/bin/bash

dir=$(dirname $0)/option_bytes
fname=sbank.bin
if [ $1 == "-c" ]; then
    fname=dbank.bin
shift
elif [ $1 == "-f" ]; then
    fname=factory7800.bin
shift
fi
echo $dir/$fname
dfu-util -a1 -s 0x1FFF7800 -D $dir/$fname $@
if [ $fname == "factory7800.bin" ]; then
    sleep 1
    dfu-util -a1 -s 0x1FFFF800 -D $dir/factoryf800.bin $@
fi

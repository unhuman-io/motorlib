#!/bin/bash

rm -f tmp1.dat
dfu-util -a0 -s 0x8060000:leave -U tmp1.dat $@
rm tmp1.dat
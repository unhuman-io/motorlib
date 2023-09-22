#!/bin/bash

rm -f tmp1.dat
dfu-util -a0 -S $1 -s 0x8060000 -U tmp1.dat
tail -c+65 tmp1.dat > tmp2.dat
len=${#2}
zero_len=$((63-$len))
echo $2 > tmp3.dat
head -c $zero_len /dev/zero >> tmp3.dat
cat tmp2.dat >> tmp3.dat
dfu-util -a0 -S $1 -s 0x8060000:leave -D tmp3.dat

rm tmp{1..3}.dat
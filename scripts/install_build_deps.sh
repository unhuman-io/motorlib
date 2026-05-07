#!/bin/bash -e

cd $(dirname $0)

sudo pwd
sudo apt install -y cmake ninja-build &
./install_gcc.sh &
./install_llvm.sh &

wait
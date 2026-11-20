#!/bin/bash -e

trap `rm -rf ../dfu-util` EXIT

sudo apt install -y libusb-1.0-0-dev autoconf pandoc
git clone git://git.code.sf.net/p/dfu-util/dfu-util
cd dfu-util
git apply ../dfu-util.patch
./autogen.sh
./configure
make -j
sudo make install
rm -rf ../dfu-util

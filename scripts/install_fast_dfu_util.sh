#!/bin/bash -e

git clone git://git.code.sf.net/p/dfu-util/dfu-util
cd dfu-util
git apply ../dfu-util.patch
./autogen.sh
./configure
make -j
sudo make install
rm -rf dfu-util

#!/bin/bash

set -e

llvm_dir=$(dirname $0)/../llvm
mkdir -p $llvm_dir
cd $llvm_dir

# version=20.1.0
# declare -A sha=( [x86_64]=c1179396608c07bf68f3014923cfdfcd11c8402a3732f310c23d07c9a726b275 \
#                  [aarch64]=2fa9220f64097b71c07e6de2917f33fda1bb736964730786e90a430fdc0fa6be \
#                  [newlib]=2e649e17b161b81f0ddf5361dc8f648d66f9c1cf9665c6b8a621d44aef51ad7d)

version=21.1.1
declare -A sha=( [x86_64]=fd7fcc2eb4c88c53b71c45f9c6aa83317d45da5c1b51b0720c66f1ac70151e6e \
                 [aarch64]=dfd93d7c79f26667f4baf7f388966aa4cbfd938bc5cbcf0ae064553faf3e9604 \
                 [newlib]=d9750863c5561c05a57f6df6019efea87e9206c0eef34c4e6441f339824cc908)

arch=$(uname -m)
if [ "$arch" == "aarch64" ]; then
  farch=AArch64
else
  farch=x86_64
fi

fnames=(ATfE-$version-Linux-$farch \
        ATfE-newlib-overlay-$version)
sha_keys=($arch newlib)
for i in "${!fnames[@]}"; do
  fname=${fnames[$i]}
  url=https://github.com/arm/arm-toolchain/releases/download/release-$version-ATfE/$fname.tar.xz
  wget $quiet $url -O $fname.tar.xz
  cat <(printf "${sha[${sha_keys[$i]}]} $fname.tar.xz\n")
  sha256sum -c <(printf "${sha[${sha_keys[$i]}]} $fname.tar.xz\n")
  if [ $i -eq 0 ]; then
    tar xf $fname.tar.xz --strip-components=1
  else
    tar xf $fname.tar.xz
  fi
  
  rm $fname.tar.xz
done

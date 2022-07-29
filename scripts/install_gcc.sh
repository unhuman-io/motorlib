#!/bin/bash

set -e

gcc_dir=$(dirname $0)/../gcc
mkdir -p $gcc_dir
cd $gcc_dir

version=gcc-arm-11.2-2022.02
arch=$(uname -m)
fname=$version-$arch-arm-none-eabi
declare -A sha=( [x86_64]=8c5acd5ae567c0100245b0556941c237369f210bceb196edfe5a2e7532c60326 \
                 [aarch64]=ef1d82e5894e3908cb7ed49c5485b5b95deefa32872f79c2b5f6f5447cabf55f )

url=https://developer.arm.com/-/media/Files/downloads/gnu/11.2-2022.02/binrel/$fname.tar.xz
wget $url
cat <(printf "${sha[$arch]} $fname.tar.xz\n")
sha256sum -c <(printf "${sha[$arch]} $fname.tar.xz\n")
tar xf $fname.tar.xz --strip-components=1
rm $fname.tar.xz

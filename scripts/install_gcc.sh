#!/bin/bash

set -e

gcc_dir=$(dirname $0)/../gcc
mkdir -p $gcc_dir
cd $gcc_dir

# version=11.2-2022.02
# declare -A sha=( [x86_64]=8c5acd5ae567c0100245b0556941c237369f210bceb196edfe5a2e7532c60326 \
#                  [aarch64]=ef1d82e5894e3908cb7ed49c5485b5b95deefa32872f79c2b5f6f5447cabf55f )
# version=11.3.rel1
# declare -A sha=( [x86_64]=d420d87f68615d9163b99bbb62fe69e85132dc0a8cd69fca04e813597fe06121 \
#                  [aarch64]=fb9e562a90de1b3a2961b952193c1c6520872aa1482c0a5e0ab79970ec6e7690 )
#
# version=12.2.rel1
# declare -A sha=( [x86_64]=84be93d0f9e96a15addd490b6e237f588c641c8afdf90e7610a628007fc96867 \
#                  [aarch64]=62d66e0ad7bd7f2a183d236ee301a5c73c737c886c7944aa4f39415aab528daf )

version=12.3.rel1
declare -A sha=( [x86_64]=12a2815644318ebcceaf84beabb665d0924b6e79e21048452c5331a56332b309 \
                 [aarch64]=960ec0bce309528f603639d8228ef39e6fb9185289ff42b01aa3b4de315accef )

arch=$(uname -m)
fname=arm-gnu-toolchain-$version-$arch-arm-none-eabi
url=https://developer.arm.com/-/media/Files/downloads/gnu/$version/binrel/$fname.tar.xz
wget $url
cat <(printf "${sha[$arch]} $fname.tar.xz\n")
sha256sum -c <(printf "${sha[$arch]} $fname.tar.xz\n")
tar xf $fname.tar.xz --strip-components=1
rm $fname.tar.xz

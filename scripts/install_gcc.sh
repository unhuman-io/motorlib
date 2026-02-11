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
#
# version=12.3.rel1
# declare -A sha=( [x86_64]=12a2815644318ebcceaf84beabb665d0924b6e79e21048452c5331a56332b309 \
#                  [aarch64]=960ec0bce309528f603639d8228ef39e6fb9185289ff42b01aa3b4de315accef )

# version=13.2.rel1
# declare -A sha=( [x86_64]=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb \
#                  [aarch64]=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a )

# version=13.3.rel1
# declare -A sha=( [x86_64]=95c011cee430e64dd6087c75c800f04b9c49832cc1000127a92a97f9c8d83af4 \
#                  [aarch64]=c8824bffd057afce2259f7618254e840715f33523a3d4e4294f471208f976764 )

# version=14.2.rel1
# declare -A sha=( [x86_64]=62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823 \
#                  [aarch64]=87330bab085dd8749d4ed0ad633674b9dc48b237b61069e3b481abd364d0a684 )

# version=14.3.rel1
# declare -A sha=( [x86_64]=8f6903f8ceb084d9227b9ef991490413014d991874a1e34074443c2a72b14dbd \
#                  [aarch64]=ebaf2d47f2e7f7b645864c5c8cf839e526daed83a2e675a3525d03f5ba3d2be9 )

version=15.2.rel1
declare -A sha=( [x86_64]=597893282ac8c6ab1a4073977f2362990184599643b4c5ee34870a8215783a16 \
                 [aarch64]=d061559d814b205ed30c5b7c577c03317ec447ca51cd5a159d26b12a5bbeb20c )

arch=$(uname -m)
fname=arm-gnu-toolchain-$version-$arch-arm-none-eabi
url=https://developer.arm.com/-/media/Files/downloads/gnu/$version/binrel/$fname.tar.xz
wget $quiet $url -O $fname.tar.xz
cat <(printf "${sha[$arch]} $fname.tar.xz\n")
sha256sum -c <(printf "${sha[$arch]} $fname.tar.xz\n")
tar xf $fname.tar.xz --strip-components=1
rm $fname.tar.xz

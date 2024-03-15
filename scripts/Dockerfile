FROM ubuntu:22.04

COPY install_gcc.sh /install_gcc.sh
RUN apt-get update && \
    apt-get install -y wget xz-utils build-essential git && \
    /install_gcc.sh

# newer dfu util
COPY install_fast_dfu_util.sh /install_fast_dfu_util.sh
COPY dfu-util.patch /dfu-util.patch
RUN apt-get install -y autoconf libusb-1.0-0-dev && \
    /install_fast_dfu_util.sh

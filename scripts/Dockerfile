FROM ubuntu:24.04

COPY install_gcc.sh /install_gcc.sh
RUN apt-get update && \
    apt-get install -y wget xz-utils build-essential dfu-util git && \
    /install_gcc.sh
ENV PATH="${PATH}:/gcc/bin"

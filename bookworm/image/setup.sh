#!/bin/bash

set -eu -o pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get -y update
apt-get -y upgrade
apt-get install -y --no-install-recommends $(cat <<EOF
build-essential
cmake
coinor-libcoinutils-dev
default-jdk
file
g++
gcc
gfortran
libclang-14-dev
libeigen3-dev
libgfortran-12-dev
libgl1-mesa-dev
libglib2.0-dev
libjchart2d-java
liblapack-dev
libmumps-seq-dev
libopengl-dev
libspdlog-dev
libx11-dev
libxt-dev
lsb-release
make
nasm
ocl-icd-opencl-dev
opencl-headers
patch
patchelf
pkg-config
python3-all-dev
python3-matplotlib
python3-numpy
python3-yaml
yasm
zlib1g-dev
EOF
)

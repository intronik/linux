#!/bin/sh -e
# settings start here
DIR=$PWD
echo building intronik kernel for beagle bone
export CC=/home/intronik/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
export CORES=1
export BUILD=intronik9
export DISTRO=cross
export DEBARCH=armhf
export ARCH=arm
export config="intronik_defconfig"
export DEPLOY=/home/intronik/deploy
export LOCALVERSION=-${BUILD}
export CROSS_COMPILE="${CC}"
mkdir -p ${DEPLOY}
# make --jobs=${CORES} menuconfig
# make --jobs=${CORES} zImage modules dtbs
fakeroot make -j${CORES} ARCH=arm KBUILD_DEBARCH=${DEBARCH} LOCALVERSION=-${BUILD} CROSS_COMPILE=${CC} KDEB_PKGVERSION=1${DISTRO} deb-pkg

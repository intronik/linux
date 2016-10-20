#!/bin/sh -e
# settings start here
DIR=$PWD
echo building intronik kernel for beagle bone
export CC=/home/INTRONIK/linuxdev/gcc-4.9/bin/arm-linux-gnueabihf-
export CORES=1
export BUILD=intronik7
export DISTRO=cross
export DEBARCH=armhf
export VERSION=jessie
export ARCH=arm
# export config="intronik_defconfig"
export DEPLOY=/home/INTRONIK/linuxdev/deploy
export LOCALVERSION=-${BUILD}
export CROSS_COMPILE="${CC}"
mkdir -p ${DEPLOY}
#make --jobs=${CORES} menuconfig
#make --jobs=${CORES} zImage modules dtbs
# https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/commit/scripts/package/builddeb?id=3716001bcb7f5822382ac1f2f54226b87312cc6b
fakeroot make -j${CORES} ARCH=arm KBUILD_DEBARCH=${DEBARCH} LOCALVERSION=-${BUILD} CROSS_COMPILE=${CC} KDEB_SOURCENAME=intronik KDEB_PKGVERSION=1 deb-pkg

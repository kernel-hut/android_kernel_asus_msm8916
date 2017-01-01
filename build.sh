#!/bin/bash

# Made By ShadowReaper1 @xda-developers <ramanverma4183@gmail.com>.
# Do not take this script without my permission.
# Edit the fields according to your environment.
# This script is universal.

KERNEL_DIR=$PWD
# CHANGE THIS TO YOUR ARM64 TOOLCHAIN DIRECTORY IF YOU ARE BUILDING FOR A 64BIT DEVICE
ARM64_TOOLCHAIN=~/UBERTC/bin/aarch64-linux-android-
# YOUR DEVICES DEFCONFIG USUALLY STARTS WITH "DEVICE CODENAME" AND ENDS WITH "_defconfig"
MAX=zc550kl-custom_defconfig
LASER=ze500kl-custom_defconfig
DTBTOOL=$KERNEL_DIR/tools
COMPLETED_BUILDS=../zip
CORES=4
BUILDS_DIR=$KERNEL_DIR/zip
ANYKERNEL=$KERNEL_DIR/zip/base_files
KERNEL_ZIP=Reaper-Darkness_$(date +%d%m%Y_%H%M).zip

# Build Starts From Here.

echo ================================================================
echo
echo First Lets Take Care Of This
echo
echo ================================================================ 
echo "Do you wanna clean the build(y/n)"
read answer
if echo "$answer" | grep -iq "^y" ;then 
echo "Cleaning"
make clean mrproper
rm -rf $KERNEL_DIR/arch/arm64/boot/dtb
rm -rf $KERNEL_DIR/arch/arm64/boot/Image
rm -rf $KERNEL_DIR/arch/arm64/boot/Image.gz
rm -rf $KERNEL_DIR/arch/arm64/boot/Image.gz-dtb
rm -rf $KERNEL_DIR/arch/arm64/boot/dts/*.dtb
rm -rf $KERNEL_DIR/arch/arm/boot/dtb
rm -rf $KERNEL_DIR/arch/arm/boot/Image
rm -rf $KERNEL_DIR/arch/arm/boot/Image.gz
rm -rf $KERNEL_DIR/arch/arm/boot/Image.gz-dtb
rm -rf $KERNEL_DIR/arch/arm/boot/dts/*.dtb
fi
clear
echo ================================================================
echo
echo                           Starting                              
echo
echo ================================================================
echo
echo ================================================================
echo
echo Please Enter The Name Of The Device You Want To Build Kernel For
echo
echo For Zenfone Max Type MAX
echo For Zenfone 2 Laser Type LASER
echo ================================================================
read answer
if echo "$answer" | grep -iq "^max" ;then
export ARCH=arm64
export SUBARCH=arm64
export CROSS_COMPILE=$ARM64_TOOLCHAIN
clear
echo "Building Kernel For Zenfone Max"
make $MAX
clear
echo ==========================
echo Continuing the process...
echo ==========================
echo "Do you wanna make changes in the defconfig (y/n)?"
read answer
if echo "$answer" | grep -iq "^y" ;then
echo Building Defconfig GUI
make menuconfig
echo "Do you wanna save the changes in the defconfig (y/n)?"
read answer
if echo "$answer" | grep -iq "^y" ;then
echo "Building Defconfig GUI"
cp -f .config $KERNEL_DIR/arch/arm64/configs/zc550kl-custom_defconfig
fi
clear
echo ==========================
echo Time To Wrap it up.......
echo ==========================
fi
time make -j$CORES
$DTBTOOL/dtbToolCM -2 -o $KERNEL_DIR/arch/arm64/boot/dtb -s 2048 -p $KERNEL_DIR/scripts/dtc/ $KERNEL_DIR/arch/arm64/boot/dts/
rm -rf $ANYKERNEL/dtb
rm -rf $ANYKERNEL/zImage
rm -rf $ANYKERNEL/*.zip
rm -rf $BUILD_DIR/*.zip
cp $KERNEL_DIR/arch/arm64/boot/Image $ANYKERNEL/zImage
cp $KERNEL_DIR/arch/arm64/boot/dtb $ANYKERNEL/
cd $ANYKERNEL/
zip -r9 MAX_$KERNEL_ZIP * -x README MAX_$KERNEL_ZIP
mv $KERNEL_DIR/zip/base_files/MAX_$KERNEL_ZIP $KERNEL_DIR/zip/MAX_$KERNEL_ZIP
cd $KERNEL_DIR
fi
if echo "$answer" | grep -iq "^laser" ;then
export ARCH=arm64
export SUBARCH=arm64
export CROSS_COMPILE=$ARM64_TOOLCHAIN
clear
echo "Building Kernel For Zenfone 2 LASER"
make $LASER
clear
echo ==========================
echo Continuing the process...
echo ==========================
echo "Do you wanna make changes in the defconfig (y/n)?"
read answer
if echo "$answer" | grep -iq "^y" ;then
echo Building Defconfig GUI
make menuconfig
echo "Do you wanna save the changes in the defconfig (y/n)?"
read answer
if echo "$answer" | grep -iq "^y" ;then
echo "Building Defconfig GUI"
cp -f .config $KERNEL_DIR/arch/arm64/configs/ze500kl-custom_defconfig
fi
clear
echo ==========================
echo Time To Wrap it up.......
echo ==========================
fi
time make -j$CORES
$DTBTOOL/dtbToolCM -2 -o $KERNEL_DIR/arch/arm64/boot/dtb -s 2048 -p $KERNEL_DIR/scripts/dtc/ $KERNEL_DIR/arch/arm64/boot/dts/
rm -rf $ANYKERNEL/dtb
rm -rf $ANYKERNEL/zImage
rm -rf $ANYKERNEL/*.zip
rm -rf $BUILD_DIR/*.zip
cp $KERNEL_DIR/arch/arm64/boot/Image $ANYKERNEL/zImage
cp $KERNEL_DIR/arch/arm64/boot/dtb $ANYKERNEL/
cd $ANYKERNEL/
zip -r9 LASER_$KERNEL_ZIP * -x README LASER_$KERNEL_ZIP
mv $KERNEL_DIR/zip/base_files/LASER_$KERNEL_ZIP $KERNEL_DIR/zip/LASER_$KERNEL_ZIP
cd $KERNEL_DIR
fi

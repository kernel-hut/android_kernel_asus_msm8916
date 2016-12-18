#!/bin/bash

### Taken From Ruthless Kernel
### HelpMeRuth (jukeboxruthger1@gmail.com)
### Made universal for everyone. Make sure you run with sudo or su

set -e

KERNEL_DIR=$PWD
TOOLCHAINDIR=~/UBERTC
KERNEL_TOOLCHAIN=$TOOLCHAINDIR/bin/aarch64-linux-android-
KERNEL_DEFCONFIG=zc550kl-custom_defconfig
DTBTOOL=$KERNEL_DIR/tools/
BUILDS=../zip
JOBS=4
BUILD_DIR=$KERNEL_DIR/zip
ANY_KERNEL2_DIR=$KERNEL_DIR/zip/base_files

# The MAIN Part
echo "**** Processing ****"
echo " "
export ARCH=arm64
export SUBARCH=arm64
export CROSS_COMPILE=$KERNEL_TOOLCHAIN
echo "**** Do you wanna clean the build(y/n)? ****"
read answer
if echo "$answer" | grep -iq "^y" ;then
    echo -e "${bldgrn} Cleaning the old build ${txtrst}"
make mrproper
rm -rf $KERNEL_DIR/arch/arm64/boot/dtb
rm -rf $KERNEL_DIR/arch/arm64/boot/Image
rm -rf $KERNEL_DIR/arch/arm64/boot/Image.gz
rm -rf $KERNEL_DIR/arch/arm64/boot/Image.gz-dtb
rm -rf $KERNEL_DIR/arch/arm64/boot/dts/*.dtb
fi
echo "**** Kernel defconfig is set to $KERNEL_DEFCONFIG ****"
make $KERNEL_DEFCONFIG
echo -n "Do you wanna make changes in the defconfig (y/n)?"
read answer
if echo "$answer" | grep -iq "^y" ;then
    echo -e " Building Defconfig GUI ${txtrst}"
make menuconfig

echo -n "Do you wanna save the changes in the defconfig (y/n)?"
read answer
if echo "$answer" | grep -iq "^y" ;then
    echo -e "${bldgrn} Building Defconfig GUI ${txtrst}"
cp -f .config $KERNEL_DIR/arch/arm64/configs/zc550kl-custom_defconfig
fi
fi
make -j$JOBS

# Time for dtb
echo "**** Generating DTB ****"
$DTBTOOL/dtbToolCM -2 -o $KERNEL_DIR/arch/arm64/boot/dtb -s 2048 -p $KERNEL_DIR/scripts/dtc/ $KERNEL_DIR/arch/arm64/boot/dts/

echo "**** Verify zImage,dtb ****"
ls $KERNEL_DIR/arch/arm64/boot/Image
ls $KERNEL_DIR/arch/arm64/boot/dtb

#Anykernel 2 time!!
echo "**** Verifying Build Directory ****"
ls $ANY_KERNEL2_DIR
echo "**** Removing leftovers ****"
rm -rf $ANY_KERNEL2_DIR/dtb
rm -rf $ANY_KERNEL2_DIR/zImage
### just get rid of every zip, who wants a zip in a zip?
rm -rf $BUILD_DIR/*.zip

echo "**** Copying zImage ****"
cp $KERNEL_DIR/arch/arm64/boot/Image $ANY_KERNEL2_DIR/zImage
echo "**** Copying dtb ****"
cp $KERNEL_DIR/arch/arm64/boot/dtb $ANY_KERNEL2_DIR/

## Set build number
echo "**** Setting Build Number ****"
NUMBER=$(cat number)
INCREMENT=$(expr $NUMBER + 1)
sed -i s/$NUMBER/$INCREMENT/g $KERNEL_DIR/number
FINAL_KERNEL_ZIP=Reaper-Darkness-V$INCREMENT.0.zip

echo "**** Time to zip up! ****"
cd $ANY_KERNEL2_DIR/
zip -r9 $FINAL_KERNEL_ZIP * -x README $FINAL_KERNEL_ZIP
cd ..
cd ..
echo $FINAL_KERNEL_ZIP
mv $KERNEL_DIR/zip/base_files/$FINAL_KERNEL_ZIP zip/$FINAL_KERNEL_ZIP

echo "**** Good Bye!! ****"
cd $KERNEL_DIR

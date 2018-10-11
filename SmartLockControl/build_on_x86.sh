#!/bin/bash -e

export CROSS_COMPILE=arm-linux-gnueabi-

MODNAME_ADDITATION_1F="External_GPIO_module"
MODNAME_ADDITATION_1="extGPIO.ko"
MODNAME_F="SmartLock"
MODNAME="SmartLockControl.ko"

OFILE="$BUILD_KERNEL/include/generated/utsrelease.h"
MESSAGE="#define UTS_RELEASE \"4.17.11-sunxi\" "


# parse commandline options
while [ ! -z "$1"  ] ; do
        case $1 in
           -h|--help)
                echo "TODO: help"
                ;;
            --clean)
                echo "Clean module sources"
                make ARCH=arm clean
                ;;
            --module)
                echo "Build module"
		echo "$MESSAGE" > "$OFILE"
                make ARCH=arm

		FLOCAL=`pwd`
                cp "$FLOCAL/$MODNAME_ADDITATION_1F/$MODNAME_ADDITATION_1" $FLOCAL
		cp "$FLOCAL/$MODNAME_F/$MODNAME" $FLOCAL
                ;;
            --deploy)
                echo "Deploy kernel module"
                scp $MODNAME_ADDITATION_1 opi:~
		scp $MODNAME opi:~
                ;;
            --kconfig)
                echo "configure kernel"
                make ARCH=arm config
                ;;
            
            --dtb)
                echo "configure kernel"
                make ARCH=arm dtb
                cp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dts ${TRAINING_ROOT}
		scp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dtb opi:~
                ;;
        esac
        shift
done

echo "Done!"

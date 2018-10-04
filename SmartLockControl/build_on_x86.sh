#!/bin/bash -e

export CROSS_COMPILE=arm-linux-gnueabi-

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
                ;;
            --deploy)
                echo "Deploy kernel module"
                scp $MODNAME root@192.168.10.2:~
                scp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dtb root@192.168.10.2:~
                ;;
            --kconfig)
                echo "configure kernel"
                make ARCH=arm config
                ;;
            
            --dtb)
                echo "configure kernel"
                make ARCH=arm dtb
                cp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dts ${TRAINING_ROOT}
                ;;
        esac
        shift
done

echo "Done!"

#!/bin/bash -e


MODNAME="st7735.ko"

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
                make ARCH=arm
                ;;
            --deploy)
                echo "Deploy kernel module"
                cp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dts ${TRAINING_ROOT}/st7735
                scp $MODNAME Orange:~/
                scp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dtb Orange:~/
                ;;
            --kconfig)
                echo "configure kernel"
                make ARCH=arm config
                ;;
            
            --dtb)
                echo "configure kernel"
                make ARCH=arm dtb
                ;;
        esac
        shift
done

echo "Done!"

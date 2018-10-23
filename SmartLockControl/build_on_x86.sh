#!/bin/bash -e

export CROSS_COMPILE=arm-linux-gnueabihf-

SSH_NAME=
BOARD_USER=armuser
BOARD_IP=192.168.102.126


BOARD_ADDRESS=${SSH_NAME:-$BOARD_USER@$BOARD_IP}

# parse commandline options
while [ ! -z "$1"  ] ; do
        case $1 in
           -h|--help)
                echo "TODO: help"
                ;;
            --clean)
                echo "Clean module sources"
                make clean
                ;;
            --module)
                make 
                ;;
            --deploy)
                echo "Deploy kernel module"
                scp External_GPIO_module/extGPIO.ko SmartLock/SmartLockControl.ko mfrc522_driver/mfrc522.ko $BOARD_ADDRESS:~
                ;;
            --kconfig)
                echo "configure kernel"
                make config
                ;;
            
            --dtb)
                echo "configure kernel"
                make dtb
                cp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dts ${TARGET_ROOT}
		        scp $BUILD_KERNEL/arch/arm/boot/dts/sun8i-h3-orangepi-one.dtb $BOARD_ADDRESS:~
                ;;
        esac
        shift
done

echo "Done!"

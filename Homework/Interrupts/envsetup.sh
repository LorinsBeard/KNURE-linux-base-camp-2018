#! /bin/sh


export CROSS_COMPILE=arm-linux-gnueabi-

export TRAINING_ROOT=${HOME}/Documents/Course/KNURE-linux-base-camp-2018/Homework/Interrupts

export BUILD_KERNEL=${HOME}/Documents/Course/Armbian/arm/source/usr/src/


#export KERNEL_IMG=${BUILD_KERNEL}/arch/x86/boot/bzImage
#export ROOTFS_IMG=${TRAINING_ROOT}/rootfs.img

echo -e "\t CROSS_COMPILE \t = ${CROSS_COMPILE}"
echo -e "\t TRAINING_ROOT \t = ${TRAINING_ROOT}"
echo -e "\t BUILD_KERNEL \t = ${BUILD_KERNEL}"
#echo -e "\t BUILD_ROOTFS \t = ${BUILD_ROOTFS}"

#cp ${BUILD_ROOTFS}/images/rootfs.ext2 ${TRAINING_ROOT}/rootfs.img
#qemu-system-i386 -kernel ${KERNEL_IMG} -append "root=/dev/sda" -hda ${ROOTFS_IMG} -redir tcp:8022::22 &

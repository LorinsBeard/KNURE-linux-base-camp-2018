#!/bin/sh


export CROSS_COMPILE=arm-linux-gnueabihf-

export TARGET_ROOT=${HOME}/Documents/Course/Attestation_Project_Linux_Kernel/GIT/SmartLockControl

export BUILD_KERNEL=${HOME}/Documents/Course/Armbian/arm/source/usr/src/


echo -e "\t CROSS_COMPILE \t = ${CROSS_COMPILE}"
echo -e "\t TARGET_ROOT \t = ${TARGET_ROOT}"
echo -e "\t BUILD_KERNEL \t = ${BUILD_KERNEL}"


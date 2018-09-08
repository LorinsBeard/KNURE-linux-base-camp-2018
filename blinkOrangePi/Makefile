KDIR = ~/armbianBuild/build/cache/sources/linux-mainline/linux-4.14.y/

export CROSS_COMPILE=~/armbianBuild/build/cache/toolchains/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-

PWD = $(shell pwd) 
TARGET = ledOrange

obj-m   := $(TARGET).o 

default:
	$(MAKE) ARCH=arm -C $(KDIR) M=$(PWD) modules 
	
clean: 
	@rm -f *.o .*.cmd .*.flags *.mod.c *.order 
	@rm -f .*.*.cmd *~ *.*~ TODO.*
	@rm -fR .tmp* 
	@rm -rf .tmp_versions 
	
disclean: clean 
	@rm *.ko *.symvers

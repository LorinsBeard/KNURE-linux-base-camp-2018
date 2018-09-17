KDIR = ~/armbianBuild/build/cache/sources/linux-mainline/linux-4.14.y/

export CROSS_COMPILE=~/armbianBuild/build/cache/toolchains/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-

CONFIG_MODULE_SIG=n

obj-m := ssd1306.o

PWD := $(shell pwd)

default:
		  $(MAKE) ARCH=arm  -C $(KDIR) M=$(PWD) modules

clean:
		  $(MAKE) ARCH=arm  -C $(KDIR) SUBDIRS=$(PWD) clean

config:
		$(MAKE) ARCH=arm -C $(KDIR) menuconfig

dtb:
		$(MAKE) ARCH=arm -C $(KDIR) dtbs

.PHONY: clean default config dtb

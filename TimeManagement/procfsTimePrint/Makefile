#
# procfs example
#

TARGET = procfs_printTime


ifneq ($(KERNELRELEASE),)

obj-m := $(TARGET).o
$(TARGET)-objs := print_time.o

else

KERNELDIR := $(BUILD_KERNEL)

.PHONY: all clean
all:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) modules
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) clean

endif

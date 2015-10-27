CURRENT_PATH := $(shell pwd)
ARM_LINUX_KERNEL :=/work2/sdk06/board-support/linux-3.2.0-psp04.06.00.11

obj-m +=led.o
obj-m +=button.o
obj-m += powerRelay.o
obj-m +=ads8320.o
obj-m +=ioGather.o
obj-m +=relayCtl.o
obj-m +=lcd1602.o
modules:
	$(MAKE) -C $(ARM_LINUX_KERNEL) M=$(CURRENT_PATH) modules

clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions Module.symvers .Makefile.swp modules.order

.PHONY: modules modules_install clean

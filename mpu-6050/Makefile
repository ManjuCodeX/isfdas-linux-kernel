obj-m += mpu_6050.o

ccflags-y += -fno-stack-protector

KDIR := /home/manju/linux-6.12.91
PWD := $(shell pwd)

KBUILD_EXTRA_SYMBOLS := /home/manju/full_proj/isfdas/Module.symvers

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		-C $(KDIR) M=$(PWD) \
		KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS) \
		modules

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		-C $(KDIR) M=$(PWD) clean

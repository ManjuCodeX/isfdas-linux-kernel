obj-m += w25q64.o

ccflags-y += -fno-stack-protector

KDIR := /home/manju/linux-6.12.91
PWD  := $(shell pwd)

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		-C $(KDIR) M=$(PWD) modules

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		-C $(KDIR) M=$(PWD) clean


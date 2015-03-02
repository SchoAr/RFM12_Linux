# Makefile example
#   Adrian Remonda 2014

ifneq ($(KERNELRELEASE),)
	obj-m := RFM12.o
	CFLAGS_hello.o := -DDEBUG
else
	obj-m := RFM12.o
	TARGET = 192.168.1.103
	TARGET_DIR = /root
	KERNELDIR ?= ~/Driver_Development/Neuer_Ordner2/linux-xlnx
	PWD := $(shell pwd)

default:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(CC) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(CC) -C $(KERNELDIR) M=$(PWD) clean

install:        
	scp *.ko root@$(TARGET):$(TARGET_DIR)

check: 
	$(MAKE) ARCH=arm CROSS_COMPILE=$(CC) -C $(KERNELDIR) M=$(PWD) modules -Wextra -Werror
endif



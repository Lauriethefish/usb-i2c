include ../.env

obj-m += usb-i2c.o
ifeq ($(DRIVER_DEBUG),1)
	CFLAGS_usb-i2c.o += -DDEBUG
endif

PWD := $(CURDIR)

all: ../.env
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
 

clean: 
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

remove:
	-sudo rmmod usb-i2c.ko

insert: remove all
	sudo insmod usb-i2c.ko

logs:
	journalctl --since "10 minutes ago" | grep kernel
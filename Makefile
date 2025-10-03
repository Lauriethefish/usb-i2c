
all:
	$(MAKE) -C usb-i2c
	$(MAKE) -C usb-i2c-rom build

clean:
	$(MAKE) -C usb-i2c clean
	$(MAKE) -C usb-i2c-rom clean
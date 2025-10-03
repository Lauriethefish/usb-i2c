# USB-I2C
An "I2C over USB bridge" which allows I2C devices to be used on a modern computer without any I2C ports. Based on the STM32 blue pill.

It supports I2C standard mode and fast mode, with configurable clock frequency within each mode.

## Structure
This project has two main parts:
- A ROM for the STM32F103 which allows function as a USB to I2C converter. This is `usb-i2c-rom`.

- A Linux driver which presents a standard I2C interface, implemented via a USB connection. This is `usb-i2c`.

## Compilation

### Requirements

For the driver:
- GCC 13 or newer.
- Linux headers. Install with `sudo apt-get install linux-headers-generic`.
- GNU make.

For the ROM:
- The `arm-none-eabi` toolchain. Download from [here](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain), extract, and add the path to `.env`.
- OpenOCD for flashing to the STM32. Install with `sudo apt-get install openocd`.

### Building

To build everything in one go, run `make` from the project root.

#### Enabling debug mode
If you wish to read debug messages from the ROM or driver, edit `.env`,
and change `ROM_DEBUG`/`DRIVER_DEBUG` from 0 to 1. Run `make clean` after doing this.

#### Testing the ROM

Firstly, check you have the hardware requirements all set up.
You need an `STM32F103` microcontroller, on some sort of testing board (I use a blue pill, but I imagine other boards should work fine).

Pin `B10` is the I2C `SCL` and `B11` is `SDA`. These need a pullup resistor to VDD - the exact requirement depends on the length of your bus.
Often any I2C devices you connect will have built-in pullup.

For testing, I have been using an SH1106 display, since there are lots of libraries out there to manipulate them, but you can use whatever you wish.

You also need an ST-LINK to connect your board to a PC and allow flashing.

Navigate to `usb-i2c-rom` and run `make run` to deploy the ROM to an STM32. If debug mode is enabled, you need to keep this window open, otherwise the chip
will fail to print debug messages and crash. If debug mode is disabled, you can safely close the window.

#### Testing the driver

Navigate to `usb-i2c` and run `make insert` to install the driver. When the microcontroller is connected via USB, a new I2C bus will be created.
To work out the device file, run `make logs`.

```
usb_i2c:usbi2c_probe: Registered i2c bus driver at i2c-23
```

In this case, the file is `/dev/i2c-23`.
The driver is configured in fast mode at 400kHz by default, but this can be changed with the ioctl interface described below.

## IOCTL interface

The driver has a simple IOCTL interface which can be used to configure the I2C mode (standard/fast mode) and the clock frequency.

The header is `./usb-i2c/usb-i2c-ioctl.h`.

```c
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include "../usb-i2c/usb-i2c-ioctl.h"

struct configure_i2c_data {
    int frequency;
    bool fastMode;
};

int main() {
    // Open USBI2C adapter (NOT the I2C bus).
    int file = open("/dev/usbi2c-adap2", O_RDWR);
    if(file < 0) {
        printf("Error opening USBI2C: %s\n", strerror(errno));
        return 1;
    }

    // Change to fast mode at 300kHz.
    struct configure_i2c_data data;
    data.frequency = 300;
    data.fastMode = true;

    int res = ioctl(file, IOCTRL_CONFIGURE_USBI2C, &data);
    printf("retval: %d\n", res);
    return 0;
}
```

## Demonstration

In this demo, I use the [Luma.OLED](https://luma-oled.readthedocs.io/en/latest/) library to draw some graphics on an SH1106 display,
connected via the blue pill to my PC.

[Youtube Link](https://www.youtube.com/watch?v=1emYreDPznk)

## Future improvements

- The STM32 has two I2C peripherals, but this project only uses the second one. Potentially support for both could be added.
- The STM32 also works as an I2C slave. We could add I2C slave support to the driver and ROM.
- Right now we implement raw I2C, not SMBUS. Most SMBUS features can be emulated with raw I2C, but not all of them. In the future,
we could add explicit SMBUS support.

Also, one thing to bear in mind - I designed the ROM with no external dependencies so that I could learn how the STM32 registers work/
get a more low level understanding. In the future, it makes sense to reimplement the ROM with the HAL so that it is more readable.

## (Unavoidable?) limitations

### Latency
- The latency when sending I2C transactions is quite high, due to the need to send a USB packet to the STM32, and read a packet back before the transaction can complete.
- This limits the maximum bandwidth, especially when each I2C transaction is small.
- The problem is quite difficult to avoid because we need to read the result of the previous I2C transaction to return to the caller before the next transaction can be started.

A potential solution is to just "assume" the transaction completes successfully, and read the result later/fail a later transaction if there's a problem. However,
this would be confusing to userspace applications.
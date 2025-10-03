#pragma once

// Public header defining the exposed IOCTLs for the USBI2C adapter.

#include <linux/ioctl.h>

#define USBI2C_MAGIC 0x55
#define CONFIGURE_I2C_CMD 0x01

// Data sent in the IOCTL call for configuring the USBI2C bus.
struct configure_i2c_data {
    int frequency;
    bool fastMode;
};

// IOCTL command code for configuring the USBI2C bus.
#define IOCTRL_CONFIGURE_USBI2C _IOW(USBI2C_MAGIC, CONFIGURE_I2C_CMD, struct configure_i2c_data*)
#pragma once

#include "stm32.h"

/// This header file is for a basic STM32 I2C master driver, which carries out I2C transactions based upon
/// data streamed over a USB connection.

// Sets up the master clocks, AFIO configuration and other registers.
// Must be called before any USB bulk transfers begin.
// The APB1 clock is assumed to be 36MHz.
void I2CMaster_Init();

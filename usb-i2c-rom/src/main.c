#include "usb/public.h"
#include "usb_descriptors.h"
#include "stm32.h"
#include "cortex.h"
#include "interrupts.h"
#include "delay.h"
#include "debug.h"
#include "i2c-master.h"

// Entry point for an I2C to USB converter ROM for the STM32 blue pill.

// Sets the PLL clock to 72 MHz, and sets up other clocks so that the USB peripheral functions correctly.
// NB: Assumes the HSE oscillator is 8MHz
void clock_setup() {
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON; // Enable HSE oscillator.
    while((RCC->CR & RCC_CR_HSERDY_MASK) == 0) {
        // Wait for HSE clock to stabilise.
        asm volatile("");
    }

    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_OSC; // Set PLLSCR to external oscillator, which is 8MHz on the blue pill.
    
    // The input of the PLLMUL on the blue pill is 8MHz, so the output is 72MHz with a 9x multiplier
    RCC->CFGR |= RCC_CFGR_PLLMUL_TIMES9;

    RCC->CR |= RCC_CR_PLLON;
    // Wait until clock stabilises.
    while((RCC->CR & RCC_CR_PLLRDY_MASK) == 0) {
        asm volatile("");
    }

    // The PLLVCO is hence 2 * 72MHz = 144MHz.
    // If we set the USB prescaler to `/1.5`, then the USB clock is 48MHz, as required.
    // NB: This is the default value.

    // Change system clock to use the PLL clock
    // This is necessary because the APB1 peripheral clock needs to be at least 8MHz to avoid data overrun/underrun problems. (see ref. manual).
    // Set APB1 prescaler to divide by 2 first to avoid more than 36MHz to APB1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV_2;

    // Setting to 2 wait states. Failing to do this will lead to unpredictable behaviour due to unreliable memory access.
    *FLASH_ACR |= FLASH_ACR_TWO_WAIT_STATES;

    RCC->CFGR |= RCC_CFGR_SWS_PLL;
}


int main() {
    clock_setup();
    delay_init();
    I2CMaster_Init();

    USB_SetDescriptors(&device_descriptor, (usb_configuration_descriptor*) &cfg_descriptor);
    if(!USB_Begin()) {
        dbgprintf("Unable to initialise USB driver\n");
        return 1;
    }
}
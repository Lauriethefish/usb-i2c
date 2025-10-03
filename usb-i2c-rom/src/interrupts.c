#include "interrupts.h"

void (*interruptVector[])(void) __attribute__((used, section (".vectors"))) = {
    // Initial stack pointer position.
    _stacktop,
    handle_reset,
    0,
    handle_hardfault,
    handle_mem_management_fault,
    handle_bus_fault,
    handle_usage_fault,
    0, // Reserved
    0, // Reserved
    0, // Reserved
    0, // Reserved
    0, // SVCall
    0, // Debug Monitor
    0, // Reserved
    0, // PendSV
    systick_elapsed, // SysTick
    0, // WWDG
    0, // PVD
    0, // TAMPER
    0, // RTC
    0, // FLASH
    0, // RCC
    0, // EXTI Line0
    0, // EXTI Line1
    0, // EXTI Line2
    0, // EXTI Line3
    0, // EXTI Line4
    0, // DMA1_Channel1
    0, // DMA1_Channel2
    0, // DMA1_Channel3
    0, // DMA1_Channel4
    0, // DMA1_Channel5
    0, // DMA1_Channel6
    0, // DMA1_Channel7
    0, // ADC1_2
    USB_HP_CAN_TX, // USB High Priority interrupts
    USB_LP_CAN_RX0, // USB low priority interrupts
    0, // CAN_RX1
    0, // CAN_SCE
    0, // EXTI9_5
    0, // TIM1_BRK
    0, // TIM1_UP
    0, // TIM1_TRG_COM
    0, // TIM1_CC
    0, // TIM2
    0, // TIM3
    0, // TIM4
    0, // I2C1_EV
    0, // I2C2_ER
    I2C2_EV, // I2C2_EV
    I2C2_ER, // I2C2_ER
};
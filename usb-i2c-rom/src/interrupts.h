#pragma once

#define USB_HP_CAN_TX_MASK (1 << 19)
#define USB_LP_CAN_RX0_MASK (1 << 20)
// Interrupt indexes 33 and 34, so indexes 1 and 2 in the second NVIC_ISER register.
#define I2C2_EV_MASK (1 << 1)
#define I2C2_ER_MASK (1 << 2)

extern void _stacktop(void); // Address which is 1 byte outside the end of RAM.
void handle_reset(void);
void handle_hardfault(void);
void handle_mem_management_fault(void);
void handle_bus_fault(void);
void handle_usage_fault(void);
void USB_LP_CAN_RX0(void);
void USB_HP_CAN_TX(void);
void systick_elapsed(void);

void I2C2_EV();
void I2C2_ER();
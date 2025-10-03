
#pragma once

#include <stdint.h>

// Definitions for the stm32f103 peripherals.

#define BOOL uint8_t
#define FALSE 0
#define TRUE 1
#define NULL ((void*) 0)

// Structure of each GPIO port
typedef struct _GPIO_t {
    int CRL; // Port configuration register low.
    int CRH; // Port configuration register high.
    int IDR; // Port input data register.
    int ODR; // Port output data register.
    int BSRR; // Port bit set/reset register
    int BRR; // Port bit reset register.
    int LKLR; // Port configuration lock register.
} GPIO_t;

#define GPIO_A ((volatile GPIO_t*) 0x40010800)
#define GPIO_B ((volatile GPIO_t*) 0x40010C00)
#define INPUT_ANALOGUE 0
#define INPUT_FLOATING 1
#define INPUT_PULL 2 // Pullup or pulldown

#define GP_OUTPUT_PUSH_PULL 0
#define GP_OUTPUT_OPEN_DRAIN 1
#define AF_OUTPUT_PUSH_PULL 2
#define AF_OUTPUT_OPEN_DRAIN 3

#define INPUT_MODE 0
#define OUTPUT_MODE_MAX_10_MHZ 1
#define OUTPUT_MODE_MAX_2_MHz 2 
#define OUTPUT_MODE_MAX_50_MHz 3

// Convenience macros for setting the port configuration bits and port mode bits for port `portNum`.
#define CLEAR_PORT_CFG_MASK(portNum) (~(0xF << (portNum * 4)))
#define SET_PORT_CNF_MASK(portNum, CNF) (CNF << (portNum * 4 + 2))
#define SET_PORT_MODE_MASK(portNum, MODE) (MODE << (portNum * 4))

// Clock control registers, used to enable/disable peripherals
typedef struct _RCC_t {
    int CR;
    int CFGR;
    int CIR;
    int APB2RSTR;
    int APB1RSTR;
    int AHBENR;
    int APB2ENR;
    int APB1ENR;
    int BDCR;
    int CSR;
    int AHBRSTR;
    int CFGR2;
} RCC_t;
#define RCC ((volatile RCC_t*) 0x40021000)

#define I2C_CCR_SM_MODE (0 << 15)
#define I2C_CCR_FM_MODE (1 << 15)
#define I2C_CCR_MASTER_MODE_FM (1 << 15)
#define I2C_CCR_MASTER_MODE_SM (0 << 15)
#define I2C_CCR_FM_DUTY_CYCLE_FACTOR_2 (0 << 14)
#define I2C_CCR_FM_DUTY_CYCLE_FACTOR_16_BY_9 (1 << 14)

#define I2C_SR1_SMBALERT_MASK (1 << 15)
#define I2C_SR1_TIMEOUT_MASK (1 << 14)
#define I2C_SR1_PECERR_MASK (1 << 12)
#define I2C_SR1_OVR_MASK (1 << 11)
#define I2C_SR1_AF_MASK (1 << 10)
#define I2C_SR1_ARLO_MASK (1 << 9)
#define I2C_SR1_BERR_MASK (1 << 8)
#define I2C_SR1_TxE_MASK (1 << 7)
#define I2C_SR1_RxNE_MASK (1 << 6)
#define I2C_SR1_STOPF_MASK (1 << 4)
#define I2C_SR1_ADD10_MASK (1 << 3)
#define I2C_SR1_BTF_MASK (1 << 2)
#define I2C_SR1_ADDR_MASK (1 << 1)
#define I2C_SR1_SB_MASK 1

#define I2C_SR2_DUALF_MASK (1 << 7)
#define I2C_SR2_SMBHOST_MASK (1 << 6)
#define I2C_SR2_SMBDEFAULT_MASK (1 << 5)
#define I2C_SR2_GENCALL_MASK (1 << 4)
#define I2C_SR2_TRA_MASK (1 << 2)
#define I2C_SR2_BUSY_MASK (1 << 1)
#define I2C_SR2_MSL_MASK 1

#define I2C_CR1_PE 1
#define I2C_CR1_SWRST (1 << 15)
#define I2C_CR1_STOP (1 << 9)
#define I2C_CR1_START (1 << 8)
#define I2C_CR1_ACK (1 << 10)
#define I2C_CR1_POS (1 << 11)

#define I2C_CR2_ITBUFEN (1 << 10)
#define I2C_CR2_ITEVTEN (1 << 9)
#define I2C_CR2_ITERREN (1 << 8)


// Definitions for various bits within the RCC registers
#define RCC_CR_PLLON (1 << 24) // Use to turn PLL on
#define RCC_CR_PLLRDY_MASK (1 << 25) // Use to check if PLL is ready
#define RCC_CR_HSEON (1 << 16) // Use to turn HSE on
#define RCC_CR_HSERDY_MASK (1 << 17) // Use to check if HSE is ready

// Set in order to change the default USB clock division of 1.5 into a state where we don't divide the clock at all.
#define RCC_CFGR_RCC_CFGR_USBPRE_NOCHANGE (1 << 22)
#define RCC_CFGR_RCC_CFGR_USBPRE_DIV_1_5 0

// PLL multiplication factor.
#define RCC_CFGR_PLLMUL_TIMES2 (0 << 18)
#define RCC_CFGR_PLLMUL_TIMES3 (1 << 18)
#define RCC_CFGR_PLLMUL_TIMES4 (2 << 18)
#define RCC_CFGR_PLLMUL_TIMES5 (3 << 18)
#define RCC_CFGR_PLLMUL_TIMES6 (4 << 18)
#define RCC_CFGR_PLLMUL_TIMES7 (5 << 18)
#define RCC_CFGR_PLLMUL_TIMES8 (6 << 18)
#define RCC_CFGR_PLLMUL_TIMES9 (7 << 18)
#define RCC_CFGR_PLLMUL_TIME10 (8 << 18)
#define RCC_CFGR_PLLMUL_TIMES11 (9 << 18)
#define RCC_CFGR_PLLMUL_TIMES12 (10 << 18)
#define RCC_CFGR_PLLMUL_TIMES13 (11 << 18)
#define RCC_CFGR_PLLMUL_TIMES14 (12 << 18)
#define RCC_CFGR_PLLMUL_TIMES15 (13 << 18)
#define RCC_CFGR_PLLMUL_TIMES16 (14 << 18)

// Source for the PLL input clock.
#define RCC_CFGR_PLLSRC_HSI_DIV_2 (0 << 16)
#define RCC_CFGR_PLLSRC_HSE_OSC (1 << 16)

#define RCC_CFGR_PPRE1_NO_DIV (0b000 << 8)
#define RCC_CFGR_PPRE1_DIV_2 (0b100 << 8)
#define RCC_CFGR_PPRE1_DIV_4 (0b101 << 8)
#define RCC_CFGR_PPRE1_DIV_8 (0b110 << 8)
#define RCC_CFGR_PPRE1_DIV_16 (0b111 << 8)

#define RCC_APB1ENR_USBEN (1 << 23)
#define RCC_APB1ENR_I2C2EN (1 << 22)
#define RCC_APB1ENR_I2C1EN (1 << 21)

#define RCC_APB1RSTR_I2C2RST (1 << 22)

// Masks for enabling GPIO perhiperals
#define RCC_APB2ENR_IOPAEN (1 << 2)
#define RCC_APB2ENR_IOPBEN (1 << 3)
#define RCC_APB2ENR_IOPCEN (1 << 4)
#define RCC_APB2ENR_IOPDEN (1 << 5)
#define RCC_APB2ENR_IOPEEN (1 << 6)
#define RCC_APB2ENR_IOPFEN (1 << 7)
#define RCC_APB2ENR_IOPGEN (1 << 8)

#define RCC_APB2ENR_USART1EN (1 << 14)


// OR in order to change the system clock source. NB: The clock must have stabilised first!
#define RCC_CFGR_SWS_HSI 0 
#define RCC_CFGR_SWS_HSE 1
#define RCC_CFGR_SWS_PLL 2 

typedef struct TIM_t {
    int CR1;
    int CR2;
    int SMCR;
    int DIER;
    int SR;
    int EGR;
    int CCMR1;
    int CCMR2;
    int CCER;
    int CNT;
    int PSC;
    int ARR;
    int _res1;
    int CCR1;
    int CCR2;
    int CCR3;
    int CCR4;
    int _res2;
    int DCR;
    int DMAR;
} TIM_t;

#define TIM2 ((volatile TIM_x*) 0x40000000)

typedef struct _USART_t {
    int SR; // Status register
    int DR; // Data register
    int BRR; // Baud rate register
    int CR1; // Control register 1
    int CR2; // Control register 2
    int CR3; // Control register 3
    int GTPR; // Guard time and prescaler register
} USART_t;

#define USART_CR1_UE (1 << 13) // USART enable
#define USART_CR1_M_8_BITS 0 // 8 bit word length is the default
#define USART_CR1_M_9_BITS (1 << 12)

#define USART_CR1_TCIE (1 << 6) // Transmission complete interrupt enable
#define USART_CR1_TE (1 << 3) // Transmitter enable.
#define USART_CR1_RE (1 << 2)

#define USART1 ((volatile USART_t*) 0x40013800)

typedef struct _I2C_t {
    int CR1;
    int CR2;
    int OAR1;
    int OAR2;
    int DR;
    int SR1;
    int SR2;
    int CCR;
    int TRISE;
} I2C_t;

#define I2C1 ((volatile I2C_t*) 0x40005400)
#define I2C2 ((volatile I2C_t*) 0x40005800)

// USB full-speed peripheral.
typedef struct USB {
    int EPR[8]; // 8 Endpoint registers: each represents an endpoint pair, with transmission and reception of data.
    char _res[16 * 2]; // 32 bytes reserved
    int CNTR;
    int ISTR;
    int FNR;
    int DADDR;
    int BTABLE;
} USB;

// Represents the value of the STAT_TX or STAT_RX bits in the EPR
typedef enum _USB_STAT {
    UsbStatDisabled = 0,
    UsbStatStall = 1,
    UsbStatNak = 2,
    UsbStatValid = 3
} USB_STAT;

#define USB ((volatile USB*) 0x40005C00)

// Interrupt masks, to turn USB interrupts on/off
#define USB_CNTR_CTRM (1 << 15)
#define USB_CNTR_PMAOVRM (1 << 14)
#define USB_CNTR_ERRM (1 << 13)
#define USB_CNTR_WKUPM (1 << 12)
#define USB_CNTR_SUSPM (1 << 11)
#define USB_CNTR_RESETM (1 << 10)
#define USB_CNTR_SOFM (1 << 9)
#define USB_CNTR_ESOFM (1 << 8)

#define USB_CNTR_RESUME (1 << 4) // Send resume signal to host
#define USB_CNTR_FSUSP (1 << 3) // Force suspend
#define USB_CNTR_LP_MODE (1 << 2) // Low-power mode 
#define USB_CNTR_PDWN (1 << 1) // Powerdown
#define USB_CNTR_FRES 1 // Force USB reset

#define USB_ISTR_CTR (1 << 15)
#define USB_ISTR_PMA_OVR (1 << 14)
#define USB_ISTR_ERR (1 << 13)
#define USB_ISTR_WAKEUP (1 << 12)
#define USB_ISTR_SUSP (1 << 11)
#define USB_ISTR_RESET (1 << 10)
#define USB_ISTR_SOF (1 << 9)
#define USB_ISTR_ESOF (1 << 8)
#define USB_ISTR_DIR (1 << 4) // If dir is 0, transaction is writing from device to host. Else from host to device.
#define USB_ISTR_EP_ID_MASK 0xF

#define USB_DADDR_MASK 0x7F
#define USB_DADDR_EF (1 << 7)

#define USB_EPR_CTR_RX (1 << 15) // Can read or `write 0` to clear. Indicates write/setup successfully completed
#define USB_EPR_DTOG_RX (1 << 14) // Toggles the data toggle

// Creates a mask to change STAT_RX from `prevStat` to `newStat`
// There is no way to check the current value, so code must keep track of it.
// This macro can be used to change from one value to another.
#define USB_EPR_STAT_RX_MASK(prevStat, newStat) ((prevStat ^ newStat) << 12)
// Use to get the SETUP bit, to determine if this is a setup transaction.
#define USB_EPR_SETUP_MASK (1 << 11)

// Use to set the endpoint type
#define USB_EPR_EP_TYPE_BULK (0 << 9)
#define USB_EPR_EP_TYPE_CONTROL (1 << 9)
#define USB_EPR_EP_TYPE_ISO (2 << 9)
#define USB_EPR_EP_TYPE_INTERRUPT (3 << 9)

#define USB_EPR_EP_KIND (1 << 8)
#define USB_EPR_CTR_TX (1 << 7)
#define USB_EPR_DTOG_TX (1 << 6)

#define USB_EPR_STAT_TX_MASK(prevStat, newStat) ((prevStat ^ newStat) << 4)
// Use to get endpoint address.
#define USB_EPR_EA_MASK 0xF

#define USB_COUNT_RX_BLSIZE (1 << 15)
// Sets NUM_BLOCK to the specified value, using an OR operation.
#define USB_COUNT_RX_NUM_BLOCK(value) (value << 10)

#define USB_COUNT_RX_NUM_BYTES_MASK 0x1FF

// Buffer descriptor table.
typedef struct BF_DESC_TABLE_ENTRY {
    int TX_ADDR; // Transmission buffer address
    int TX_CNT; // Transmission buffer count
    int RX_ADDR; // Reception buffer address
    int RX_COUNT; // Reception byte count
} BF_DESC_TABLE_ENTRY;

typedef struct BF_DESC_TABLE {
    BF_DESC_TABLE_ENTRY entries[8]; // 8 supported endpoints
} BF_DESC_TABLE;

#define FLASH_ACR ((volatile int*) 0x40022000)

#define FLASH_ACR_ZERO_WAIT_STATE 0
#define FLASH_ACR_ONE_WAIT_STATE 1
#define FLASH_ACR_TWO_WAIT_STATES 2

// SRAM used to hold USB buffer descriptors and the packets contents themselves.
// This ranges up to 0x400063FF, however this is only actually 512 bytes of memory.
// Specifically, it is 256 two-byte-words. Each word can only be accessed using a 32 bit word load or store.
// The upper two bytes get ignored.
#define USB_SRAM 0x40006000
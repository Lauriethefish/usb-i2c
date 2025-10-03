#pragma once

// Definitions related to built-in functions of the ARM Cortex M3 core.

// Represents the registers for the SysTick counter.
typedef struct SysTick_t {
    int CSR; // Control and status register.
    int RVR; // Reset value
    int CVR; // Current value
    int CALIB; // Calibration value.
} SysTick_t;

#define SysTick ((volatile SysTick_t*) 0xE000E010)

// Masks for the bits in the SysTick CSR.  
#define SYSTICK_COUNTFLAG_MASK (1 << 16)
#define SYSTICK_CLKSOURCE_EXTERNAL (1 << 2)
#define SYSTICK_TICKINT_ENA (1 << 1)
#define SYSTICK_ENA 1

typedef struct _SysCtrlBlock {
    int ACTLR; // Aux. control register
    char _reserved[3316];
    int CPUID; // CPUID base register
    int ICSR; // Interrupt control and state register
    int VTOR; // Vector table offset register
    int AIRCR; // Application interrupt and reset control register
    int SCR; // System control register
    int CCR; // Configuration and control register
    int SHPR[3]; // System handler priority register 1-3
    int SHCRS; // System handler control and state register
    int CFSR; // Configurable fault status register
    int HFSR; // HardFault status register
    int _reserved2;
    int MMAR; // MemManage fault address register
    int BFAR; // BusFault address register
    int AFSR; // Auxillary fault status register
} SysCtrlBlock;


// System handler priority registers

#define SysCtrlBlock ((volatile SysCtrlBlock*) 0xE000E008)

#define USGFAULTENA (1 << 18)
#define BUSFAULTENA (1 << 17)
#define MEMFAULTENA (1 << 16)

// Interrupts 0 through to 31
#define NVIC_ISER0 ((volatile unsigned int*) 0xE000E100)

// Interrupts 32 through to 63
#define NVIC_ISER1 ((volatile unsigned int*) 0xE000E104)

#define NVIC_IPR ((volatile char*) 0xE000E400)
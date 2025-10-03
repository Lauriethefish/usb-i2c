#include "debug.h"
#include "interrupts.h"
#include "cortex.h"

// Initialised globals start/end position in flash
extern char _data_start;
extern char _data_end;
// Initialised globals load position in RAM.
extern const char _data_load;

// 0-initialised globals start/end in RAM. NB: No corresponding area in flash, as all we need to do is write a 0.
extern char _bss_start;
extern char _bss_end;

extern int main(void);

void infinite_loop(void) {
a:
    goto a;
}

// Interrupt handler for system reset.
void handle_reset(void) {
    // Load data (initialised globals) into RAM
    const char* src = &_data_load;
    volatile char* dst = &_data_start;
    char* dst_end = &_data_end;
    while(dst < dst_end) {
        *dst = *src;
        src++;
        dst++;
    }

    // Zero out the bss region
    volatile char* pos = &_bss_start;
    char* end = &_bss_end;
    while(pos < end) {
        *pos = 0;
        pos++;
    }

    // Enable MemManage faults, UsageFaults and BusFaults.
    SysCtrlBlock->SHCRS |= USGFAULTENA | BUSFAULTENA | MEMFAULTENA;

    int ret = main();
    printf("Main retval: %d\n", ret);

    infinite_loop();
}


void handle_hardfault(void) {
    allow_blocking_immediately();
    printf("-------- Hardfault --------\nHFSR: %w\nCFSR: %w\n", SysCtrlBlock->HFSR, SysCtrlBlock->CFSR);
    infinite_loop();
}

void handle_mem_management_fault(void) {
    allow_blocking_immediately();
    printf("-------- MemManage fault --------\nCFSR: %w\nMMAR: %w\n", SysCtrlBlock->CFSR, SysCtrlBlock->MMAR);
    infinite_loop();
}

void handle_bus_fault(void) {
    allow_blocking_immediately();
    printf("-------- BusFault --------\nCFSR: %w\nBFAR: %w\n", SysCtrlBlock->CFSR, SysCtrlBlock->BFAR);
    infinite_loop();
}

void handle_usage_fault(void) {
    allow_blocking_immediately();
    printf("-------- UsageFault --------\nCFSR: %w\n", SysCtrlBlock->CFSR);
    infinite_loop();
}
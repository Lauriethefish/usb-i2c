#include "delay.h"
#include "cortex.h"
#include "debug.h"
//#include "main.h"

// Currently using the maximum possible value for the initial systick counter, to allow the largest 
// possible delay with our (rather naive) implementation.
#define SYSTICK_COUNT_DOWN_FROM 0x00FFFFFF

BOOL initCalled = FALSE;

void systick_elapsed(void) {
    // Inform debug print that systick has reached zero. This might mean it's time to write the buffer to stdout.
    debug_on_systick_zero();
}

// Calculates the number of systicks of delay for the given number of microseconds.
int calculate_systicks_us(int us) {
    // Systick is configured to run at 9MHz (see delay_init)
    // So each microsecond it counts down by 9
    return 9 * us;
}

// Waits for the specified number of systicks.
void delay_by_systicks(int initial_ticks, int num_ticks) {
    int stop_at;
    if(initial_ticks < num_ticks) { // Case where counter wraps around
        num_ticks -= initial_ticks;

        // This is now how many ticks we must wait past wraparound
        stop_at = (SYSTICK_COUNT_DOWN_FROM + 1) - num_ticks;

        // First wait until wraparound, where COUNTFLAG is set
        while((SysTick->CSR & SYSTICK_COUNTFLAG_MASK) == 0) {}
    }   else    {
        stop_at = initial_ticks - num_ticks;
    }

    while(SysTick->CVR > stop_at) {}
}


void delay_init() {
    if(initCalled) {
        return;
    }
    initCalled = TRUE;

    // Set reset value for SysTick to the max possible - allow the largest possible delay
    SysTick->RVR = SYSTICK_COUNT_DOWN_FROM;

    // Reduce the priority of the systick interrupt to the minimum possible.
    // This avoids systick interrupting I2C or USB routines, which can cause I2C transfers to stall.

    // Enable SysTick and the elapsed interrupt.
    // Set PRIGROUP to 3
    *AIRCR = (0b011 << 8) | 0x5FA << 16;
    *SHPR_3 = 255 << 24;

    // For some reason, even having systick enabled seems to prevent I2C from working properly.
    // This is despite correctly configuring the priority to be lower than I2C and USB.
    // I have confirmed that neither the I2C nor USB interrupts are being interrupted by systick.
	SysTick->CSR = SYSTICK_ENA;

    // Default value for bit 2, so count is sourced from AHB/8
    // Our system clock is 72MHz, so the systick frequency is 9MHz

    // SysTick now counts from 0x00FFFFFF to 0, then repeats forever...
}

void delay_us(int us) {
    // Record SysTick as ASAP
    int initial_ticks = SysTick->CVR;
    int num_ticks = calculate_systicks_us(us);

    delay_by_systicks(initial_ticks, num_ticks);
}

// Gets the current time
int get_time_ref() {
    return SysTick->CVR;
}

// Waits until it is at least the specified number of microseconds after time_ref
void delay_us_remaining(int time_ref, int us) {
    delay_by_systicks(time_ref, calculate_systicks_us(us));
}

int get_elapsed_us(int time_ref) {
    int ticks = (SysTick->CVR > time_ref) ? (time_ref + (SYSTICK_COUNT_DOWN_FROM + 1) - SysTick->CVR) : (time_ref - SysTick->CVR);

    return ticks / 9;
}

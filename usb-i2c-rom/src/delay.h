#pragma once

// Definitions for a simple delay module, based upon the ARM Cortex M3 SysTick counter.

#include "stm32.h"

// Initialises the delay system.
// This sets up the SysTick counter and must be called before any of the other functions.
// NB: The AHB clock must be 72MHz such that the SysTick counter can take AHB/8, i.e. a 9MHz frequency.
void delay_init();

void delay_by_systicks(int initial_ticks, int num_ticks);

// Produces a delay by the specified number of microseconds
void delay_us(int us);

// Gets a token representing. the current time.
int get_time_ref();

// Waits until it is at least the specified number of microseconds after time_ref
void delay_us_remaining(int time_ref, int us);

// Gets the number of elapsed microseconds since the given time.
int get_elapsed_us(int time_ref);

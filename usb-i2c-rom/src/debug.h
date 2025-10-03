#pragma once

#include <stdint.h>
#include <stdarg.h>
#include "stm32.h"

// Basic module for printing debug information using semihosting.

#define MAX_PRINTF_LEN 2048

#define BKPT() asm volatile("bkpt")

#ifdef DEBUG
#define dbgprintf(...) printf(__VA_ARGS__)
#endif

#ifndef DEBUG
#define dbgprintf(...)
#endif

// Prints the given null-terminated buffer to stdout.
void print(char* buffer);

// Prevents printf from blocking until systick elapses twice.
// Call if entering a time-critical section (or if we're about to write a whole bunch of data and want to minimise the number of write requests,
// which will make the write more efficient.)
// NB: At most `MAX_PRINTF_LEN` bytes can be buffered until leaving the time-critical section.
// Additional data is lost.
void prevent_blocking();

// Allows printf to block even if systick hasn't elapsed twice yet (see `prevent_blocking`)
// This will block if there is pending data
void allow_blocking_immediately();

// Called by the systick interrupt handler (in delay.c) whenever systick elapses.
void debug_on_systick_zero();

void disable_interrupts();

void enable_interrupts();


// Well-understood `printf` function. Maximum output length is MAX_PRINTF_LEN. Any text beyond this gets truncated.
// Use a combination of `format` and `print` if you need to write more, or call `printf` multiple times.
// `fmtString` must be NULL-terminated.
// If printing debug messages, use the `dbgprintf` macro instead, which skips the debug messages in a release build.
void printf(const char* fmtString, ...);

// Prints a message and enters an infinite loop. Disables all interrupts.
void abort();

// Supports `printf` style formatting with the `d` and `x` specifiers, as well as `%w`, which prints a full
// 32 bit word in binary with underscore separated bytes.
// If the output won't fit in `buffer`, it will be truncated. (there will always be a null byte at the end, either way)
// Returns the number of bytes written, including the NULL byte.
int format(char* buffer, int bufferLength, const char* fmtString, va_list argList);

#include "debug.h"
#include "stm32.h"
#include <stdarg.h>

// Semihosting is only available on debug builds.
#ifdef DEBUG

int semihost_request(int reason, void* arg) {
    int value;
    __asm volatile (
        "mov r0, %[rsn] \n"
        "mov r1, %[arg] \n"
        "bkpt 0xAB      \n"
        "mov %[val], r0 \n"

        : [val] "=r" (value)  
        : [rsn] "r" (reason), [arg] "r" (arg)
        : "r0", "r1", "r2", "r3", "ip", "lr", "memory", "cc"
    );

  return value;
}

#endif

#ifndef DEBUG

int semihost_request(int reason, void* arg) {
    return 0;
}

#endif

int sys_open(char* filename, int mode, int namelength) {
    int params[3];
    params[0] = (int) filename;
    params[1] = mode;
    params[2] = namelength;
 
    // SYS_OPEN request
    return semihost_request(0x01, &params[0]);
}

void sys_write(int file, char* data, int length) {
    int params[3];
    params[0] = file;
    params[1] = (int) data;
    params[2] = length;

    // SYS_WRITE request
    semihost_request(0x05, &params[0]);
}

int stdout_handle = -1;
void print(char* data) {
    if(stdout_handle == -1) {
        stdout_handle = sys_open(":tt", 4, 3);
    }

    char* pos = data;
    while(*pos) {
        *pos++;
    }
    int length = pos - data;

    sys_write(stdout_handle, data, length);
}

int numSysTickElapsesUntilBlock = 0;

// Prevents printf from blocking until systick elapses twice.
// Call if entering a time-critical section.
void prevent_blocking() {
    // Need at least 2 times to ensure a 1 second delay.
    numSysTickElapsesUntilBlock = 2;
}

char printfBuffer[MAX_PRINTF_LEN];
int bufferPos = 0; // Initially writing to the beginning of the buffer

void printfToBuffer(const char* fmtString, va_list argList) {
    // Create a `view` of the remaining space within the buffer
    int bufferLength = MAX_PRINTF_LEN - bufferPos;
    char* buffer = &printfBuffer[bufferPos];

    int lengthInclNull = format(buffer, bufferLength, fmtString, argList);
    bufferPos += lengthInclNull;
    bufferPos--; // NULL byte should be overwritten the next time we write more data.
}

void printfFlushBuffer() {
    // Print any pending data, assuming there's at least one byte.
    if(bufferPos) {
        print(printfBuffer);
        bufferPos = 0;
    }
}

void allow_blocking_immediately() {
    numSysTickElapsesUntilBlock = 0;
    printfFlushBuffer();
}

// Called by the systick interrupt handler (in delay.c) whenever systick elapses.
void debug_on_systick_zero() {
    if(numSysTickElapsesUntilBlock) {
        numSysTickElapsesUntilBlock--;
    }   else    {
        printfFlushBuffer();
    }
}

void printf(const char* fmtString, ...) {
    va_list argList;

    // Format arguments
    va_start(argList, fmtString);
    printfToBuffer(fmtString, argList);
    va_end(argList);
    // Write to stdout with semihost request
    if(numSysTickElapsesUntilBlock == 0) {
        printfFlushBuffer();
    }
}

void disable_interrupts() {
    asm volatile("CPSID I");

}

void enable_interrupts() {
    asm volatile("CPSIE I");
}

void abort() {

    printfFlushBuffer();
    print("System abort triggered\n");
    while(1) { asm volatile(""); }
}

// Formats an integer `n` in the given base, and puts the result into `outputBuffer`.
// `endOfBuffer` gives the first byte which is outside the bounds of the buffer.
// `base` must be between 2 and 16 inclusive.
// `minLength` gives the minimum length of the output number. (any leftover length will be padded with zeroes.)
// Setting `minLength` to 0 signifies no minimum length.
// Returns the first byte in `outputBuffer` after the number was written.
const char* numerals = "0123456789ABCDEF";
char* fmt_in_base(char* outputBuffer, char* endOfBuffer, int n, int base, int minLength) {
    // Temporarily holds the number while we figure out how many leading zeroes we need 
    char tmpBuffer[32];
    char* endOfTmp = &tmpBuffer[sizeof(tmpBuffer)];
    char* ptr = endOfTmp; // Start with least significant digit.

    // Copy the digits of the number to the output, stopping once there are no more digits left.
    while(n) {
        ptr--;
        *ptr = *(numerals + (n % base));

        n /= base;
    }
    // Ptr points to the most significant digit.
    int numDigits = endOfTmp - ptr;
    // Write padding digits.
    while(numDigits < minLength) {
        *outputBuffer = '0';
        outputBuffer++;
        numDigits++;
        if(outputBuffer == endOfBuffer) return outputBuffer;
    }

    // Write non-padding digits
    while(ptr < endOfTmp) {
        *outputBuffer = *ptr;
        outputBuffer++;
        ptr++;

        if(outputBuffer == endOfBuffer) return outputBuffer;
    }

    return outputBuffer;
}

// Formats a full word in base 2, placing an underscore between every 8 
char* fmt_word_in_base_2(char* outputBuffer, char* endOfBuffer, int word) {
    int currentBit = 32;
    while(currentBit) {
        // Every 8 bits, add a separator
        if((currentBit & 0b111) == 0 && currentBit != 32) {
            *outputBuffer = '_';
            outputBuffer++;

            if(outputBuffer == endOfBuffer) {
                return endOfBuffer;
            }
        }
        currentBit--;

        // Write a 1 or a 0 depending on the current bit.
        if(word & (1 << currentBit)) {
            *outputBuffer = '1';
        }   else    {
            *outputBuffer = '0';
        }
        outputBuffer++;

        if(outputBuffer == endOfBuffer) {
            return endOfBuffer;
        }
    }

    return outputBuffer;
}

int format(char* buffer, int bufferLength, const char* fmtString, va_list argList) {
    char* bufferStart = buffer;
    char* endOfBuffer = buffer + bufferLength - 1; // Last possible location of the NULL byte within the buffer.

    // Parse for fmtString until we get to a NULL byte
    const char* currChar = fmtString;
    int currentArg = 0;

    while(*currChar && (buffer < endOfBuffer)) {
        if(*currChar == '%') {
            // Parse format specifier.
            currChar++;
            if(!*currChar) break;

            // NB: we only support the width section and the type section

            int minLength;
            // The width must be between 0 and 9. Multiple digits aren't supported.
            if(*currChar >= '0' && *currChar <= '9') {
                minLength = *currChar - '0';

                currChar++;
                if(!*currChar) break;
            }   else    {
                minLength = 1; // Need at least `1` digit, else a value of zero just gives an empty string.
            }

            int val = va_arg(argList, int);

            // Determine the base using the type specifier.
            int base;
            switch(*currChar) {
                case 'x':
                case 'X':
                    base = 16;
                    break;
                case 'w':
                    // Custom type specifier which is a full binary 32 bit word.
                    // This is printed using underscores between every 8 bits.
                    buffer = fmt_word_in_base_2(buffer, endOfBuffer, val);
                    currChar++;
                    continue;
                case 'd':
                    base = 10;
                    break;
                default:
                    print("Invalid fmt string\n");
                    goto exit;
            }

            buffer = fmt_in_base(buffer, endOfBuffer, val, base, minLength);
            currChar++;
        }   else    {
            // Add char directly to output
            *buffer = *currChar;
            buffer++;
            currChar++;
        }
    }

exit:
    *buffer = '\0';
    buffer++;
    return buffer - bufferStart;
}
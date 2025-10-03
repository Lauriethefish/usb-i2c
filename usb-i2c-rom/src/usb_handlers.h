#pragma once

/// This header contains handlers for the USB bulk in/bulk out events in our USB to I2C converter.
/// Data read from bulk out (or to be sent to bulk in) is stored in a double buffer, ensuring that there
/// are always more bytes for I2C to send. (Since USB full-speed is faster than I2C)

#include "stm32.h"

// Writes the specified data to the bulk in buffer.
// Returns TRUE if the data could fit.
// If both buffers are full, returns FALSE.
// The buffer is flushed once it reaches a full packet.
BOOL USBHandlers_WriteToBulkInBuffer(char* buffer, int length);

// Ensures that any remaining data in the buffer is sent to bulk in.
// Will also send a zero length packet if the transfer size was a multiple of the packet size.
// If the most recently read packet from bulk out WASN'T a short packet, calling this function ensures that the rest of the data
// in the transfer is discarded.
// `USBHandlers_StartBulkOutTransfer` will be called once the next transfer starts.
void USBHandlers_CompleteTransfer();

// Reads data from the bulk out buffer into `readInto`.
// Returns `0` if successful.
// Returns `-1` if insufficient data was available (i.e. the host was too slow)
// Returns `1` if at the end of the current bulk out transfer (i.e. the I2C transaction is complete). 
int USBHandlers_ReadFromBulkOutBuffer(char* readInto, int length);

/// CALLBACKS
// This function is called whenever a new transfer begins on bulk out.
// (this signals that an I2C transfer should be started.)
void USBHandlers_StartBulkOutTransfer();

// Called when the host sends a control packet to select the I2C mode (standard vs fast mode)
// and the bus frequency.
void USBHandlers_ConfigureI2CMode(BOOL fastMode, int frequency);
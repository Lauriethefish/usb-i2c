#include "usb_handlers.h"
#include "usb/public.h"
#include "debug.h"
#include "delay.h"
#include "../../shared/protocol.h"

// Buffer that the I2C module reads from.
char bulkOutPublicBuf[MAX_PACKET_SIZE];
int publicBufNextByte; // Next byte to extract from buffer
int bulkOutPublicBufSize; // Total bytes in buffer

//int firstHalfBytesRead;

char bulkInPublicBuf[MAX_PACKET_SIZE];
int bulkInPublicBufSize; // Current bytes in buffer, automatically flushed when full.

volatile BOOL bulkInPrivateBufFull = FALSE;
volatile BOOL endOfTransfer = TRUE; // No transfer ongoing

BOOL bulkOutGotShortPacket = TRUE; // End of previous transfer has been received.

// We read the bulk out buffer in two halves to avoid copying too much data when trying to write
// one byte to I2C - we only have a limited number of clock cycles on each "transmit buffer empty" interrupt.
//volatile BOOL bulkOutReadFirstHalf = FALSE;

// Attempts to fill the bulk in public buffer with `length` bytes from `buffer`.
// Returns number of bytes successfully filled.
int USBHandlers_FillBulkIn(char* buffer, int length) {
    int spaceLeft = MAX_PACKET_SIZE - bulkInPublicBufSize;
    int bytesToRead = length < spaceLeft ? length : spaceLeft;

    for(int i = 0; i < bytesToRead; i++) {
        bulkInPublicBuf[bulkInPublicBufSize] = buffer[i];
    }
    bulkInPublicBufSize += bytesToRead;
    return bytesToRead;
}

BOOL USBHandlers_CopyBulkInBufferToUSB() {
    if(bulkInPrivateBufFull) {
        return FALSE;
    }

    if(!USB_WriteTransmitBuffer(1, &bulkInPublicBuf[0], bulkInPublicBufSize)){
        // Should never happen as endpoint `1` exists and the buffer is the right size.
        dbgprintf("USB_WriteTransmitBuffer failed\n");
        abort();
    }

    bulkInPrivateBufFull = TRUE;
    bulkInPublicBufSize = 0;
}

// Writes the specified data to the bulk in buffer.
// Returns TRUE if the data could fit.
// If both buffers are full, returns FALSE.
BOOL USBHandlers_WriteToBulkInBuffer(char* buffer, int length) {
    int bytesRead = USBHandlers_FillBulkIn(buffer, length);
    length -= bytesRead;
    buffer += bytesRead;

    if(bulkInPublicBufSize == MAX_PACKET_SIZE) {
        if(!USBHandlers_CopyBulkInBufferToUSB()) {
            // Host too slow - couldn't flush the buffer since they haven't yet read the previous packet.
            return FALSE;
        }
    }

    if(!length) {
        return TRUE;
    }

    // Try to copy the rest of the bytes using the (now empty) buffer.
    bytesRead = USBHandlers_FillBulkIn(buffer, length);
    return bytesRead == length;
}

// Allows the host to read the data in a bulk in buffer, even if it isn't full yet.
void USBHandlers_CompleteTransfer() {
    endOfTransfer = TRUE;
    // Try to flush the bulk in buffer immediately.
    USBHandlers_CopyBulkInBufferToUSB();
    // if this isn't possible due to pending data to transmit, the transfer complete interrupt
    // should do the flushing.

    // If bulk out transfer has already completed (due to reception of a short packet),
    // ready it for the next transfer.
    // OR: if we haven't got a short packet yet, we need to wait until we receive one.
    // Either way, we're ready for more data immediately.
    USB_BulkOutReadyForMoreData(1);
}

// Copies the bulk out private buffer (in the USB peripheral.) to the public buffer.
// If `firstHalf` is TRUE, the first half of the buffer is copied, and the same applies for `secondHalf`.
// Both can be TRUE to copy the whole buffer.
void USBHandlers_CopyBulkOutBuffers(/*BOOL firstHalf, BOOL secondHalf*/) {
    // Read the first half of the buffer
    /*
    if(firstHalf) {
        if(!USB_ReadReceiveBuffer(1, &bulkOutPublicBuf[0], 32, &firstHalfBytesRead, 0)) {
            // This error should never happen - endpoint `1` should exist.
            dbgprintf("USB_ReadReceiveBuffer failed\n");
            abort();
        }
        //dbgprintf("Copied first half: %d at %d\n", firstHalfBytesRead, publicBufNextByte);
    }
    if(secondHalf) {
        int numRead;
        if(!USB_ReadReceiveBuffer(1, &bulkOutPublicBuf[32], 32, &numRead, 32)) {
            // This error should never happen - endpoint `1` should exist.
            dbgprintf("USB_ReadReceiveBuffer failed\n");
            abort();
        }
        bulkOutPublicBufSize = firstHalfBytesRead + numRead;

        //dbgprintf("Copied 2nd half, total: %d at %d\n", bulkOutPublicBufSize, publicBufNextByte);

        // Reset buffer position to zero if we've finished reading the buffer by reading the second half.
        publicBufNextByte = 0;

        // Only `ACK` additional data if we're continuing the same transfer.
        // Otherwise, we should wait for the I2C master to complete the transfer before allowing the next one.
        if(bulkOutPublicBufSize == MAX_PACKET_SIZE) {
            USB_BulkOutReadyForMoreData(1);
        }
    }*/

    int numRead;
    if(!USB_ReadReceiveBuffer(1, &bulkOutPublicBuf[0], 64, &numRead, 0)) {
        // This error should never happen - endpoint `1` should exist.
        dbgprintf("USB_ReadReceiveBuffer failed\n");
        abort();
    }
    bulkOutPublicBufSize = numRead;

    //dbgprintf("Copied 2nd half, total: %d at %d\n", bulkOutPublicBufSize, publicBufNextByte);

    // Reset buffer position to zero if we've finished reading the buffer by reading the second half.
    publicBufNextByte = 0;

    // Only `ACK` additional data if we're continuing the same transfer.
    // Otherwise, we should wait for the I2C master to complete the transfer before allowing the next one.
    if(bulkOutPublicBufSize == MAX_PACKET_SIZE) {
        USB_BulkOutReadyForMoreData(1);
    }

}

// Reads at most `length` bytes into `readInto` from the bulk out public buffer.
// Returns the number of bytes read.
int USBHandlers_FillFromBulkOut(char* readInto, int length) {
    // Read all that we can from the public buf.
    int publicBufRemBytes = bulkOutPublicBufSize - publicBufNextByte;
    int toRead = publicBufRemBytes < length ? publicBufRemBytes : length;

    char* end = readInto + toRead;
    while(readInto < end) {
        *readInto = bulkOutPublicBuf[publicBufNextByte];
        readInto++;
        publicBufNextByte++;
    }

    return toRead;
}

// Reads data from the bulk out buffer into `readInto`.
// Returns `0` if successful.
// Returns `-1` if insufficient data was available (i.e. the host was too slow)
// Returns `1` if at the end of the current bulk out transfer (i.e. the I2C transaction is complete). 
int USBHandlers_ReadFromBulkOutBuffer(char* readInto, int length) {
    int bytesRead = USBHandlers_FillFromBulkOut(readInto, length);
    readInto += bytesRead;
    length -= bytesRead;

    // If the first half of the public buffer is empty, we can copy the first half from the private buffer immediately
    // Note that in the case of a short packet, it doesn't matter that the condition might never be met, because
    // there is no subsequent packet.
    //if(!bulkOutReadFirstHalf && publicBufNextByte >= 32) {
    //    bulkOutReadFirstHalf = TRUE;
    //    USBHandlers_CopyBulkOutBuffers(TRUE, FALSE);
    //}

    if(!length) {
        return 0;
    }

    // If more bytes to read but the public buffer has a short packet, this indicates the end of the transfer
    if(bulkOutPublicBufSize < MAX_PACKET_SIZE) {
        return 1;
    }

    // If the public buffer is empty, swap it with the private buffer
    if(publicBufNextByte == bulkOutPublicBufSize) {
        USBHandlers_CopyBulkOutBuffers(/*FALSE, TRUE*/);
    }

    int time_ref = get_time_ref();

    // ...and try again
    bytesRead = USBHandlers_FillFromBulkOut(readInto, length);
    length -= bytesRead;
    // *Still* more data to read - looks like the host didn't provide us with data fast enough
    if(length) {
        return -1;
    }

    //dbgprintf("Time taken: %d\n", (time_ref - get_time_ref()) << 3);

    return 0;
}

/// USB driver event handlers.

// Fired when the USB device has its configuration selected.
// (This implementation only supports a single configuration)
void USB_HandleConfigurationSelected() {
    dbgprintf("USB_HandleConfigurationSelected\n");
}

CONTROL_TRANSFER_TYPE USB_HandleSetupTransaction(int endpointNumber, setup_packet setupPacket) {
    dbgprintf("setup packet. bmtype: %w, b_request: %w\n", setupPacket.bmRequestType, setupPacket.bRequest);
    if(setupPacket.bmRequestType == I2C_MODE_BM_REQUEST_TYPE
        && setupPacket.bRequest == I2C_MODE_B_REQUEST) {
        USB_ResetBulkEndpoint(1);

        // MSB has request type
        dbgprintf("Setting up i2c\n");
        BOOL isFastMode = setupPacket.wValue & 1;
        int frequency = setupPacket.wValue >> 1;

        // Reset all transaction status
        bulkOutPublicBufSize = 0;
        bulkInPublicBufSize = 0;
        bulkOutGotShortPacket = TRUE;
        endOfTransfer = TRUE;
        bulkInPrivateBufFull = FALSE;
        //bulkOutReadFirstHalf = FALSE;

        USBHandlers_ConfigureI2CMode(isFastMode, frequency);

        return ControlTransferTypeNoData;
    }   else    {
        return ControlTransferTypeSendDataError; // No additional control transfers supported beyond what the USB module handles automatically
    }
}

// Sent when a control transfer data stage completes, receiving data from the host.
// Returns `TRUE` if more data should be read.
BOOL USB_HandleControlTransferReceiveDataFromHost(int endpointNumber) {
    return FALSE;
}

// Fired when the host is ready for another data stage from device to host.
// Returns `TRUE` if this is not the final data stage: i.e., there is more data to be read.
BOOL USB_HandleControlTransferSendDataToHost(int endpointNumber) {
    return FALSE;
}

// Fired when a control transfer completes, i.e. the status change is successfully transmitted.
void USB_HandleControlTransferComplete(int endpointNumber) {
    return;
}

// Fired when the host writes data to bulk out.
void USB_HandleBulkOutTransferComplete(int endpointNumber) {
    //bulkOutReadFirstHalf = FALSE;
    int packetLength = USB_GetReceiveBufferLength(endpointNumber);
    if(endOfTransfer) {
        // If the transfer is marked as over, and we have already received the short packet,
        // then this packet is the start of the next transfer.
        if(bulkOutGotShortPacket) {
            bulkOutGotShortPacket = FALSE;
            endOfTransfer = FALSE;

            // Immediately copy the first packet to the public buffer.
            USBHandlers_CopyBulkOutBuffers(/*TRUE, TRUE*/);
            //bulkOutReadFirstHalf = TRUE;

            USBHandlers_StartBulkOutTransfer();
        }   else if(packetLength == MAX_PACKET_SIZE)   {
            dbgprintf("Reading more packets to terminate transfer\n");
            // If transfer is marked as over, but we're still receiving full packets, 
            // discard the packets (skipping the rest of the transfer.)
            // (Happens only in the case of errors)
            USB_BulkOutReadyForMoreData(endpointNumber);
        }
    }

    if(packetLength < MAX_PACKET_SIZE) {
        // If the short packet arrives after we've already marked end of transfer,
        // then we're now ready for the next transfer to start
        if(endOfTransfer) {
            USB_BulkOutReadyForMoreData(endpointNumber);
        }
        bulkOutGotShortPacket = TRUE;
    }
}

// Fired when the host reads data from bulk in.
void USB_HandleBulkInTransferComplete(int endpointNumber) {
    dbgprintf("USB_HandleBulkInTransferComplete\n");
    bulkInPrivateBufFull = FALSE;
    // Normally, we wouldn't send the next packet immediately, and would rather wait for the buffer to fill up first.
    // However, at the end of a transfer, the buffer will never fill up, so we send it immediately.
    if(endOfTransfer) {
        if(bulkInPublicBufSize) {
            USBHandlers_CopyBulkInBufferToUSB();
        }   else    {
            dbgprintf("Sent packet terminating transfer\n");
        }
        // TODO: potential race condition if the computer tries to start the next transfer before finishing reading the previous one
        // ideally should be able to handle this although we'll try to make the driver robust.
    }
}

// Sent when the host disconnects from the STM32. (either is unplugged or disables the device)
void USB_HandleSuspend() {
    dbgprintf("USB_HandleSuspend\n");    
}

// Called when we wakeup from a suspension
void USB_HandleResume() { }

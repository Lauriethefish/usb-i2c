/// Primary file for the USB driver. Handles setting up USB endpoints upon reset, as well 
/// as the USB interrupts.

#include "public.h"
#include "private.h"
#include "../stm32.h"
#include "../cortex.h"
#include "../debug.h"
#include "../delay.h"
#include "../interrupts.h"

usbd_endpoint endpoints[MAX_ENDPOINT_PAIRS];
usbd_endpoint* endpointsByNumber[ENDPOINT_NUMBER_MAX];
int numEndpoints = 0;

usb_device_descriptor* pDeviceDescriptor;
usb_configuration_descriptor* pConfigurationDescriptor;

// Table to convert USB transfer types in descriptors into the values used in the endpoint register in the stm32.
usb_epr_transfer_type endpointDescriptorTransferTypeToEndpointRegister[4] = {
    UsbEprTransferTypeControl, // Transfer type 00 = Control
    UsbEprTransferTypeIsochronous, // Transfer type 01 = Isochronous
    UsbEprTransferTypeBulk, // Transfer type 10 = Bulk
    UsbEprTransferTypeInterrupt // Transfer type 11 = Interrupt.
};

void USB_SetDescriptors(usb_device_descriptor* _pDeviceDescriptor, usb_configuration_descriptor* _pConfigurationDescriptor) {
    pDeviceDescriptor = _pDeviceDescriptor;
    pConfigurationDescriptor = _pConfigurationDescriptor;
}


int totalUsbSRAMCapacityNeeded = 0; // Check whether our endpoint buffers will fit in USB SRAM
BOOL USB_AddEndpoint(usb_endpoint_descriptor* endpointDescriptor) {
    int epDescriptorTransferType = endpointDescriptor->bmAttributes & USB_EP_ATTRIBUTES_TRANSFER_TYPE_MASK;
    usb_epr_transfer_type transferType = endpointDescriptorTransferTypeToEndpointRegister[epDescriptorTransferType];
    int endpointNumber = endpointDescriptor->bEndpointAddress & USB_EP_ADDRESS_EP_NUMBER_MASK;
    BOOL isIn = endpointDescriptor->bEndpointAddress & USB_EP_ADDRESS_EP_DIRECTION_MASK;
    BOOL isControl = transferType == UsbEprTransferTypeControl;

    // Check existing endpoints to see if the endpointNumber has an entry: each entry is for an endpoint pair,
    // which has both "in" and "out" endpoints.

    usbd_endpoint* pEndpoint = endpointsByNumber[endpointNumber];
    if(pEndpoint == NULL) { // No direction of the endpoint has been added yet.
        if(numEndpoints >= MAX_ENDPOINT_PAIRS) {
            dbgprintf("FAILED: Max endpoint pairs reached\n");
            return FALSE;
        }
        pEndpoint = &endpoints[numEndpoints];
        endpointsByNumber[endpointNumber] = pEndpoint;

        // Do initial endpoint configuration
        pEndpoint->transferType = transferType;
        pEndpoint->endpointId = numEndpoints;
        pEndpoint->endpointNumber = endpointNumber;
        if(isControl) {
            pEndpoint->canReceive = TRUE;
            pEndpoint->canSend = TRUE;
        }
        pEndpoint->hasDataInTransmitBuffer = FALSE;
        pEndpoint->readyToReceiveData = TRUE;
        
        pEndpoint->endpointStatus = EndpointStatusIdle;
        numEndpoints++;
    }   else if(isControl) {
        // Endpoint number has already been declared.
        // For most endpoint types, their RX and TX directions are declared separately, so this isn't necessarily an error.
        // Control endpoints have both directions implied by one endpoint descriptor
        dbgprintf("FAILED: Control endpoint number %d declared twice\n", endpointNumber);
        return FALSE;
    }

    if(!isControl) {
        if(isIn) {
            if(pEndpoint->canReceive) {
                dbgprintf("FAILED: same direction (RX) of endpoint %d declared twice\n", endpointNumber);
                return FALSE;
            }   else    {
                pEndpoint->canReceive = TRUE;
                return TRUE;
            }
        }   else    {
            if(pEndpoint->canSend) {
                dbgprintf("FAILED: same direction (TX) of endpoint %d declared twice", endpointNumber);
                return FALSE;
            }   else    {
                pEndpoint->canSend = TRUE;
                return TRUE;
            }
        }
    }

    if(isControl) {
        totalUsbSRAMCapacityNeeded += MAX_PACKET_SIZE * 2; // Both RX and TX declared together.
    }   else    {
        totalUsbSRAMCapacityNeeded += MAX_PACKET_SIZE; // RX and TX declared separately.
    }

    // TODO: verify remaining attributes have correct values.
    return TRUE;
}

BOOL USB_BuildConfiguration() {
    dbgprintf("Building USB configuration from descriptors\n");
    // The buffer descripption table requires 8 entries each with 4, two-byte words, so 64 bytes.
    totalUsbSRAMCapacityNeeded = 64;

    if(!pDeviceDescriptor) {
        dbgprintf("FAILED: No device descriptor specified\n");
        return FALSE;
    }

    if(!pConfigurationDescriptor) {
        dbgprintf("FAILED: No configuration descriptor specified\n");
        return FALSE;
    }

    // Add endpoint 0, which is always a control endpoint.
    endpoints[0].canReceive = TRUE;
    endpoints[0].canSend = TRUE;
    endpoints[0].endpointId = 0;
    endpoints[0].endpointNumber = 0;
    endpoints[0].endpointStatus = EndpointStatusIdle;
    endpoints[0].transferType = UsbEprTransferTypeControl;
    numEndpoints = 1;
    endpointsByNumber[0] = &endpoints[0];

    char* cfgCursor = (char*) pConfigurationDescriptor;
    int totalLength = pConfigurationDescriptor->wTotalLengthLow + (pConfigurationDescriptor->wTotalLengthHigh << 8);
    char* endOfCfg = cfgCursor + totalLength;
    // Walk the interface descriptors and the endpoints: verify each has been set up properly.
    cfgCursor += pConfigurationDescriptor->bLength;
    while(cfgCursor < endOfCfg) {
        usb_descriptor* desc = (usb_descriptor*) cfgCursor;
        // For each endpoint descriptor, add an endpoint.
        if(desc->bDescriptorType == USB_ENDPOINT_DESCRIPTOR_TYPE) {
            dbgprintf("Adding EP\n");
            if(!USB_AddEndpoint((usb_endpoint_descriptor*) desc)) {
                return FALSE;
            }   
        }
        cfgCursor += desc->bLength;
    }

    dbgprintf("SUCCESS: Endpoints added to configuration\n");

    if(totalUsbSRAMCapacityNeeded > USB_SRAM_CAPACITY) {
        dbgprintf("FAILED: Not enough buffer space for all USB endpoints. Need: %d, max: %d\n", totalUsbSRAMCapacityNeeded, USB_SRAM_CAPACITY);
        return FALSE;
    }

    return TRUE;
}

// Pulls the D+ pin of the USB peripheral low, which triggers the host to enumerate the device.
void USB_pull_dplus_low() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIO A

    // Set to general purpose output push pull
    // Set output value to 0.

    int portNum = 12 - 8; // Pin 12 is in the "configuration register high."
    GPIO_A->CRH &= CLEAR_PORT_CFG_MASK(portNum);
    GPIO_A->CRH |= SET_PORT_CNF_MASK(portNum, GP_OUTPUT_PUSH_PULL);
    GPIO_A->CRH |= SET_PORT_MODE_MASK(portNum, OUTPUT_MODE_MAX_10_MHZ);
}

void USB_float_dplus() {
    int portNum = 12 - 8;
    GPIO_A->CRH &= CLEAR_PORT_CFG_MASK(portNum);
    GPIO_A->CRH |= SET_PORT_CNF_MASK(portNum, INPUT_FLOATING);
    GPIO_A->CRH |= SET_PORT_MODE_MASK(portNum, INPUT_MODE);
}

BOOL USB_Begin() {
    if(!USB_BuildConfiguration()) {
        dbgprintf("USB_BuildConfiguration failed. USB startup failed\n");
        return FALSE;
    }

    USB_pull_dplus_low();
    delay_us(5000); // Wait for the device to detect the bus reset
    USB_float_dplus();

    // Enable the USB clock
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // Disable the USB powerdown: i.e., start up the peripheral
    USB->CNTR &= ~USB_CNTR_PDWN;

    // Clear reset condition on USB part after t_STARTUP
    // This is 1us for STM32F103.
    delay_us(1);

    // Remove USB reset condition.
    USB->CNTR &= ~USB_CNTR_FRES;
    USB->ISTR = 0;

    // Enable interrupts on successful data transfer, errored data transfer, and on USB reset
    USB->CNTR |= USB_CNTR_CTRM | USB_CNTR_ERRM | USB_CNTR_RESETM
        | USB_CNTR_SUSPM;

    // Enable USB interrupts
    *NVIC_ISER0 = USB_HP_CAN_TX_MASK | USB_LP_CAN_RX0_MASK;

    return TRUE;
}

// Creates a buffer description table for the USB endpoints.
// The USB SRAM holds 256 2-byte words, i.e. 512 bytes.
void USB_CreateBufferDescTable() {
    // Locate at the beginning of USB SRAM (we can choose but the beginning is easiest)
    BF_DESC_TABLE* bufferDescTable = (BF_DESC_TABLE*) USB_SRAM;
    USB->BTABLE = (int) bufferDescTable;

    uint32_t* afterDescTable = (uint32_t*) (bufferDescTable + 1);
    int currentAddress = 64; // Just past the end of the BF_DESC_TABLE
    for(int i = 0; i < numEndpoints; i++) {
        BF_DESC_TABLE_ENTRY* bdtEntry = &bufferDescTable->entries[i];
        usbd_endpoint* endpoint = &endpoints[i];

        // This code doesn't attempt to check that all the buffers fit in the memory - this is handled by `USB_AddEndpoint`
        if(endpoint->canSend) {
            bdtEntry->TX_ADDR = currentAddress;
            bdtEntry->TX_CNT = MAX_PACKET_SIZE;
            currentAddress += MAX_PACKET_SIZE;
        }
        if(endpoint->canReceive) {
            bdtEntry->RX_ADDR = currentAddress;
            currentAddress += MAX_PACKET_SIZE;

            if(MAX_PACKET_SIZE != 64) {
                dbgprintf("Bitmasks set up for 64 byte packets but MAX_PACKET_SIZE didn't match\n");
                BKPT();
            }
            bdtEntry->RX_COUNT = USB_COUNT_RX_BLSIZE | USB_COUNT_RX_NUM_BLOCK(1);
        }

    }
}

// Configures the register for the given endpoint, changing the current status of the transmitting/receiving end to the given value.
void USB_ConfigureEndpoint(int endpointId, 
    USB_STAT newRxStat,
    USB_STAT newTxStat, BOOL setEPKind) {
    usbd_endpoint* pEndpoint = &endpoints[endpointId];

    int prevRegValue = USB->EPR[endpointId];
    USB_STAT prevRxStat = (prevRegValue >> 12) & 3;
    USB_STAT prevTxStat = (prevRegValue >> 4) & 3;

    int regValue = pEndpoint->endpointNumber | pEndpoint->transferType 
        | USB_EPR_STAT_RX_MASK(prevRxStat, newRxStat) | USB_EPR_STAT_TX_MASK(prevTxStat, newTxStat);

    if(setEPKind) {
        regValue |= USB_EPR_EP_KIND;
    }

    USB->EPR[endpointId] = regValue;
}

// Configures a bulk endpoint based on the `hasDataInTransmitBuffer` and `readyToReceiveData` fields in usbd_endpoint
void USB_ConfigureBulkEndpoint(usbd_endpoint* pEndpoint) {
    USB_ConfigureEndpoint(pEndpoint->endpointId,
        pEndpoint->readyToReceiveData ? UsbStatValid : UsbStatNak,
        pEndpoint->hasDataInTransmitBuffer ? UsbStatValid : UsbStatNak,
        FALSE);
}

BOOL USB_WriteTransmitBuffer(int endpointNumber, char* buffer, int length) {
    // The USB SRAM is accessed with 32 bit words, but each word actually only references 16 bits of memory.
    // To improve abstraction we write the data in the required format automatically

    if(endpointNumber >= ENDPOINT_NUMBER_MAX || endpointNumber < 0) {
        dbgprintf("Endpoint number out of range\n");
        return FALSE;
    }
    usbd_endpoint* pEndpoint = endpointsByNumber[endpointNumber];
    if(!pEndpoint) {
        dbgprintf("Endpoint number unknown\n");
        return FALSE;
    }

    if(length > MAX_PACKET_SIZE) {
        dbgprintf("USB_WriteTransmitBuffer FAILED: Tried to write %d bytes, more than max packet size\n", length);
        return FALSE;
    }

    // Read buffer descriptor table to get address of TX buffer.
    // NB: TX_ADDR is relative to the USB SRAM (16 bit words), not our view of the USB SRAM (32 bit words)
    BF_DESC_TABLE_ENTRY* bdtEntry = &((BF_DESC_TABLE*) USB_SRAM)->entries[pEndpoint->endpointId];
    bdtEntry->TX_CNT = length;

    uint32_t* usbBuffer = (uint32_t*) (USB_SRAM + (bdtEntry->TX_ADDR * 2));

    int numWords = length >> 1;
    for(int word = 0; word < numWords; word++) {
        // Transmission starts from the least significant byte (that is, little endian)
        // So we send the second byte of the buffer as the most significant byte to counteract.
        int byteNum = word << 1;
        usbBuffer[word] = ((int) buffer[byteNum] | ((int) buffer[byteNum + 1] << 8));
    }

    // If we have an extra byte leftover, write the final byte into the LSByte of another word. 
    if(length > (numWords << 1)) {
        usbBuffer[numWords] = buffer[length - 1];
    }

    pEndpoint->hasDataInTransmitBuffer = TRUE;
    USB_ConfigureBulkEndpoint(pEndpoint);
    return TRUE;
}

int USB_GetReceiveBufferLength(int endpointNumber) {
    if(endpointNumber >= ENDPOINT_NUMBER_MAX || endpointNumber < 0) {
        return -1;
    }
    usbd_endpoint* pEndpoint = endpointsByNumber[endpointNumber];
    if(!pEndpoint) {
        return -1;
    }

    BF_DESC_TABLE_ENTRY* bdtEntry = &((BF_DESC_TABLE*) USB_SRAM)->entries[pEndpoint->endpointId];
    return bdtEntry->RX_COUNT & USB_COUNT_RX_NUM_BYTES_MASK;
}

BOOL USB_ReadReceiveBuffer(int endpointNumber, char* buffer, int maxLength, int* outRxLength, int offset) {
    // Similarly, reading data requires removing the two "padding" bytes between each word of data.
    if(endpointNumber >= ENDPOINT_NUMBER_MAX || endpointNumber < 0) {
        return FALSE;
    }
    usbd_endpoint* pEndpoint = endpointsByNumber[endpointNumber];
    if(!pEndpoint) {
        return FALSE;
    }
    
    BF_DESC_TABLE_ENTRY* bdtEntry = &((BF_DESC_TABLE*) USB_SRAM)->entries[pEndpoint->endpointId];
    uint32_t* usbBuffer = (uint32_t*) (((char*) USB_SRAM) + (bdtEntry->RX_ADDR * 2));
    usbBuffer += offset >> 1;

    int bytesReceived = bdtEntry->RX_COUNT & USB_COUNT_RX_NUM_BYTES_MASK;
    if(bytesReceived == 0) {
        *outRxLength = 0;
        return TRUE; // Zero length packet - don't try to read its data from USB SRAM
    }

    int bytesAfterOffset = bytesReceived - offset;
    if(bytesAfterOffset < 0) {
        *outRxLength = 0;
        return TRUE;
    }

    // Only read as many bytes as were specified.
    if(bytesAfterOffset > maxLength) {
        bytesAfterOffset = maxLength;
    }

    int wordsReceived = bytesAfterOffset >> 1;
    uint32_t* usbEndOfBuffer = usbBuffer + wordsReceived;

    while(usbBuffer < usbEndOfBuffer) {
        uint32_t word = *usbBuffer;
        // 16 bit words have the first byte received as the least significant
        *buffer = word & 0xFF;
        buffer++;
        *buffer = word >> 8;
        buffer++;

        usbBuffer++;
    }

    // Read final byte, in least significant half of the final word
    if(bytesAfterOffset & 1) {
        *buffer = *usbBuffer;
    }

    *outRxLength = bytesAfterOffset;
    return TRUE;
}

void USB_BulkOutReadyForMoreData(int endpointNumber) {
    // Packet has been copied into a user buffer, so we are ready for another packet.
    usbd_endpoint* pEndpoint = endpointsByNumber[endpointNumber];
    pEndpoint->readyToReceiveData = TRUE;
    USB_ConfigureBulkEndpoint(pEndpoint);
}

void USB_ResetBulkEndpoint(int endpointNumber) {
    usbd_endpoint* pEndpoint = endpointsByNumber[endpointNumber];
    pEndpoint->readyToReceiveData = TRUE;
    pEndpoint->hasDataInTransmitBuffer = FALSE;
    USB_ConfigureBulkEndpoint(pEndpoint);
}

// Sets up the registers and enables the given endpoint, at index `endpointId` in `endpoints`
void USB_InitialiseEndpoint(int endpointId) {
    usbd_endpoint* pEndpoint = &endpoints[endpointId];
    // RX is set to VALID by default - we are always able to receive data.
    // TX is set to NAK by default - until we add data to the TX buffer, we may assume there is nothing to send.
    // If the host requests to do a read, it will be forced to wait until data is available.
    USB_ConfigureEndpoint(endpointId, UsbStatValid, UsbStatNak, FALSE);
}

// Called by the control endpoint handler when a configuration is selected.
void USB_SetupNonDefaultEndpoints() {
    for(int i = 1; i < numEndpoints; i++) {
        dbgprintf("Initialising endpoint %d\n", i);
        USB_InitialiseEndpoint(i);
    }
}

// Called by the control endpoint handler when the device moves back to "Address" mode, i.e. no configuration selected.
void USB_ClearNonDefaultEndpoints() {
    for(int i = 1; i < numEndpoints; i++) {
        USB->EPR[i] = 0;
    }
}

/// Interrupt handlers for the USB peripheral (low/high priority interrupts)

// Called when the USB peripheral is ready to set up endpoints and other parameters.
void USB_Reset() {
    dbgprintf("USB_Reset -----------------------------------\n");
    USB->DADDR = USB_DADDR_EF; // Enable USB device.

    // Initialise default endpoint, only.
    USB_Control_Reset();
    USB_ClearNonDefaultEndpoints();
    USB_InitialiseEndpoint(0);

    USB_CreateBufferDescTable();
}
// Called if a transfer succeeds.
void USB_CorrectTransfer() {
    int endpointId = USB->ISTR & USB_ISTR_EP_ID_MASK;
    int isOut = USB->ISTR & USB_ISTR_DIR;

    usbd_endpoint* pEndpoint = &endpoints[endpointId];

    BF_DESC_TABLE_ENTRY* bdtEntry = &((BF_DESC_TABLE*) USB_SRAM)->entries[endpointId];
    USBD_ENDPOINT_STATUS epStatus = pEndpoint->endpointStatus;

    // Currently only control and bulk transfers are supported: more to come! (one day!)

    dbgprintf("USB_CorrectTransfer for endpoint %d. Current status: %d\n", endpointId, epStatus);
    switch(pEndpoint->transferType) {
    case UsbEprTransferTypeControl:
        USB_ControlTransferStateMachine(pEndpoint, bdtEntry);
        break;
    case UsbEprTransferTypeBulk:
        if(isOut) {
            // Not ready to receive more data until the existing data is processed
            pEndpoint->readyToReceiveData = FALSE;
            USB_ConfigureBulkEndpoint(pEndpoint);

            USB_HandleBulkOutTransferComplete(pEndpoint->endpointNumber);
        }   else    {
            // Transfer buffer is empty until it is refilled.
            pEndpoint->hasDataInTransmitBuffer = FALSE;
            USB_ConfigureBulkEndpoint(pEndpoint);

            USB_HandleBulkInTransferComplete(pEndpoint->endpointNumber);
        }
        break;
    default:
        dbgprintf("Unsupported endpoint type\n");
        abort();
    }
}

// Called if a transfer fails
void USB_Error() {
    dbgprintf("USB error detected\n");
    dbgprintf("EPR0: %w", USB->EPR[0]);
    // Normally ignore errors, since the USB peripheral will re-send data automatically.
    // Still log something, as the error can be used to detect loose cables or other issues.
}

/// The below functions are the NVIC interrupt handlers, which check registers for the appropriate sub-interrupt.

// USB low priority interrupts: dispatch to the relevant interrupt handler and clear the interrupt.
void USB_LP_CAN_RX0(void) {
    prevent_blocking();
    // Reset request
    if(USB->ISTR & USB_ISTR_RESET) {
        USB->ISTR &= ~USB_ISTR_RESET;
        USB_Reset();
    }   else if(USB->ISTR & USB_ISTR_ERR) {
        USB->ISTR &= ~USB_ISTR_ERR;
        USB_Error();
    }   else if(USB->ISTR & USB_ISTR_CTR) {
        // No need to clear the CTR bit (it is read-only)
        USB_CorrectTransfer();
    }   else if(USB->ISTR & USB_ISTR_SUSP) {
        USB->ISTR &= ~USB_ISTR_SUSP;
        USB_HandleSuspend();
    }   else if(USB->ISTR & USB_ISTR_WAKEUP) {
        USB->ISTR &= ~USB_ISTR_WAKEUP;
        USB_HandleResume();
    }
}

// USB high priority interrupts (currently only bulk transfers)
void USB_HP_CAN_TX(void) {
    dbgprintf("Unimplemented: USB_HP_CAN_TX\n");
    BKPT(); // Not yet used as bulk endpoints not implemented.
}

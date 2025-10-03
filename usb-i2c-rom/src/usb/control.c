/// State machine for control transfers in the USB driver.
/// We call into this file whenever a transfer succeeds on a control endpoint.
/// The data is read/written using the control endpoint callbacks, and some control requests
/// (such as those to get device descriptors), are handled automatically.

#include "public.h"
#include "private.h"
#include "spec_types.h"
#include "../debug.h"

/// Internal handlers for control transfer events.
/// These deal with returning descriptors on endpoint 0, as well as SetAddress requests 
/// Other requests are forwarded to the user handler.
/// TODO: Also implement the other setup requests.

char* sendingData;
int sendingLength;

uint16_t deviceStatus; // Remote wakeup disabled, self powered disabled.

// The reference manual states that when we receive a SetAddress request, we must only set the address
// once the status change is received. So we keep track of a pending address here.
int pendingAddress;

// Set to 0 if in the default state or address state.
// Set to 1 if in the configured state; only one configuration is supported.
int selectedConfiguration;

// Resets the control state machine whenever the USB device resets
void USB_Control_Reset() {
    selectedConfiguration = 0;
    pendingAddress = -1;
    sendingData = NULL;
    sendingLength = 0;
}

CONTROL_TRANSFER_TYPE USB_HandleSetupTransaction_Internal(int endpointNumber, setup_packet setupPacket) {
    if(endpointNumber != 0) {
        return USB_HandleSetupTransaction(endpointNumber, setupPacket);
    }

    switch(setupPacket.bRequest) {
    case GET_DESCRIPTOR_REQUEST_TYPE:
        int readingDescriptorType = setupPacket.wValue >> 8; // High byte contains descriptor type
        int descriptorLen;
        if(readingDescriptorType == USB_DEVICE_DESCRIPTOR_TYPE) {
            dbgprintf("Sending device descriptor\n", setupPacket.wLength);
            sendingData = (char*) pDeviceDescriptor;
            descriptorLen = sizeof(usb_device_descriptor);
        }   else if(readingDescriptorType == USB_CFG_DESCRIPTOR_TYPE) {
            dbgprintf("Sending configuration descriptor\n", setupPacket.wLength);
            sendingData = (char*) pConfigurationDescriptor;
            descriptorLen = pConfigurationDescriptor->wTotalLengthLow | (pConfigurationDescriptor->wTotalLengthHigh << 8);
        }   else    {
            dbgprintf("Unknown descriptor type %d\n", readingDescriptorType);
            return ControlTransferTypeSendDataError;
        }

        // We send the full length of the descriptor if setupPacket.wLength is long enough.
        // Otherwise, we send only the first setupPacket.wLength bytes.
        sendingLength = descriptorLen > setupPacket.wLength ? setupPacket.wLength : descriptorLen;

        return ControlTransferTypeSendDataToHost;
    case SET_ADDRESS_REQUEST_TYPE:
        pendingAddress = setupPacket.wValue & 0xFF;
        dbgprintf("SetAddress: %d\n", pendingAddress);

        return ControlTransferTypeNoData;
    case GET_CONFIGURATION_REQUEST_TYPE:
        // Send the bConfigurationValue in response
        sendingData = (char*) &selectedConfiguration;
        sendingLength = 1;
        return ControlTransferTypeSendDataToHost;
    case GET_STATUS_REQUEST_TYPE:
        // TODO: Right now this sends a value of 0 no matter the request
        // This is the correct value for our purposes, however we should be sending a request error if the request is for an interface/endpoint that doesn't exist.
        sendingData = (char*) &deviceStatus;
        sendingLength = sizeof(deviceStatus);
        return ControlTransferTypeSendDataToHost;
    case SET_CONFIGURATION_REQUEST_TYPE:
        if(selectedConfiguration == 0 && setupPacket.wValue == 1) {
            selectedConfiguration = setupPacket.wValue;
            dbgprintf("Configuration 1 selected\n");
            USB_SetupNonDefaultEndpoints();
            USB_HandleConfigurationSelected();
            return ControlTransferTypeNoData;
        }   else  if(selectedConfiguration == 1 && setupPacket.wValue == 0) {
            selectedConfiguration = 0;
            dbgprintf("Moved back to address state\n");
            USB_ClearNonDefaultEndpoints();
            return ControlTransferTypeNoData;
        }   else    {
            // Invalid request, leave configuration unchanged
            return ControlTransferTypeSendDataError;
        }
    default:
        dbgprintf("Unknown request type %d\n", setupPacket.bRequest);
        // Request type not supported, send on to the underlying handler:
        return USB_HandleSetupTransaction(endpointNumber, setupPacket);
    }
}

BOOL USB_HandleControlTransferReceiveDataFromHost_Internal(int endpointNumber) {
    // No supported request types require receiving data from the host
    return USB_HandleControlTransferReceiveDataFromHost(endpointNumber);
}

BOOL USB_HandleControlTransferSendDataToHost_Internal(int endpointNumber) {
    if(endpointNumber != 0 && sendingData == NULL) {
        return USB_HandleControlTransferSendDataToHost(endpointNumber);
    }   else    {
        int length = sendingLength > MAX_PACKET_SIZE ? MAX_PACKET_SIZE : sendingLength;
        if(!USB_WriteTransmitBuffer(endpointNumber, sendingData, length)) {
            dbgprintf("Writing transmit buffer failed\n");
            abort();
        }

        sendingLength -= length;
        if(sendingLength) {
            sendingData += length;
            return TRUE;
        }   else    {
            sendingData = NULL;
            return FALSE;
        }
    }
}

void USB_HandleControlTransferComplete_Internal(int endpointNumber) {
    if(endpointNumber == 0 && pendingAddress != -1) {
        USB->DADDR |= pendingAddress & USB_DADDR_MASK;
        pendingAddress = -1;
        dbgprintf("Set address complete\n");
    }   else    {
        USB_HandleControlTransferComplete(endpointNumber);
    }
}

/// State machines for control transfers

void USB_DispatchSetupPacket(usbd_endpoint* pEndpoint, BF_DESC_TABLE_ENTRY* bdtEntry) {
    int endpointId = pEndpoint->endpointId;
    if(!(USB->EPR[endpointId] & USB_EPR_SETUP_MASK)) {
        print("SETUP flag not set, but endpoint is idle??\n");
        // TODO: work out a way to shut down the USB peripheral in this case, or recover from the error
        BKPT();
    }
    
    setup_packet setupPacket;
    int bytesRead;
    if(!USB_ReadReceiveBuffer(pEndpoint->endpointNumber, (char*) &setupPacket, sizeof(setup_packet), &bytesRead, 0)) {
        print("Reading SETUP packet should never fail\n");
        abort();
    }

    switch(USB_HandleSetupTransaction_Internal(pEndpoint->endpointNumber, setupPacket)) {
    case ControlTransferTypeNoData:
        // Immediately send status change to host
        bdtEntry->TX_CNT = 0;
        pEndpoint->endpointStatus = EndpointStatusSendingStatusChange;
        USB_ConfigureEndpoint(endpointId, UsbStatNak, UsbStatValid, FALSE);
        break;

    case ControlTransferTypeReceiveDataError:
        // Send a "Request Error" by setting RX to a stall
        USB_ConfigureEndpoint(endpointId, UsbStatStall, UsbStatNak, FALSE);
        pEndpoint->endpointStatus = EndpointStatusIdle;
        break;

    case ControlTransferTypeReceiveDataFromHost:
        pEndpoint->endpointStatus = EndpointStatusDataStageReceive;
        USB_ConfigureEndpoint(endpointId, UsbStatValid, UsbStatNak, FALSE);
        break;
    case ControlTransferTypeSendDataError:
        // Send a "Request Error" by setting TX to a stall
        USB_ConfigureEndpoint(endpointId, UsbStatNak, UsbStatStall, FALSE);
        pEndpoint->endpointStatus = EndpointStatusIdle;
        break;

    case ControlTransferTypeSendDataToHost:
        if(!USB_HandleControlTransferSendDataToHost_Internal(pEndpoint->endpointNumber)) {
            pEndpoint->endpointStatus = EndpointStatusFinalDataStageSend;
        }   else    {
            pEndpoint->endpointStatus = EndpointStatusDataStageSend;
        }
        USB_ConfigureEndpoint(endpointId, UsbStatNak, UsbStatValid, FALSE);
        
        break;
    }
}

// Called whenever a packet is transferred successfully to a control endpoint.
void USB_ControlTransferStateMachine(usbd_endpoint* pEndpoint, BF_DESC_TABLE_ENTRY* bdtEntry) {
    int endpointId = pEndpoint->endpointId;
    switch(pEndpoint->endpointStatus) {
    case EndpointStatusIdle:
        // Both RX and TX are NAK
        pEndpoint->endpointStatus = EndpointStatusGotSetup;
        USB_DispatchSetupPacket(pEndpoint, bdtEntry);

        break;

    case EndpointStatusDataStageReceive:
        // Handle the received data and check if we need any more.
        if(!USB_HandleControlTransferReceiveDataFromHost_Internal(pEndpoint->endpointNumber)) {
            // Received last block of data, so send a status change back: 0 byte transfer
            bdtEntry->TX_CNT = 0;
            pEndpoint->endpointStatus = EndpointStatusSendingStatusChange;
            USB_ConfigureEndpoint(endpointId, UsbStatNak, UsbStatValid, FALSE);
        }   else    {
            USB_ConfigureEndpoint(endpointId, UsbStatValid, UsbStatNak, FALSE);
        }
        break;

    case EndpointStatusDataStageSend:
        if(!USB_HandleControlTransferSendDataToHost_Internal(pEndpoint->endpointNumber)) {
            pEndpoint->endpointStatus = EndpointStatusFinalDataStageSend;
            // Still sending the last block of data, so leave endpoint parameters the same.
        }

        USB_ConfigureEndpoint(endpointId, UsbStatNak, UsbStatValid, FALSE);
        break;
    case EndpointStatusFinalDataStageSend:
        // Receive a status change (0 byte RX)
        // To ensure the RX is 0 bytes, we set EP_KIND, which corresponds to the STATUS_OUT bit for a control endpoint.
        pEndpoint->endpointStatus = EndpointStatusReceivingStatusChange;
        USB_ConfigureEndpoint(endpointId, UsbStatValid, UsbStatNak, TRUE);
        break;

    case EndpointStatusReceivingStatusChange:
        pEndpoint->endpointStatus = EndpointStatusIdle;
        USB_HandleControlTransferComplete_Internal(pEndpoint->endpointNumber);
        USB_ConfigureEndpoint(endpointId, UsbStatValid, UsbStatNak, FALSE);
        break;

    case EndpointStatusSendingStatusChange:
        pEndpoint->endpointStatus = EndpointStatusIdle;
        USB_HandleControlTransferComplete_Internal(pEndpoint->endpointNumber);
        USB_ConfigureEndpoint(endpointId, UsbStatValid, UsbStatNak, FALSE);
        
        // Allowed to receive but cannot transmit.
        break;
    default:
        // Unreachable as we should always set a valid endpoint status.
        dbgprintf("Invalid endpoint status %d for control transaction\n", pEndpoint->endpointStatus);
        abort();
    }
}
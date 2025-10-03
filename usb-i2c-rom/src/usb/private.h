#pragma once

#include "spec_types.h"

#define MAX_ENDPOINT_PAIRS 8
#define ENDPOINT_NUMBER_MAX 16 // Endpoint number is a 4-bit value.
#define USB_SRAM_CAPACITY 512

// Enumeration for the transfer types found in the USB endpoint register.
// These have different identifiers to the endpoint types found in endpoint descriptors,
// so we must be careful to convert between.
typedef enum _usb_epr_transfer_type {
	UsbEprTransferTypeControl = USB_EPR_EP_TYPE_CONTROL,
	UsbEprTransferTypeIsochronous = USB_EPR_EP_TYPE_ISO,
	UsbEprTransferTypeBulk = USB_EPR_EP_TYPE_BULK,
	UsbEprTransferTypeInterrupt = USB_EPR_EP_TYPE_INTERRUPT
} usb_epr_transfer_type;

typedef enum _USBD_ENDPOINT_STATUS {
    EndpointStatusIdle,

    /// Control transfers

    EndpointStatusGotSetup,

    // Host to device transfers: data received from host, then a status change sent back to host.
    EndpointStatusDataStageReceive,
    EndpointStatusSendingStatusChange,

    // Device to host transfers: data written to host, then a status change received from the host.
    EndpointStatusDataStageSend, 
    EndpointStatusFinalDataStageSend, // Final data stage; upon completion, we move to `EndpointStatusReceivingStatusChange`
    EndpointStatusReceivingStatusChange,
} USBD_ENDPOINT_STATUS;

// Internal structure holding details about an endpoint pair.
typedef struct _usbd_endpoint {
    usb_epr_transfer_type transferType;
    int endpointId; // Index of this endpoint within `endpoints`
    int endpointNumber; // Index of this endpoint in the USB endpoint descriptor.

    // Whether the endpoint can (at some point) send and receive data.
    // Both always TRUE for control endpoints.
    BOOL canSend;
    BOOL canReceive;

    // Field for bulk endpoints which indicates whether the endpoint has more data to transmit,
    // or whether the transmit buffer is empty (FALSE)
    // If TRUE, the TX side should be set to VALID, otherwise it should be NAK
    BOOL hasDataInTransmitBuffer;

    // Field for bulk endpoints which indicates whether they are ready to receive additional data from the host (TRUE),
    // or whether they are still processing the data (FALSE)
    BOOL readyToReceiveData;

    USBD_ENDPOINT_STATUS endpointStatus;
} usbd_endpoint;

// The index into this array corresponds to the index of the endpoint register on the USB peripheral (USB_EPnR).
extern usbd_endpoint endpoints[MAX_ENDPOINT_PAIRS];
// The index into this array corresponds to the endpoint number in the USB endpoint descriptor.
extern usbd_endpoint* endpointsByNumber[ENDPOINT_NUMBER_MAX];
extern int numEndpoints;

// User specified device and configuration descriptors.
extern usb_device_descriptor* pDeviceDescriptor;
extern usb_configuration_descriptor* pConfigurationDescriptor;

// Called by the control endpoint handler when a configuration is selected.
void USB_SetupNonDefaultEndpoints();

// Called by the control endpoint handler when the device moves back to "Address" mode, i.e. no configuration selected.
void USB_ClearNonDefaultEndpoints();

// Configures the register for the given endpoint, changing the current status of the transmitting/receiving end to the new value,
// and updating it within the structure.
void USB_ConfigureEndpoint(int endpointId, 
    USB_STAT newRxStat,
    USB_STAT newTxStat, BOOL setEPKind);

// Resets the control state machine whenever the USB device resets
void USB_Control_Reset();

// Called whenever a packet is transferred successfully to a control endpoint.
void USB_ControlTransferStateMachine(usbd_endpoint* pEndpoint, BF_DESC_TABLE_ENTRY* bdtEntry);
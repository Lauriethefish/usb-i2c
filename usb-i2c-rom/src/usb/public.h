#pragma once

// Public definitions for USBD, the simple USB driver.

#include "./spec_types.h"
#include "../stm32.h"

// Remember to change in `CreateBufferDescTable` as well.
#define MAX_PACKET_SIZE 64

// This header provides the interface through which the rest of the project communicates with the USB driver.
// The USB driver is minimalistic and only supports one configuration. Alternate settings for interfaces are not supported.


// NB: Methods for setting up endpoints are omitted as these are determined automatically from the configuration descriptor.

// Sets the device descriptor and configuration descriptor for the endpoint.
// The `configurationDescriptor` may be followed by interface/endpoint descriptors, and its length is determined using the field `wTotalLength`.
// NB: The implementation has no support for string descriptors.
// Control transfer requests for descriptors are handled automatically.
void USB_SetDescriptors(usb_device_descriptor* pDeviceDescriptor, usb_configuration_descriptor* pConfigurationDescriptor);

// Starts the USB peripheral, allowing the USB to connect to a host and enumerate.
// The clock to the USB must be set to 48MHz.
// Also does verification to ensure that the endpoints added match up with the deviceDescriptor and configurationDescriptor.
// Returns `FALSE` if verification fails.
BOOL USB_Begin();

// Writes the data in the given buffer to be transmitted to the endpoint.
// Returns TRUE on success, FALSE on failure: e.g. if `buffer` is longer than `MAX_PACKET_SIZE`,
// or if `endpointNumber` doesn't exist, etc..
BOOL USB_WriteTransmitBuffer(int endpointNumber, char* buffer, int length);

// Reads the data from the receiving buffer for `endpointNumber` into `buffer`, of length `maxLength`. If
// there is more data than will fit into the buffer, the remaining data is truncated.
// The receive buffer is read from the byte with index `offset` and onwards. (offset must be a multiple of two)
// Returns TRUE on success, or FALSE if the endpoint didn't exist, or another error occured.
//
// If successful, `outRxLength` is overwritten with the number of bytes received.
BOOL USB_ReadReceiveBuffer(int endpointNumber, char* buffer, int maxLength, int* outRxLength, int offset);

// Gets the length of the receive buffer for the given endpoint without copying anything from it.
// If `endpointNumber` is not a valid endpoint, `-1` is returned.
int USB_GetReceiveBufferLength(int endpointNumber);

// Indicates to the driver that the client is ready to read more data from bulk out
// - i.e. it has processed all the data it has received.
void USB_BulkOutReadyForMoreData(int endpointNumber);

// Resets the given bulk endpoint number, for the host this means:
// The host CAN SEND data to the endpoint
// The host can't receive anything from the endpoint.
void USB_ResetBulkEndpoint(int endpointNumber);


/// CALLBACKS

// These callbacks give methods which need to be defined by the application using the USB driver.
// The methods handle the data recieved, for example control or bulk transfers.

// A USB control transfer starts with a SETUP transaction, followed by data stages (all in the same direction), then status change
// (0 byte transfer in the opposite direction)

typedef enum _CONTROL_TRANSFER_TYPE {
    ControlTransferTypeNoData, // Transfer ends immediately
    ControlTransferTypeReceiveDataFromHost, // Transfer consists of one or more stages of reading data from the host
    ControlTransferTypeReceiveDataError, // Transfer would consist of reading data from the host but the request is invalid.
    ControlTransferTypeSendDataToHost, // Transfer consists of one or more stages of writing data to host
    ControlTransferTypeSendDataError, // Transfer would write data to the host but there's an error.
} CONTROL_TRANSFER_TYPE;

// Handles a SETUP packet, which begins a control transfer. Returns a value indicating the direction data will be sent.
CONTROL_TRANSFER_TYPE USB_HandleSetupTransaction(int endpointNumber, setup_packet setupPacket);

// Sent when a control transfer data stage completes, receiving data from the host.
// Returns `TRUE` if more data should be read.
BOOL USB_HandleControlTransferReceiveDataFromHost(int endpointNumber);

// Fired when the host is ready for another data stage from device to host.
// Returns `TRUE` if this is not the final data stage: i.e., there is more data to be read.
BOOL USB_HandleControlTransferSendDataToHost(int endpointNumber);

// Fired when a control transfer completes, i.e. the status change is successfully transmitted.
void USB_HandleControlTransferComplete(int endpointNumber);

// Fired when the host writes data to bulk out.
// When this function is called, it won't get called again until `USB_BulkOutReadyForMoreData` is called.
// This allows the client to delay further writes until it has processed all the data it has received.
void USB_HandleBulkOutTransferComplete(int endpointNumber);

// Fired when the host reads data from bulk in.
void USB_HandleBulkInTransferComplete(int endpointNumber);

// Sent when the host disconnects from the STM32.
// Specifically, the host is supposed to send a SOF packet every MS, but if it does not,
// for 3ms, a suspend condition is generated.
void USB_HandleSuspend();

// Sent when the host resumes after suspension.
void USB_HandleResume();

// Fired when the USB device has its configuration selected.
// (This implementation only supports a single configuration)
void USB_HandleConfigurationSelected();
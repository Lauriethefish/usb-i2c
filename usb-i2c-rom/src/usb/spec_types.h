#pragma once

// Definitions for various binary data structures with layouts that match the USB 2.0 specification.

#include <stdint.h>
#include "../stm32.h"

#define GET_STATUS_REQUEST_TYPE 0
#define CLEAR_FEATURE_REQUEST_TYPE 1
#define SET_FEATURE_REQUEST_TYPE 3
#define SET_ADDRESS_REQUEST_TYPE 5
#define GET_DESCRIPTOR_REQUEST_TYPE 6
#define SET_DESCRIPTOR_REQUEST_TYPE 7
#define GET_CONFIGURATION_REQUEST_TYPE 8
#define SET_CONFIGURATION_REQUEST_TYPE 9
#define GET_INTERFACE_REQUEST_TYPE 10
#define SET_INTERFACE_REQUEST_TYPE 11
#define SYNCH_FRAME_REQUEST_TYPE 12


typedef struct _setup_packet {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} setup_packet;

#define USB_DEVICE_DESCRIPTOR_TYPE 1
#define USB_CFG_DESCRIPTOR_TYPE 2
#define USB_INTERFACE_DESCRIPTOR_TYPE 4
#define USB_ENDPOINT_DESCRIPTOR_TYPE 5

// Represents a generic USB descriptor.
typedef struct _usb_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
} usb_descriptor;

typedef struct _usb_device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} usb_device_descriptor;

typedef struct _usb_configuration_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	// Total length of descriptor is split between two uint8_t values to keep the alignment of this structure
	// to one byte. Otherwise the length of the structure becomes 10 due to padding.
	uint8_t wTotalLengthLow;
	uint8_t wTotalLengthHigh;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
} usb_configuration_descriptor;

typedef struct _usb_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternativeSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} usb_interface_descriptor;

#define USB_EP_ADDRESS_EP_NUMBER_MASK 0xF
#define USB_EP_ADDRESS_EP_DIRECTION_MASK (1 << 7)
#define USB_EP_ATTRIBUTES_TRANSFER_TYPE_MASK 0b11

typedef struct _usb_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint8_t wMaxPacketSizeLow;
	uint8_t wMaxPacketSizeHigh;
	uint8_t bInterval;
} usb_endpoint_descriptor;



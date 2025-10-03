#include "usb/public.h"

// This header contains the device, configuration, interface and endpoint descriptors for the USBI2C converter.

usb_device_descriptor device_descriptor = {
    .bLength = sizeof(usb_device_descriptor),
    .bDescriptorType = USB_DEVICE_DESCRIPTOR_TYPE,
    .bcdUSB = 0x0200, // USB 2.0
    .bDeviceClass = 0xFF,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 0x40,
    .idVendor = 0x0A05,
    .idProduct = 0x3748,
    .bcdDevice = 0x0100,
    .iManufacturer = 0, // String descriptors not supported, so set all of them to 0.
    .iProduct = 0,
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};

typedef struct _test_cfg_descriptor {
    usb_configuration_descriptor cfg_descriptor;
    usb_interface_descriptor interface_descriptor;
    usb_endpoint_descriptor endpoint_descriptors[2];
} test_cfg_descriptor;

test_cfg_descriptor cfg_descriptor = {
    .cfg_descriptor = {
        .bLength = 9,
        .bDescriptorType = USB_CFG_DESCRIPTOR_TYPE,
        .wTotalLengthLow = sizeof(test_cfg_descriptor),
        .wTotalLengthHigh = 0,
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0, // No string describing configuration.
        .bmAttributes = 0b10000000,
        .bMaxPower = 50, // 100mA (todo check)
    },
    .interface_descriptor = {
        .bLength = sizeof(usb_interface_descriptor),
        .bDescriptorType = USB_INTERFACE_DESCRIPTOR_TYPE,
        .bInterfaceNumber = 0,
        .bAlternativeSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0xFF,
        .bInterfaceSubClass = 0x50,
        .bInterfaceProtocol = 0x44,
        .iInterface = 0,
    },
    .endpoint_descriptors = {
        {
            .bLength = sizeof(usb_endpoint_descriptor),
            .bDescriptorType = USB_ENDPOINT_DESCRIPTOR_TYPE,
            .bEndpointAddress = 1, // Endpoint number: 1, endpoint direction: OUT
            .bmAttributes = 0b10, // Bulk endpoint
            .wMaxPacketSizeLow = 64,
            .wMaxPacketSizeHigh = 0,
            .bInterval = 0,
        },
        {
            .bLength = sizeof(usb_endpoint_descriptor),
            .bDescriptorType = USB_ENDPOINT_DESCRIPTOR_TYPE,
            .bEndpointAddress = 1 | USB_EP_ADDRESS_EP_DIRECTION_MASK, // Endpoint number: 1, endpoint direction: IN
            .bmAttributes = 0b10, // Bulk endpoint
            .wMaxPacketSizeLow = 64,
            .wMaxPacketSizeHigh = 0,
            .bInterval = 0,
        }
    }
};
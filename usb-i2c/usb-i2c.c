#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include "../shared/protocol.h"
#include "usb-i2c-ioctl.h"

// Prefix log messages with function/module name
#undef pr_fmt
#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#define BULK_PACKET_SIZE 64
// Custom error code that indicates the USB transfer ended before the required data could be read 
#define EENDOFTRANSFER 36

// Timeout (ms) before assuming the dongle is not responding.
#define BULK_TRANSFER_TIMEOUT 200
#define CONTROL_TRANSFER_TIMEOUT 100

// Virtual I2C adapter that uses a USB bridge.
struct usbi2c_adapter {
    struct i2c_adapter i2c_adap; // Need to cast usbi2c_adapter* to i2c_adapter*
    struct usb_device* dev;
    struct usb_interface* reg_to_interface;

    // Buffers used when sending data to bulk out, or reading data from bulk in.
    // Must be on a per-interface basis, since separate I2C buses might have concurrent accesses.
    char bulkOutBuf[BULK_PACKET_SIZE];
    int bulkOutBufLen; // Index of first byte not in the buffer
    char bulkInBuf[BULK_PACKET_SIZE];
    int bulkInBufLen; // Number of bytes in the current packet
    int bulkInBufCursor; // Index of next byte to read from buffer.

    // True if a short packet has been received from bulk in, terminating the transfer
    bool bulkInShortPacket;
    // True if a short packet has been sent to bulk out, terminating the transfer.
    bool bulkOutShortPacket;
    // Whenever a packet is read from bulk in, the final byte is stored here.
    // If a short packet is detected, then this holds the result of the I2C transaction.
    i2c_trans_result transResult;

    struct mutex mode_mutex;
    // Whether we have configured the i2c mode.
    bool configuredMode;
    // TRUE for fast mode, FALSE for standard mode.
    bool fastMode;
    // In kHz.
    int frequency;
};

static struct usb_driver door_driver;

static int usbi2c_usbfileopen(struct inode* i, struct file* f) {
    int subminor = iminor(i);

    struct usb_interface* inter = usb_find_interface(&door_driver, subminor);
    if(!inter) {
        pr_err("usb_find_interface didn't find interface!\n");
        return -ENODEV;
    }
    
    struct usbi2c_adapter* usbi2c = usb_get_intfdata(inter);
    if(!usbi2c) {
        pr_err("No usbi2c_adapter associated with interface..\n");
        return -ENODEV;
    }

    f->private_data = usbi2c;
    return 0;
}

static int usbi2c_usbfilerelease(struct inode *, struct file *) {
    return 0;
}

static long usbi2c_unlocked_ioctl(struct file * f, unsigned command, unsigned long data) {
    struct usbi2c_adapter* usbi2c = f->private_data;

    if(command == IOCTRL_CONFIGURE_USBI2C) {
        struct configure_i2c_data config_data;
        if(copy_from_user(&config_data, (void*) data, sizeof(struct configure_i2c_data))) {
            return -EFAULT;
        }

        pr_info("Changing I2C bus configuration: fast mode: %s, frequency: %d\n",
            config_data.fastMode ? "TRUE" : "FALSE",
            config_data.frequency);
        mutex_lock(&usbi2c->mode_mutex);
        usbi2c->configuredMode = FALSE;
        usbi2c->fastMode = config_data.fastMode ? 1 : 0;
        usbi2c->frequency = config_data.frequency;
        mutex_unlock(&usbi2c->mode_mutex);
        
        return 0;
    }   else    {
        return -EINVAL;
    }
}

// File operations valid on the USB device file for the driver.
static struct file_operations fops = {
    .open = usbi2c_usbfileopen,
    .release = usbi2c_usbfilerelease,
    .unlocked_ioctl = usbi2c_unlocked_ioctl
};

// Information about the USBI2C driver.
// This is shared between all interfaces.
static struct usb_class_driver class = {
    .name = "usb/usbi2c-adap%d",
    .fops = &fops,
};

static void usbi2c_disconnect(struct usb_interface* interface);

// Sends a message to the dongle to configure the i2c mode, only if `adap->configuredMode` is FALSE.
// Uses the currently selected mode in the adapter.
// If `force` is TRUE, a message will be sent regardless of `adap->configuredMode`.
//
// Changing the I2C mode also resets the I2C, so if the device isn't responding for some reason,
// it is appropriate to call this function. If configuring the i2c mode fails, the USBI2C adapter
// will be removed.
// Returns a nonnegative value upon success.
static int usbi2c_configurei2c(struct usbi2c_adapter* adap, bool force) {
    mutex_lock(&adap->mode_mutex);
    bool alreadyConfigured = adap->configuredMode;
    bool fastMode = adap->fastMode;
    int frequency = adap->frequency;
    adap->configuredMode = TRUE;
    mutex_unlock(&adap->mode_mutex);

    if(alreadyConfigured && !force) {
        return 0;
    }

    // Cap to max frequency depending on mode.
    if(fastMode && frequency > 400) frequency = 400;
    if(!fastMode && frequency > 100) frequency = 100;

    pr_debug("Configuring fast mode: %s, freq (limited): %dkHz\n", fastMode ? "TRUE" : "FALSE", frequency);

    uint16_t value = (fastMode ? I2C_MODE_REQUEST_WVALUE_MODE_MASK : 0) | 
        I2C_MODE_REQUEST_WVALUE_SET_FREQUENCY(frequency);

    int retVal = usb_control_msg_send(adap->dev,
        0,
        I2C_MODE_B_REQUEST,
        I2C_MODE_BM_REQUEST_TYPE,
        value,
        0,
        NULL,
        0,
        CONTROL_TRANSFER_TIMEOUT,
        0);

    if(retVal < 0) {
        pr_err("usb_control_msg_send failed with return %d\n"
                " The USBI2C dongle is not responding to requests...\n", retVal);
    }

    return retVal;
}

// Empties the bulk out buffer on the given adapter.
// This will send a zero-length packet if the buffer is already empty.
static int usbi2c_empty_bulkoutbuf(struct usbi2c_adapter* adap) {
    if(adap->bulkOutShortPacket) {
        pr_debug("skipped - transfer already over\n");
        return 0;
    }

    // Ensure only one packet terminating the transfer is sent.
    if(adap->bulkOutBufLen < BULK_PACKET_SIZE) {
        adap->bulkOutShortPacket = TRUE;
    }

    int lengthTransferred;
    int retVal = usb_bulk_msg(adap->dev,
        usb_sndbulkpipe(adap->dev, BULK_EP_OUT),
        &adap->bulkOutBuf[0],
        adap->bulkOutBufLen,
        &lengthTransferred,
        BULK_TRANSFER_TIMEOUT);

    if(retVal >= 0) {
        // Should always be able to send a full packet (64 bytes for our full-speed USBI2C converter) in one go.
        // If the buffer didn't get sent in one packet, this is an error.
        if(lengthTransferred != adap->bulkOutBufLen) {
            return -EIO;
        }
        adap->bulkOutBufLen = 0;
    }   else if(retVal == -ETIMEDOUT) {
        pr_err("usbi2c adapter didn't respond for %dms - is it malfunctioning!\nTrying a reset\n", BULK_TRANSFER_TIMEOUT);
        usbi2c_configurei2c(adap, TRUE);
    }

    return retVal;
}


// Copies the given data to the bulk out buffer.
// If the buffer fills up, a packet of data will be sent.
// This continues until all data is buffered or transmitted.
static int usbi2c_sendtobulkout(struct usbi2c_adapter* adap, char* data, int length) {
    int retVal = 0;
    // Send USB packets until `length` bytes are sent.
    while(length > 0) {
        int spaceInBuf = BULK_PACKET_SIZE - adap->bulkOutBufLen;
        int bytesToCopy = length > spaceInBuf ? spaceInBuf : length;
        memcpy(&adap->bulkOutBuf[adap->bulkOutBufLen], data, bytesToCopy);
        data += bytesToCopy;
        adap->bulkOutBufLen += bytesToCopy;
        length -= bytesToCopy;

        if(adap->bulkOutBufLen == BULK_PACKET_SIZE) {
            retVal = usbi2c_empty_bulkoutbuf(adap);
            if(retVal < 0) {
                return retVal;
            }
        }
    }

    return retVal;
}

// Fills the bulk in buffer.
// If the previous packet received was a short packet, this will return -EENDOFTRANSFER (since the I2C transaction isn't going to send anymore data)
// If the packet is short (lower in length than 64 bytes), then this sets the flag `bulkInShortPacket` in `adap`.
// `transResult` is also updated with the last byteof the packet
static int usbi2c_fillbulkinbuffer(struct usbi2c_adapter* adap) {
    if(adap->bulkInShortPacket) {
        return -EENDOFTRANSFER;
    }

    int retVal = usb_bulk_msg(adap->dev,
        usb_rcvbulkpipe(adap->dev, BULK_EP_IN),
        &adap->bulkInBuf[0],
        BULK_PACKET_SIZE,
        &adap->bulkInBufLen,
        BULK_TRANSFER_TIMEOUT);
    adap->bulkInBufCursor = 0;

    if(retVal < 0) {
        pr_err("usb_bulk_msg failed: %d\n", retVal);

        if(retVal == -ETIMEDOUT) {
            pr_err("usbi2c adapter didn't respond for %dms - is it malfunctioning!\nTrying a reset\n", BULK_TRANSFER_TIMEOUT);
            usbi2c_configurei2c(adap, TRUE);
        }
    }   else    {
        if(adap->bulkInBufLen < BULK_PACKET_SIZE) {
            adap->bulkInShortPacket = true;
        }
        if(adap->bulkInBufLen > 0) {
            adap->transResult = adap->bulkInBuf[adap->bulkInBufLen - 1];
        }
        adap->bulkInBufCursor = 0;
    }

    return retVal;
}

// Reads `length` bytes from bulk in into `data`.
// Returns -EIO if the transfer ends before `length` bytes could be read.
static int usbi2c_readfrombulkin(struct usbi2c_adapter* adap, char* data, int length) {
    while(length) {
        int bytesLeftInBuf = adap->bulkInBufLen - adap->bulkInBufCursor;
        int bytesToCopy = length < bytesLeftInBuf ? length : bytesLeftInBuf;

        memcpy(data, &adap->bulkInBuf[adap->bulkInBufCursor], bytesToCopy);
        length -= bytesToCopy;
        data += bytesToCopy;
        adap->bulkInBufCursor += bytesToCopy;

        // If still more to read, fetch another packet
        if(length) {
            int retVal = usbi2c_fillbulkinbuffer(adap);
            if(retVal < 0) {
                pr_err("usbi2c_fillbulkinbuffer failed\n");
                return retVal;
            }
        }
    }

    return 0;
}

// Keeps reading bytes from bulk in until it receives a short packet (indicating the end of the transfer)
static int usbi2c_ensurebulkinendoftransfer(struct usbi2c_adapter* adap) {
    int retVal;
    while(!adap->bulkInShortPacket) {
        retVal = usbi2c_fillbulkinbuffer(adap);
        if(retVal < 0) {
            pr_err("usbi2c_fillbulkinbuffer failed\n");
            return retVal;
        }
    }

    return 0;
}

// Returns an appropriate error code representing `result`.
static int usbi2c_handletransresult(i2c_trans_result result) {
    switch(result) {
        case I2CResultSuccess:
            return 0;
        case I2CResultAddressNoAck:
            return -ENXIO;
        // TODO: I can't seem to find a better error code than `EIO` for any of the remaining errors.
        default:
            return -EIO;
    }
}

// Writes the given i2c_msg and the message contents (if it is a writing message) to bulk out.
// Returns 0 on success if the message is writing, else returns the number of bytes the message
// will read if it is reading.
// A negative return indicates failure.
static int usbi2c_writemsgdesc(struct usbi2c_adapter* adap, struct i2c_msg toSend) {
    int retVal = 0;

    i2c_section_header header;
    
    bool isRead = toSend.flags & I2C_M_RD;
    // Upper 7 bits is the address, lower bit is the read/write flag.
    header.address = toSend.addr << 1 | isRead;
    header.num_bytes_low = toSend.len & 0xFF;
    header.num_bytes_high = toSend.len >> 8;

    // Send message header
    retVal = usbi2c_sendtobulkout(adap, (char*) &header, sizeof(i2c_section_header));
    if(retVal < 0) {
        pr_err("usbi2c_sendtobulkout failed: %d\n", retVal);
        return retVal;
    }

    pr_debug("Sending msg len: %d, is read: %d\n", toSend.len, isRead);

    if(isRead) {
        return toSend.len;
    }   else    {
        retVal = usbi2c_sendtobulkout(adap, toSend.buf, toSend.len);
        if(retVal < 0) {
            pr_err("usbi2c_sendtobulkout failed: %d", retVal);
            return retVal;
        }   else    {
            return 0;
        }
    }
    
}


// Prepares the adapter for a new bulk transfer which represents a new I2C transaction.
static void usbi2c_reset_bulk_transfer(struct usbi2c_adapter* adap) {
    adap->transResult = I2CResultSuccess;
    adap->bulkInShortPacket = FALSE;
    adap->bulkInBufLen = 0;
    adap->bulkInBufCursor = 0;
    adap->bulkOutBufLen = 0;
    adap->bulkOutShortPacket = FALSE;
}



static int usbi2c_xfer(struct i2c_adapter* adap, struct i2c_msg *msgs, int num_msgs) {
    int retVal = 0;
    struct usbi2c_adapter* usbi2c = (struct usbi2c_adapter*) adap;
    usbi2c_reset_bulk_transfer(usbi2c);
    pr_debug("entry\n");

    retVal = usbi2c_configurei2c(usbi2c, FALSE);
    if(retVal < 0) {
        pr_err("usbi2c_configurei2c failed: %d", retVal);
        return retVal;
    }

    if(num_msgs > U16_MAX) {
        pr_err("Transfer had %d messages, but the max is %d\n", num_msgs, U16_MAX);
        return -ENOTSUPP;
    }
    // No need to verify message length since we use two bytes for length,
    // and so does the Linux API.

    i2c_trans_header trans_header;
    trans_header.num_sections_low = num_msgs & 0xFF;
    trans_header.num_sections_high = num_msgs >> 8;
    retVal = usbi2c_sendtobulkout(usbi2c, (char*) &trans_header, sizeof(i2c_trans_header));
    if(retVal < 0) {
        pr_err("usbi2c_sendtobulkout failed: %d\n", retVal);
        goto exit;
    }

    int nextSendMsgIndex = 0;
    int nextRecvMsgIndex = 0;
    while(nextSendMsgIndex < num_msgs) {
        int numBytesPendingRead = 0;
        // Keep sending I2C messages until we definitely have enough bytes to receive a packet.
        while(numBytesPendingRead < BULK_PACKET_SIZE && nextSendMsgIndex < num_msgs)    {
            struct i2c_msg toSend = msgs[nextSendMsgIndex];
            nextSendMsgIndex++;

            retVal = usbi2c_writemsgdesc(usbi2c, toSend);
            if(retVal >= 0) {
                numBytesPendingRead += retVal;
            }   else    {
                pr_err("usbi2c_writemsgdesc failed: %d\n", retVal);
                goto exit;
            }
        }

        // Empty bulk out buffer if we have sent all messages.
        if(nextSendMsgIndex == num_msgs) {
            pr_debug("emptying bulk out buf\n");
            retVal = usbi2c_empty_bulkoutbuf(usbi2c);
            if(retVal < 0) {
                pr_err("usbi2c_empty_bulkoutbuf failed: %d\n", retVal);
                goto exit;
            }
        }

        // Read until the number of bytes pending read is less than the packet size.
        // Alternatively, if we've already sent all messages, we don't care how many bytes are pending a read.
        while((numBytesPendingRead >= BULK_PACKET_SIZE || nextSendMsgIndex == num_msgs) && nextRecvMsgIndex < num_msgs) {
            struct i2c_msg toRecv = msgs[nextRecvMsgIndex];
            nextRecvMsgIndex++;
            if(!(toRecv.flags & I2C_M_RD)) continue;

            numBytesPendingRead -= toRecv.len;
            retVal = usbi2c_readfrombulkin(usbi2c, toRecv.buf, toRecv.len);
            if(retVal == -EENDOFTRANSFER) {
                // Reassign result based on the error code from the dongle, which is the last byte received.
                if(usbi2c->transResult != I2CResultSuccess) {
                    retVal = usbi2c_handletransresult(usbi2c->transResult);
                }
                goto exit;
            }   else if(retVal < 0) {
                goto exit;
            }
        }
    }
    
    // If all required data was read, we must read the final byte directly, in-case it hasn't
    // been read yet.
    retVal = usbi2c_readfrombulkin(usbi2c, (char*) &usbi2c->transResult, 1);
    if(retVal >= 0) {
        pr_debug("Trans result: %d\n", usbi2c->transResult);
        retVal = usbi2c_handletransresult(usbi2c->transResult);
    }   else    {
        pr_err("usbi2c_readfrombulkin failed when reading transaction result\n");
    }

exit:
    // Empty bulk out buffer. Does nothing if buffer was already emptied.
    int subretVal = usbi2c_empty_bulkoutbuf(usbi2c);
    if(subretVal < 0) pr_err("usbi2c_empty_bulkoutbuf failed: %d\n", subretVal);

    // Ensures we've read the final packet in the bulk in transfer, "just in case" we haven't due to an error.
    // This is normally guaranteed, unless we made some kind of error on our end, 
    // since errors normally cause *less* data than we would expect from bulk in.
    subretVal = usbi2c_ensurebulkinendoftransfer(usbi2c);
    if(subretVal < 0) pr_err("usbi2c_ensurebulkinendoftransfer failed: %d\n", subretVal);

    pr_debug(KERN_INFO "retval: %d\n", retVal);
    return retVal < 0 ? retVal : num_msgs;
}


static u32 usbi2c_functionality(struct i2c_adapter* adap) {
    // We only support I2C and the SMBUS commands that the kernel can emulate using I2C.
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm usbi2c_algorithm = {
    .xfer = usbi2c_xfer,
    .functionality = usbi2c_functionality
};

// Checks that the given `usb_interface` has the necessary endpoints for USBI2C.
// Returns TRUE if so.
static bool usbi2c_verifyinterface(struct usb_interface* interface) {
    pr_debug("entry\n");

    struct usb_host_interface *host_inter = interface->cur_altsetting;

    bool gotBulkIn = FALSE;
    bool gotBulkOut = FALSE;
    for(int i = 0; i < host_inter->desc.bNumEndpoints; i++) {
        struct usb_endpoint_descriptor epDescriptor = host_inter->endpoint[i].desc;

        if((epDescriptor.bmAttributes & 3) == USB_ENDPOINT_XFER_BULK) {
            if(epDescriptor.bEndpointAddress == BULK_EP_OUT) {
                gotBulkOut = TRUE;
            }
            if(epDescriptor.bEndpointAddress == BULK_EP_IN) {
                gotBulkIn = TRUE;
            }
        }
    }

    pr_info("Had bulk out: %d, had bulk in: %d\n", gotBulkOut, gotBulkIn);
    return gotBulkIn && gotBulkOut;
}

static int usbi2c_probe(struct usb_interface *interface, const struct usb_device_id* id) {
    int retval = 0;
    struct usbi2c_adapter* usbi2c = NULL;
    bool devRegistered = false;
    struct usb_device* device = interface_to_usbdev(interface);

    pr_debug("entry\n");

    if(!usbi2c_verifyinterface(interface)) {
        pr_err("usbi2c_verifyinterface FAILED: missing one or more endpoints\n");
        return -ENODEV; // Not a USBI2C.
    }
    
    // Indicate to the USB stack that we have claimed the interface.    
    retval = usb_register_dev(interface, &class);
    if(retval < 0) {
        pr_err("Unable to get a minor for the device.\n");
        goto error;
    }
    devRegistered = true;


    usbi2c = kmalloc(sizeof(struct usbi2c_adapter), 0);
    if(!usbi2c) {
        retval = -ENOMEM;
        goto error;
    }

    usb_set_intfdata(interface, usbi2c);
    
    usbi2c->i2c_adap.owner = THIS_MODULE;
    // No specific class for the devices on the bus - we are a generic usbi2c adapter.
    usbi2c->i2c_adap.class = 0;
    usbi2c->i2c_adap.algo = &usbi2c_algorithm;
    usbi2c->i2c_adap.dev.parent = &device->dev;
    usbi2c->dev = device;
    usbi2c->reg_to_interface = interface;
    usbi2c->configuredMode = FALSE;
    usbi2c->fastMode = TRUE;
    usbi2c->frequency = 400;
    mutex_init(&usbi2c->mode_mutex);

    snprintf(usbi2c->i2c_adap.name,
        sizeof(usbi2c->i2c_adap.name),
        "USBI2C converter %d", interface->minor);

    // The `i2c_adapter` is the first field so this cast is valid.
    retval = i2c_add_adapter((struct i2c_adapter*) usbi2c);
    if(retval < 0) {
        pr_err("Unable to register i2c bus driver\n");
        goto error;
    }

    // Associate the `usbi2c` with the interface so that in the `disconnect` event we know which i2c bus to remove.
    usb_set_intfdata(interface, usbi2c);

    pr_info("Registered i2c bus driver at i2c-%d\n", usbi2c->i2c_adap.nr);
    return retval;

error:
    if(devRegistered) {
        usb_deregister_dev(interface, &class);
    }
    if(usbi2c) {
        kfree(usbi2c);
    }
    // Tell `door_disconnect` that there's no associated I2C bus.
    usb_set_intfdata(interface, NULL);

    return retval;
}

static void usbi2c_disconnect(struct usb_interface* interface) {
    pr_debug("usbi2c_disconnect\n");
    usb_deregister_dev(interface, &class);

    // Deregister the I2C adapter
    struct usbi2c_adapter* usbi2c = usb_get_intfdata(interface);
    usb_set_intfdata(interface, NULL);
    
    if(usbi2c) {
        pr_info("deleting i2c adapter i2c-%d\n", usbi2c->i2c_adap.nr);
        i2c_del_adapter((struct i2c_adapter*) usbi2c);
        kfree(usbi2c);
    }
}

static struct usb_device_id usbi2c_device_id_table[] = {
    { USB_DEVICE(0x0a05, 0x3748) },
    {} // Terminates the list
};

static struct usb_driver door_driver = {
    .name = "door_driver",
    .id_table = usbi2c_device_id_table,
    .probe = usbi2c_probe,
    .disconnect = usbi2c_disconnect
};

static int __init door_init(void) {
    return usb_register(&door_driver);
}

static void __exit door_exit(void) {
    usb_deregister(&door_driver);
}

module_init(door_init);
module_exit(door_exit);

MODULE_LICENSE("GPL");
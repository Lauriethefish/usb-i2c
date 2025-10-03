#pragma once

/// Structures which describe the protocol on the USB control endpoint (0).
/// Endpoint 0 can be used to set the I2C mode (standard mode or fast mode), as
/// well as controlling the bus frequency in each mode.

// A setup packet is sent which describes the I2C mode/frequency.
#define I2C_MODE_BM_REQUEST_TYPE 0b01000000
#define I2C_MODE_B_REQUEST 26

// The request `wValue` contains the I2C mode and frequency.
// The LSB has the I2C mode and the remaining bits hold the I2C frequency in kHz.

#define I2C_MODE_REQUEST_WVALUE_MODE_MASK 1
#define I2C_MODE_REQUEST_WVALUE_GET_FREQUENCY(wValue) (wValue >> 1)
#define I2C_MODE_REQUEST_WVALUE_SET_FREQUENCY(wValue) (wValue << 1)

/// Structures which describe the protocol on the USB bulk endpoints that allows I2C transfers.

// I2C transactions start with a "start condition" and finish with a "stop condition".
// To be more precise, an I2C transaction consists of some number of "sections" (at least 1), each starting with a START condition and an address.
// In each section, some number of bytes are read or written.
// We describe each I2C transaction using a USB bulk out transfer.

// The first two bytes in the transfer give the number of I2C sections (little endian).
typedef struct _i2c_trans_header {
    uint8_t num_sections_low;
    uint8_t num_sections_high;
} i2c_trans_header;

// Header for each section of the I2C transaction: a section may read or write some number of bytes.
// Each section sends a start condition and address.
typedef struct _i2c_section_header {
    // Address of the device to read from/write to.
    // I2C addresses are 7 bits. The uppermost 7 bits of this field hold the address, and the lowest bit decides whether we are
    // reading or writing (1 in the case of a read, 0 for a write)
    uint8_t address;
    // Number of bytes to read or write, as 16 bit integer.
    uint8_t num_bytes_low;
    uint8_t num_bytes_high;

    // In the case of a write, this structure is followed by the bytes to be written.
} i2c_section_header;
// The end of the USB bulk OUT transfer will generate a stop condition.

// The data read from BULK IN is the data read by i2c_section_header structures.
// The data from each I2C transaction is sent using one bulk in transfer. After the final byte of data read, a transaction
// result follows, which is one of the codes below:

typedef enum _i2c_trans_result {
    I2CResultSuccess,
    // Sent when no I2C slave ACKs when sending a device address
    // Normally indicates that no device is at the specified address. 
    I2CResultAddressNoAck,
    // Sent when an I2C device fails to ACK for a data byte. (after the address phase.)
    I2CResultDataNoAck,
    // Another master took control of the bus
    I2CResultArbitrationLost,
    I2CResultInternalError,
    // Misplaced start/stop condition occurs.
    I2CResultBusError,
    // The USB host did not supply data fast enough for the I2C bus.
    I2CResultHostTooSlow,
    // Host tried to start an I2C transaction but the I2C hadn't yet been configured.
    I2CResultNotConfigured,
} i2c_trans_result;

// One BULK IN transfer is used to send all the data for a transaction back to the host.
// The data from each reading I2C section is concatenated.

// Bulk endpoint addresses for USBI2C interface.
#define BULK_EP_OUT 0x01
#define BULK_EP_IN 0x81

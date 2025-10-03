#include "i2c-master.h"

#include "cortex.h"
#include "stm32.h"
#include "interrupts.h"
#include "../../shared/protocol.h"
#include "debug.h"
#include "usb_handlers.h"
#include "delay.h"

BOOL i2cModeConfigured;

// Number of sections remaining in the current transaction.
int numSectionsRemaining;

// TRUE iff the current i2c section is a read.
BOOL sectionIsRead;
// Number of bytes remaining to read/write in the current section.
int sectionBytesLeft;
// Slave being accessed in the current section, including read/write bit as the LSB.
uint8_t sectionSlaveAddress;

// True if the address of the slave has been sent during the current section.
BOOL addressSent;

i2c_trans_result transResult;


void I2C2_DisableInterrupts() {
    *NVIC_ISER1 &= ~(I2C2_ER_MASK | I2C2_EV_MASK);
}

void I2C2_EnableInterrupts() {
    *NVIC_ISER1 |= I2C2_ER_MASK | I2C2_EV_MASK;
}

// Initialises the I2C2 registers for fast or standard mode communication, assuming the APB1 clock is 36MHz.
// `frequency` is in kHz.
void I2CMaster_ConfigureI2C2Registers(BOOL fastMode, int frequency) {
    // Set B10 and B11 to pull-up
    I2C2->CR2 = 36; // APB1 is set to 36MHz
    dbgprintf("I2CMaster_ConfigureI2C2Registers entry. Fast mode: %d, freq: %d\n", fastMode, frequency);

    // FREQ is 36, but we want a 400kHz clock.
    // 36,000/400 = 90

    int ccrVal = 0;
    int ccrDivFactor;
    if(fastMode) {
        frequency = frequency > 400 ? 400 : frequency;
        // The CCR register is multiplied by the period
        // of the 36MHz clock to get the period of the I2C clock.
        // For a full cycle of SCL, 3 I2C clock pulses are needed. (as we set `I2C_CCR_FM_DUTY_CYCLE_FACTOR_2`)
        // So, if CCR = 1, then the frequency is 12MHz.
        ccrDivFactor = 12000 / frequency;
        ccrVal |= I2C_CCR_FM_DUTY_CYCLE_FACTOR_2 | I2C_CCR_MASTER_MODE_FM;
    }   else    {
        // In this case, for a full cycle of SCL, only 2 I2C clock pulses are needed.
        // So, if CCR = 1, the frequency is 18MHz
        ccrDivFactor = 18000 / frequency;
        frequency = frequency > 100 ? 100 : frequency;
    }
    if(ccrDivFactor > 4095) {
        dbgprintf("Given frequency of %d was too low - capping to lower limit\n", frequency);
        ccrDivFactor = 4095;
    }

    ccrVal |= ccrDivFactor;
    I2C2->CCR = ccrVal;

    // Measured in the number of clock cycles of 36MHz + 1
    // Each clock cycle of 36MHz is 27ns.
    if(fastMode) {
        // 300ns rise time for fast mode.
        // 300/27.77 is 10.8. 10.8 + 1 = 11.8. Taking the integer part: 11.
        I2C2->TRISE = 11;
        //I2C2->TRISE = 9;
    }   else    {
        // 1000ns rise time for standard mode.
        // 1000/27.77.. = 36. 36 + 1 = 37.
        I2C2->TRISE = 37;
    }

    // Enable interrupts for events/errors
    I2C2->CR2 |= I2C_CR2_ITERREN;
    I2C2->CR2 |= I2C_CR2_ITEVTEN;
    I2C2->CR2 |= I2C_CR2_ITBUFEN;
    I2C2_EnableInterrupts();
}

// Sets up the AFIO pins for I2C2
void I2CMaster_ConfigureI2C2ClocksAndAFIO() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // We need to set pins 10 and 11 to open-drain
    // These are found in CRH at indexes 2 and 3
    GPIO_B->CRH &= CLEAR_PORT_CFG_MASK(2);
    GPIO_B->CRH &= CLEAR_PORT_CFG_MASK(3);

    // Configure the I2C pins for open drain communication using an alternate function (I2C2)
    GPIO_B->CRH |= SET_PORT_CNF_MASK(2, AF_OUTPUT_OPEN_DRAIN)
        | SET_PORT_CNF_MASK(3, AF_OUTPUT_OPEN_DRAIN);

    GPIO_B->CRH |= SET_PORT_MODE_MASK(2, OUTPUT_MODE_MAX_50_MHz)
        | SET_PORT_MODE_MASK(3, OUTPUT_MODE_MAX_50_MHz);
}

void I2CMaster_Init() {
    I2CMaster_ConfigureI2C2ClocksAndAFIO();
}

// Called once the current transaction comes to an end, either due to the bulk out transfer finishing,
// or due to an error.
void I2CMaster_OnEndOfTransaction() {    
    // Write transaction result.
    dbgprintf("Trans result: %d\n", transResult);
    if(!USBHandlers_WriteToBulkInBuffer((char*) &transResult, 1)) {
        dbgprintf("Host didn't read data from bulk in fast enough\n");
        // Can't really do anything about this situation.
        // Host isn't going to be able to read the status of the transfer, so we can't salvage the connection.
        // We're going to NACK all future packets by returning - eventually the host will timeout,
        // and reconfigure the I2C mode, which will reset everything to a known state.
        return;
    }

    // Flush pending data to bulk in, ensure the rest of the transaction has been read from bulk in if this is an error condition.
    dbgprintf("Transaction finished\n");
    USBHandlers_CompleteTransfer();
}

// Upon most error conditions, we can't continue the transfer.
// This function aborts the transfer and sends the error code back to the host.
void I2CMaster_AbortTransfer() {    
    dbgprintf("USBI2C_AbortTransfer\n");
    // Wait for any stop or start conditions to clear
    while(I2C2->CR1 & (I2C_CR1_STOP | I2C_CR1_START)) { asm volatile(""); }
    I2CMaster_OnEndOfTransaction();
}

// Attempts to start the next section in the I2C transaction, whether that's a read or a write.
// Returns FALSE if the transfer has ended.
BOOL I2CMaster_StartNextSection() {
    i2c_section_header sectionHeader;
    // Try to read the section header from bulk out.
    int status = USBHandlers_ReadFromBulkOutBuffer((char*) &sectionHeader, sizeof(i2c_section_header));
    // TODO: should probably replace with an enum.
    if(status == 1) {
        I2CMaster_OnEndOfTransaction();
        return FALSE;
    }   else if(status == 2) {
        allow_blocking_immediately();
        dbgprintf("USB host was too slow.. aborting current transfer\n");
        transResult = I2CResultHostTooSlow;
        I2CMaster_AbortTransfer();
    }

    sectionIsRead = sectionHeader.address & 1;
    sectionSlaveAddress = sectionHeader.address;
    sectionBytesLeft = sectionHeader.num_bytes_high << 8 | sectionHeader.num_bytes_low;
    
    dbgprintf("Started I2C section, reading: %d, bytes: %d, address: %d. CR1: %w\n", sectionIsRead, sectionBytesLeft, sectionSlaveAddress, I2C2->CR1);
    numSectionsRemaining--;

    if(sectionIsRead) {
        // Special case per optimised examples when reading exactly 2 bytes
        if(sectionBytesLeft == 2) {
            I2C2->CR1 |= I2C_CR1_ACK | I2C_CR1_POS;
        }   else    {
            I2C2->CR1 |= I2C_CR1_ACK;
            I2C2->CR1 &= ~I2C_CR1_POS; // Clear POS if we're not reading precisely two bytes.
        }
    }

    // Generate start condition
    I2C2->CR1 |= I2C_CR1_PE;
    I2C2->CR1 |= I2C_CR1_START;
    return TRUE;
}

void USBHandlers_StartBulkOutTransfer() {

    i2c_trans_header trans_header;
    if(USBHandlers_ReadFromBulkOutBuffer((char*) &trans_header, sizeof(i2c_trans_header)) != 0) {
        allow_blocking_immediately();
        dbgprintf("Insufficient data in transaction\n");
        transResult = I2CResultHostTooSlow;
        I2CMaster_AbortTransfer();
    }
    numSectionsRemaining = (trans_header.num_sections_high << 8) | trans_header.num_sections_low;

    if(!i2cModeConfigured) {
        dbgprintf("Host tried an i2c transaction but I2C wasn't configured\n");
        transResult = I2CResultNotConfigured;
        I2CMaster_AbortTransfer();
    }

    dbgprintf("Starting new transfer, sections: %d\n", numSectionsRemaining);
    transResult = I2CResultSuccess;

    prevent_blocking();
    I2CMaster_StartNextSection();
}

void USBHandlers_ConfigureI2CMode(BOOL fastMode, int frequency) {
    // Power cycle the I2C peripheral - aborting any current transactions.
    RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;


    I2CMaster_ConfigureI2C2Registers(fastMode, frequency);

    numSectionsRemaining = 0;
    sectionBytesLeft = 0;
    sectionIsRead = FALSE;
    sectionSlaveAddress = 0;
    addressSent = FALSE;
    transResult = I2CResultSuccess;
    i2cModeConfigured = TRUE;
}

// Writes the next data value to the DR during transmission.
BOOL I2CMaster_WriteNextData() {
    int timeref = get_time_ref();
    // NOTE: This is really the only time critical code path in the binary.
    // If the bulk in buffer is empty, it has to copy from the USB SRAM to the buffer
    // (64 bytes). This can take some time and we only have 180 clock cycles to write the next byte.
    if(sectionBytesLeft) {
        char dataByte;
        int status = USBHandlers_ReadFromBulkOutBuffer(&dataByte, 1);
        I2C2->DR = dataByte;
        int after = get_time_ref();
        //dbgprintf("Took: %d\n", (timeref - after) << 3);

        // TODO: Potential issue here where if `USBHandlers_ReadFromBulkOutBuffer` needs to copy one buffer to another
        // it could take too long for the next I2C bit.
        // In theory to write the next byte we have quite a few clock cycles, but monitor this closely.
        // Update: It seems we're cutting it rather fine without optimisations, but we definitely _DO_ have enough time.
        if(status != 0) {
            dbgprintf("Host did not send data fast enough\n");
            transResult = I2CResultHostTooSlow;
            I2CMaster_AbortTransfer();
            return FALSE;
        }
        //dbgprintf("WriteNextData\n");
        sectionBytesLeft--;
    }
}

// Reads the next data value from DR, ignoring it if the buffer is full.
// Should only be called if `reading` is TRUE.
void I2CMaster_ReadNextData() {
    if(sectionBytesLeft) {
        sectionBytesLeft--;
        char dataByte = I2C2->DR;
        if(!USBHandlers_WriteToBulkInBuffer(&dataByte, 1)) {
            dbgprintf("Insufficient room in buffer for I2C data\n");
            transResult = I2CResultHostTooSlow;
            I2CMaster_AbortTransfer();
        }
    }
}

// Handlers for I2C events.
// Due to silicon bugs in the STM32, there are lots of edge cases, specifically when handling reception of data,
// for which the required register accesses vary depending on if >=3 bytes, 2 bytes or 1 byte of data is being read.
//
// This function attempts to implement the "Optimized examples" provided by ST for each of these cases, however sometimes the examples
// have contradictions where the diagrams disagree with the text.

// Called on I2C2 start condition
void I2C2_SB() {
    I2C2->DR = sectionSlaveAddress;
}

// Called on I2C2 address sent condition
void I2C2_ADDR() {
    addressSent = TRUE;

    // Special case: if reading/writing zero bytes (I2C scanners often do this),
    // then stop immediately unless there are more sections left in the buffer.
    if(sectionBytesLeft == 0) {
        dbgprintf("Handling 0 byte transaction\n");
        I2C2->CR1 |= (numSectionsRemaining ? I2C_CR1_START : I2C_CR1_STOP);
        while(I2C2->CR1 & I2C_CR1_STOP) { asm volatile(""); }
        I2CMaster_StartNextSection();
    }

    if(sectionIsRead && sectionBytesLeft == 1) {
        // Special case per the optimised examples when reading a single byte.
        I2C2->CR1 &= ~I2C_CR1_ACK; // Clear ACK bit.
        // Clear ADDR
        int sr2 = I2C2->SR2;
        // Program STOP/START bit
        I2C2->CR1 |= (numSectionsRemaining ? I2C_CR1_START : I2C_CR1_STOP);     
        // Wait for RxE to read the data    
    }   else if(sectionIsRead && sectionBytesLeft == 2)  {
        // Special case for two bytes.
        // Clear ADDR
        int sr2 = I2C2->SR2;
        I2C2->CR1 &= ~I2C_CR1_ACK; // Clear ACK bit.
    }   else    {
        // Read SR2 to clear the ADDR bit.
        int sr2 = I2C2->SR2;
        if(!sectionIsRead) {
            I2CMaster_WriteNextData();
        }
    }
}

// Called on I2C2 byte transfer finished condition
void I2C2_BTF() {
     if(sectionIsRead) {
        if(sectionBytesLeft == 3) {
            // If we have 3 more bytes left to read, per the optimised examples, upon BTF we carry out the following sequence:
            I2C2->CR1 &= ~I2C_CR1_ACK; // Clear ACK bit
            I2CMaster_ReadNextData(); // Read DataN-2 in DR: begin DataN reception
            I2C2_DisableInterrupts();

            // Program START/STOP: If we have more data to send in this transaction, give START, otherwise, give STOP
            I2C2->CR1 |= (numSectionsRemaining ? I2C_CR1_START : I2C_CR1_STOP);         
            I2CMaster_ReadNextData(); // Read DataN-1
            I2C2_EnableInterrupts(); // Wait for RxNE = 1.
        }   else if(sectionBytesLeft == 2) {
            // Special case for 2 bytes in the optimised examples:
            // Program START/STOP
            I2C2->CR1 |= (numSectionsRemaining ? I2C_CR1_START : I2C_CR1_STOP);
            // Read both data values
            I2CMaster_ReadNextData();
            I2CMaster_ReadNextData();

            // Wait for STOP to be cleared if applicable.
            while(I2C2->CR1 & I2C_CR1_STOP) { asm volatile(""); }

            I2CMaster_StartNextSection();
        }
    }

    if(!sectionBytesLeft && !sectionIsRead)   {
        if(numSectionsRemaining) {
            I2C2->CR1 |= I2C_CR1_START;
        }   else    {
            I2C2->CR1 |= I2C_CR1_STOP;
            // Seems like we must wait for the STOP to be transmitted immediately
            // Exiting this interrupt handler before STOP is transmitted causes the I2C to lock up
            // Presumably this is another erratum for the I2C that's not mentioned in the manual.
            while(I2C2->CR1 & I2C_CR1_STOP) { asm volatile(""); }
        }

        I2CMaster_StartNextSection();
    }
}

// Called on I2C2 receive buffer not empty condition
void I2C2_RxNE() {
    // As per the I2C optimized examples, we read upon RxNE if there are more than 3 bytes left,
    // Additionally, the last byte is read on the RxNE interrupt.
    if(sectionIsRead && (sectionBytesLeft > 3 || sectionBytesLeft == 1)) {
        I2CMaster_ReadNextData();

        if(sectionBytesLeft == 0) { // If we just read the final byte (byte N)
            // Wait until STOP cleared (This will do nothing if we didn't set it, e.g. if we're going to send another START condition)
            while(I2C2->CR1 & I2C_CR1_STOP) { asm volatile(""); }
            I2C1->CR1 |= I2C_CR1_ACK; // ACK = 1: enable future reception of data.

            I2CMaster_StartNextSection();
        }
    }
}

// Called on I2C2 transmit buffer empty condition
void I2C2_TxE() {
    I2CMaster_WriteNextData();
}

void I2C2_EV() {
    int sr1 = I2C2->SR1;
    if(sr1 & I2C_SR1_SB_MASK) {
        I2C2_SB();
    }   else if(sr1 & I2C_SR1_ADDR_MASK) {
        I2C2_ADDR();
    }   else if(sr1 & I2C_SR1_BTF_MASK) {
        I2C2_BTF();
    }   else if(sr1 & I2C_SR1_RxNE_MASK) {
        I2C2_RxNE();
    }   else if(sr1 & I2C_SR1_TxE_MASK) {
        I2C2_TxE();
    }
}

// Interrupt invoked on I2C error conditions
void I2C2_ER() {
    int sr1 = I2C2->SR1;
    // Detect but ignore any unimplemented SMBus interrupts
    if(sr1 & I2C_SR1_SMBALERT_MASK) {
        I2C2->SR1 &= ~I2C_SR1_SMBALERT_MASK;
        dbgprintf("Got SMBALERT but not in SMBus mode? Will ignore\n");
    }   else if(sr1 & I2C_SR1_TIMEOUT_MASK) {
        I2C2->SR1 &= ~I2C_SR1_TIMEOUT_MASK;
        dbgprintf("Got TIMEOUT but not in SMBus mode? Will ignore\n");
    }   else if(sr1 & I2C_SR1_PECERR_MASK) {
        I2C2->SR1 &= ~I2C_SR1_PECERR_MASK;
        dbgprintf("Got PECERR but not in SMBus mode? Will ignore\n");
    }   else if(sr1 & I2C_SR1_OVR_MASK) {
        I2C2->SR1 &= ~I2C_SR1_OVR_MASK;
        dbgprintf("Internal i2c error: Overrun/underrun in I2C2\n");
        transResult = I2CResultInternalError;
        I2CMaster_AbortTransfer();
    }   else if(sr1 & I2C_SR1_AF_MASK) {
        I2C2->SR1 &= ~I2C_SR1_AF_MASK;
        dbgprintf("Got AF\n");
        transResult = addressSent ? I2CResultDataNoAck : I2CResultAddressNoAck;
        I2CMaster_AbortTransfer();
    }   else if(sr1 & I2C_SR1_ARLO_MASK) {
        I2C2->SR1 &= ~I2C_SR1_ARLO_MASK;
        dbgprintf("Got ARLO\n");
        // TODO: In this case, we must re-enable the I2C as the master in the next transfer, otherwise the USBI2C will stall.
        transResult = I2CResultArbitrationLost;
        I2CMaster_AbortTransfer();
    }   else if(sr1 & I2C_SR1_BERR_MASK) {
        I2C2->SR1 &= ~I2C_SR1_BERR_MASK;
        dbgprintf("Got BERR\n");
        transResult = I2CResultBusError;
        I2CMaster_AbortTransfer();
    }
}
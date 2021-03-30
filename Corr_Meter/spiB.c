/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#include <assert.h>
#include "driverlib.h"
#include "ads131a02_corrM.h"
//#include "assert.h"
extern void Message(const char *str);

//*****************************************************************************
//! This example shows how SPI master talks to SPI slave using 3-wire mode.
//! Incrementing data is sent by the master starting at 0x01. Received data is
//! expected to be same as the previous transmission.  USCI RX ISR is used to
//! handle communication with the CPU, normally in LPM0. If high, P1.0 indicates
//! valid data reception.  Because all execution after LPM0 is in ISRs,
//! initialization waits for DCO to stabilize against ACLK.
//! ACLK = ~32.768kHz, MCLK = SMCLK = DCO ~ 1048kHz.  BRCLK = SMCLK/2
//!
//! Use with SPI Slave Data Echo code example.  If slave is in debug mode, P1.1
//! slave reset signal conflicts with slave's JTAG; to work around, use IAR's
//! "Release JTAG on Go" on slave device.  If breakpoints are set in
//! slave RX ISR, master must stopped also to avoid overrunning slave
//! RXBUF.
//!
//!                  MSP430F5438A
//!                 -----------------
//!            /|\ |                 |
//!             |  |                 |
//!    Master---+->|RST              |
//!                |                 |
//!                |             P3.0|-> Data Out (UCB0SIMO)
//!                |                 |
//!                |             P3.1|<- Data In (UCB0SOMI)
//!                |                 |
//!                |             P3.2|-> Serial Clock Out (UCA0CLK)
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SPI peripheral
//! - GPIO Port peripheral (for SPI pins)
//! - UCB0SIMO
//! - UCB0SOMI
//! - UCB0CLK
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - USCI_B0_VECTOR
//!
//*****************************************************************************
extern void DelayMs1(int Ms);
extern void hexdec_long( uint32_t count );
extern int Report(const char *pcFormat, ...);
extern volatile uint8_t Rx_buf[10];
//*****************************************************************************
//
//Specify desired frequency of SPI communication
//
//*****************************************************************************
#define SPICLK                          500000


//****************************************************************************
//
// Macros
//
//****************************************************************************

// Determine number of bytes per command based on WORD_LENGTH
#define WORD_LENGTH             (WORD_LENGTH_BITS >> 3) //24 - 00011000 >> 3 - 0011 - WORD_LENGTH is 3 byte

// Fetch the upper byte of a 16-bit word and return it as an 8-bit value
#define UPPER_BYTE(x)           ((uint8_t) ((0xFF00 & x) >> 8))

// Fetch the lower byte of a 16-bit word and return it as an 8-bit value
#define LOWER_BYTE(x)           ((uint8_t)  (0x00FF & x))

// Combine two 8-bit values into a 16-bit word
#define COMBINE_BYTES(x, y)     (((uint16_t) x << 8) | ((uint16_t) y & 0x00FF))

//****************************************************************************
//
// Internal variables
//
//****************************************************************************
// Flag to indicate if a nDRDY gpio interrupt has occurred
static volatile bool flag_nDRDY_INTERRUPT = false;

// Array used to recall last known values of the device's register map settings */
static uint8_t registerMap[NUM_REGISTERS];

// Flag to keep track of device's register lock status
// NOTE: The device defaults to locked
static bool registersLocked = true;

// Arrays to store RX and TX data for SPI communication
static uint8_t dataTx[6 * WORD_LENGTH] = { 0 };
static uint8_t dataRx[6 * WORD_LENGTH] = { 0 };

// Variable to keep track of the number of words in an SPI frame
#ifdef SET_FIXED
    static uint8_t cmdByteLength = 6 * WORD_LENGTH;
#else
    static uint8_t cmdByteLength = WORD_LENGTH;
#endif

uint8_t receiveData = 0x00;
uint8_t returnValue = 0x00;

#define ADS131A02
//*****************************************************************************
//
//! Updates the registerMap[] array to its default values.
//!
//! \fn void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADC's register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! hardware pin control or SPI software command), as is shown in this example.
//!
//! \return None.
//
//*****************************************************************************
void restoreRegisterDefaults(void)
{
#ifdef ADS131A04
    registerMap[ID_MSB_ADDRESS]         =   ID_MSB_NU_CH_4;               /* NOTE: This a read-only register */
#endif
#ifdef ADS131A02
    registerMap[ID_MSB_ADDRESS]         =   ID_MSB_NU_CH_4;               /* NOTE: This a read-only register */
#endif
    registerMap[ID_LSB_ADDRESS]         =   0x00;                         /* NOTE: REV_ID value is unknown until read */
    registerMap[STAT_1_ADDRESS]         =   STAT_1_DEFAULT;
    registerMap[STAT_P_ADDRESS]         =   STAT_P_DEFAULT;
    registerMap[STAT_N_ADDRESS]         =   STAT_N_DEFAULT;
    registerMap[STAT_S_ADDRESS]         =   STAT_S_DEFAULT;
    registerMap[ERROR_CNT_ADDRESS]      =   ERROR_CNT_DEFAULT;
    registerMap[STAT_M2_ADDRESS]        =   STAT_M2_DEFAULT & STAT_M2_DEFAULT_MASK;
    registerMap[A_SYS_CFG_ADDRESS]      =   A_SYS_CFG_DEFAULT;
    registerMap[D_SYS_CFG_ADDRESS]      =   D_SYS_CFG_DEFAULT;
    registerMap[CLK1_ADDRESS]           =   CLK1_DEFAULT;
    registerMap[CLK2_ADDRESS]           =   CLK2_DEFAULT;
    registerMap[ADC_ENA_ADDRESS]        =   ADC_ENA_DEFAULT;
    registerMap[ADC1_ADDRESS]           =   ADC1_DEFAULT;
    registerMap[ADC2_ADDRESS]           =   ADC2_DEFAULT;
    registerMap[ADC3_ADDRESS]           =   ADC3_DEFAULT;
    registerMap[ADC4_ADDRESS]           =   ADC4_DEFAULT;
}

void spi_init (void){

    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);               //Set P1.5 for slave reset

    //P3.5,4,0 option select
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 + GPIO_PIN1 + GPIO_PIN0);
    //Initialize Master
    USCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;         //SMCLK = 1.0 MHz
    param.clockSourceFrequency = UCS_getSMCLK();
    param.desiredSpiClock = SPICLK;                                 //500000 Hz
    param.msbFirst = USCI_B_SPI_MSB_FIRST;
    param.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    returnValue =  USCI_B_SPI_initMaster(USCI_B0_BASE, &param);

    if (STATUS_FAIL == returnValue){
        return;
    }

    //Enable SPI module
    USCI_B_SPI_enable(USCI_B0_BASE);

    //Enable Receive interrupt
    //USCI_B_SPI_clearInterrupt(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT);
    //USCI_B_SPI_enableInterrupt(USCI_B0_BASE,USCI_B_SPI_RECEIVE_INTERRUPT);

    //Now with SPI signals initialized, reset slave
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);

    //Wait for slave to initialize
    //__delay_cycles(100);
    DelayMs1(5);
}

void spi_send_byte(uint8_t transmitData){

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);//Set CS low
   // __delay_cycles(100);//Wait for slave to initialize
    DelayMs1(1);

    //Initialize data values
    transmitData = 0x00;

    //USCI_A0 TX buffer ready?
    //while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT)) ;

    //Transmit Data to slave
    USCI_B_SPI_transmitData(USCI_B0_BASE, transmitData);
    while(USCI_B_SPI_isBusy(USCI_B0_BASE));         //wait for SPI

}

//*****************************************************************************
//
//! Sends SPI byte on MOSI pin and captures MISO return byte value.
//!
//! \fn uint8_t spiSendReceiveByte(const uint8_t dataTx)
//!
//! \param dataTx data byte to send on MOSI pin.
//!
//! NOTE: This function is called by spiSendReceiveArrays(). If it is called
//! directly, then the nCS pin must also be directly controlled.
//!
//! \return Captured MISO response byte.
//
//*****************************************************************************
uint8_t spiSendReceiveByte(const uint8_t dataTx)
{
    //--- INSERT YOUR CODE HERE ---
    //*  This function should send and receive single bytes over the SPI.
    //*  NOTE: This function does not control the nCS pin to allow for
    //*  more programming flexibility.


    // Remove any residual or old data from the receive FIFO
    //uint32_t junk;
    //while (SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));

    // SSI TX & RX
    uint8_t dataRx;
    USCI_B_SPI_transmitData(USCI_B0_BASE, dataTx);  //Transmit Data to slave
    while(USCI_B_SPI_isBusy(USCI_B0_BASE));         //wait for SPI
    dataRx = USCI_B_SPI_receiveData(USCI_B0_BASE);  //Receive Data from slave
    while(USCI_B_SPI_isBusy(USCI_B0_BASE));         //wait for SPI

    return dataRx;
}

//*****************************************************************************
//
//! Sends the specified SPI command to the ADC (NULL, RESET, STANDBY, or WAKEUP).
//!
//! \fn uint16_t sendCommand(uint16_t opcode)
//!
//! \param opcode 16-bit SPI command.
//!
//! NOTE: Other ADC commands have their own dedicated functions to support
//! additional functionality. This function will raise an assert if used
//! with one of these commands (RREG, WREG, WREGS, LOCK,or  UNLOCK).
//!
//! \return uint16_t response byte (typically the STATUS byte).
//
//*****************************************************************************
uint16_t sendCommand(uint16_t opcode)
{
    /* Assert if this function is used to send any of the following opcodes */
    assert(OPCODE_RREG != opcode);      /* Use "readSingleRegister()"   */
    assert(OPCODE_WREG != opcode);      /* Use "writeSingleRegister()"  */
    assert(OPCODE_LOCK != opcode);      /* Use "lockRegisters()"        */
    assert(OPCODE_UNLOCK != opcode);    /* Use "unlockRegisters()"      */
    //assert(OPCODE_WREGS != opcode);

    // Build TX byte array
    dataTx[0] = UPPER_BYTE(opcode);
    dataTx[1] = LOWER_BYTE(opcode);

#ifdef SET_CRC_EN
    // Calculates the CRC IN word and adds it to the dataTx[] array
    updateCrcIn();
#endif

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);    // Set the nCS pin LOW

    // Send the opcode (and crc word, if enabled)
    int i;
    for (i = 0; i < cmdByteLength; i++)
    {
       dataRx[i] = spiSendReceiveByte(dataTx[i]);
    }

    // Set the nCS pin HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);//Set CS High

    // If OPCODE_RESET, then recall default register settings
    if (OPCODE_RESET == opcode) { restoreRegisterDefaults(); }

    // Combine response bytes and return as a 16-bit word
    uint16_t response = COMBINE_BYTES(dataRx[0], dataRx[1]);
    return response;
}

//*****************************************************************************
//
//! Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param dataTx byte array of SPI data to send on MOSI.
//!
//! \param dataRx byte array of SPI data captured on MISO.
//!
//! \param byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
{
    /*  --- INSERT YOUR CODE HERE ---
     *
     *  This function should send and receive multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the nCS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the nCS pin high (if controlled by GPIO)
     */

    // Require that dataTx and dataRx are not NULL pointers
    assert(dataTx && dataRx);

    // Set the nCS pin LOW
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);//Set CS low;

    // Send all dataTx[] bytes on MOSI, and capture all MISO bytes in dataRx[]
    int i;
    for (i = 0; i < byteLength; i++)
    {
        dataRx[i] = spiSendReceiveByte(dataTx[i]);
    }

    // Set the nCS pin HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);//Set CS low;
}

//*****************************************************************************
//
//! Sends the LOCK command and then verifies that registers are locked.
//!
//! \fn bool lockRegisters(void)
//!
//! \return boolean to indicate if registers are locked (0 = unlocked; 1 = locked)
//
//*****************************************************************************
bool lockRegisters(void)
{
    // Build TX array
    dataTx[0] = UPPER_BYTE(OPCODE_LOCK);
    dataTx[1] = LOWER_BYTE(OPCODE_LOCK);

#ifdef SET_CRC_EN
    // Calculates the CRC IN word and adds it to the dataTx[] array
    updateCrcIn();
#endif

    // [FRAME 1] Send WREG command
    spiSendReceiveArrays(dataTx, dataRx, cmdByteLength);

    // [FRAME 2] Send NULL command to verify response
    uint16_t response = sendCommand(OPCODE_NULL);

    // Verify device response and update internal flag
    if (OPCODE_LOCK == response) { registersLocked = true; }

    return registersLocked;
}

//*****************************************************************************
//
//! Sends the UNLOCK command and then verifies that registers are unlocked
//!
//! \fn bool unlockRegisters(void)
//!
//! \return boolean to indicate if registers are locked (0 = unlocked; 1 = locked)
//
//*****************************************************************************
bool unlockRegisters(void)
{
    // Build TX array
    dataTx[0] = UPPER_BYTE(OPCODE_UNLOCK);
    dataTx[1] = LOWER_BYTE(OPCODE_UNLOCK);

#ifdef SET_CRC_EN
    // Calculates the CRC IN word and adds it to the dataTx[] array
    updateCrcIn();
#endif

    // [FRAME 1] Send WREG command
    spiSendReceiveArrays(dataTx, dataRx, cmdByteLength);

    // [FRAME 2] Send NULL command to verify response
    uint16_t response = sendCommand(OPCODE_NULL);

    // Verify device response and update internal flag
    if (OPCODE_UNLOCK == response) { registersLocked = false; }

    // Return lock status
    return registersLocked;
}


//*****************************************************************************
//
//! Writes data to a single register.
//!
//! \fn void writeSingleRegister(uint8_t address, uint8_t data)
//!
//! This command will be ignored if device registers are locked.
//!
//! NOTE: This functions also performs a NULL command frame after the register
//! write command to verify the new register value.
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! \return None.
//
//*****************************************************************************
void writeSingleRegister(uint8_t address, uint8_t data)
{
    /* Assert(s) */
    assert(address < NUM_REGISTERS);        // Register address must be in range

    // (OPTIONAL) Enforce certain register settings when writing to the D_SYS_CFG register
    // to avoid having to write code that supports multiple device modes. For example...
    //      if (D_SYS_CFG_ADDRESS == address)
    //      {
    //          data = data | D_SYS_CFG_FIXED_MASK | D_SYS_CFG_CRC_EN_MASK;
    //      }

    // Save current register value
    uint8_t oldValue = registerMap[address];

    // Build TX byte array
    dataTx[0] = UPPER_BYTE(OPCODE_WREG) | address;
    dataTx[1] = data;

#ifdef SET_CRC_EN
    // Calculates the CRC IN word and adds it to the dataTx[] array
    updateCrcIn();
#endif

    // [FRAME 1] Send WREG command
    spiSendReceiveArrays(dataTx, dataRx, cmdByteLength);

    // Update register map array with new value
    // NOTE: This ensures that the NULL command in the next frame is sent correctly.
    registerMap[address] = data;

    // [FRAME 2] Send NULL command to verify WREG response
    uint16_t response = sendCommand(OPCODE_NULL);

    // Verify device response
    if (UPPER_BYTE(response) == (UPPER_BYTE(OPCODE_RREG) | address))
    {
        // Update internal register array data
        registerMap[address] = LOWER_BYTE(response);
    }
    else
    {
        // Restore previous register value
        registerMap[address] = oldValue;
    }
}

//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint8_t readSingleRegister(uint8_t address)
//!
//! \param address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint8_t readSingleRegister(uint8_t address)
{
    /* Assert(s) */
    assert(address < NUM_REGISTERS);        // Register address must be in range

    // Build TX  array
    dataTx[0] = UPPER_BYTE(OPCODE_RREG) | address;
    dataTx[1] = 0x00;

#ifdef SET_CRC_EN
    // Calculates the CRC IN word and adds it to the dataTx[] array
    updateCrcIn();
#endif

    // [FRAME 1] Send RREG command
    spiSendReceiveArrays(dataTx, dataRx, cmdByteLength);

    // [FRAME 2] Send NULL command to retrieve the register data
    uint16_t response = sendCommand(OPCODE_NULL);

    // Verify device response
    if (UPPER_BYTE(response) == OPCODE_RREG | ((uint16_t) address))
    {
        // Update internal register array data
        registerMap[address] = LOWER_BYTE(response);
    }

    return registerMap[address];
}

//*****************************************************************************
//
//! Internal function used by readData() to convert ADC data from multiple unsigned
//! bytes into a single signed 32-bit word.
//!
//! \fn int32_t signExtend(const uint8_t dataBytes[])
//!
//! \param dataBytes is a pointer to uint8_t[] where the first element is the MSB.
//!
//! \return Returns the signed-extend 32-bit result.
//
//*****************************************************************************
int32_t signExtend(const uint8_t dataBytes[])
{
#ifdef WORD_LENGTH_24BIT

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

    // NOTE: This right-shift operation on signed data maintains the signed bit,
    // and provides for the sign-extension from 24 to 32 bits.
    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);

#elif defined WORD_LENGTH_32BIT_SIGN_EXTEND

    int32_t signByte    = ((int32_t) dataBytes[0] << 24);
    int32_t upperByte   = ((int32_t) dataBytes[1] << 16);
    int32_t middleByte  = ((int32_t) dataBytes[2] << 8);
    int32_t lowerByte   = ((int32_t) dataBytes[3] << 0);

    return (signByte | upperByte | middleByte | lowerByte);

#elif defined WORD_LENGTH_32BIT_ZERO_PADDED

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

    // NOTE: This right-shift operation on signed data maintains the signed bit,
    // and provides for the sign-extension from 24 to 32 bits.
    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit

#elif defined WORD_LENGTH_16BIT_TRUNCATED

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);

    // NOTE: This right-shift operation on signed data maintains the signed bit,
    // and provides for the sign-extension from 16 to 32 bits.
    return (((int32_t) (upperByte | lowerByte)) >> 16);

#endif
}

//*****************************************************************************
//
//! Calculates the 16-bit CRC for the selected CRC polynomial.
//!
//! \fn uint16_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes length of data byte array to include in the CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use 0xFFFF when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and has not been optimized for speed.
//!
//! \return uint16_t calculated CRC word. When performing a partial CRC computation, provide this
//! value as the input to initialValue to resume computing the CRC for additional input bytes.
//
//*****************************************************************************
uint16_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
{
    /* Assert(s) */
    assert(dataBytes != 0x00);      // "dataBytes" must not be a NULL pointer
    assert(numberBytes != 0x00);    // "numberBytes" must be greater than zero

    int bitIndex, byteIndex;
    bool dataMSb;                   // Most significant bit of data byte
    bool crcMSb;                    // Most significant bit of crc byte

    // Initial value of crc register
    uint16_t crc = initialValue;

    // CRC16-CCITT polynomial
    // NOTE: The polynomial's MSB is generally assumed to be high (and is handled by
    // the "dataMSb ^ crcMSb" operation below) and so it is excluded from this value.
    const uint16_t poly = 0x1021;

    //
    // CRC algorithm...
    //

    // Loop through all bytes in the dataBytes[] array
    for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
    {
        // Point to MSb in byte
        bitIndex = 0x80u;

        // Loop through all bits in the current byte
        while (bitIndex > 0)
        {
            // Check MSB's of data and crc
            dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
            crcMSb  = (bool) (crc & 0x8000u);

            // Left-shift CRC register
            crc <<= 1;

            // Check if XOR operation of MSBs results in additional XOR operations
            if (dataMSb ^ crcMSb)
            {
                // XOR crc with polynomial
                crc ^= poly;
            }

            // Shift MSb pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}


//*****************************************************************************
//
//! Read ADC data
//!
//! \fn bool readData(adc_data_struct *dataStruct)
//!
//! \param dataStruct pointer to data structure where results from reading data will be placed
//!
//! NOTE: This is currently the only function in this example that verifies if the
//! CRC word on DOUT is correct.
//!
//! \return bool indicating if a CRC mismatch occurred (false = No error)
//
//*****************************************************************************
bool readData(adc_data_struct *dataStruct)
{
    // Build TX array
    dataTx[0] = 0x00;
    dataTx[1] = 0x00;

#ifdef SET_CRC_EN

    // NOTE: The following CRC words are hard-coded based on word length, fixed mode, and CRC mode...
    #if defined SET_FIXED && !defined SET_CRC_MODE // Only the command word is included in CRC calculation

        #ifdef WORD_LENGTH_24BIT
            dataTx[5*WORD_LENGTH] = 0xCC;
            dataTx[5*WORD_LENGTH + 1] = 0x9C;

        #elif WORD_LENGTH_16BIT
            dataTx[5*WORD_LENGTH] = 0x1D;
            dataTx[5*WORD_LENGTH + 1] = 0x0F;

        #elif WORD_LENGTH_32BIT
            dataTx[5*WORD_LENGTH] = 0x84;
            dataTx[5*WORD_LENGTH + 1] = 0xC0;

        #endif

    #else // FIXED = 0 OR (FIXED = 1 and CRC_MODE = 1) -> All device words are included in CRC calculation

        #ifdef WORD_LENGTH_24BIT
            dataTx[5*WORD_LENGTH] = 0x4E;
            dataTx[5*WORD_LENGTH + 1] = 0xC3;

        #elif WORD_LENGTH_16BIT
            dataTx[5*WORD_LENGTH] = 0xE1;
            dataTx[5*WORD_LENGTH + 1] = 0x39;

        #elif WORD_LENGTH_32BIT
            dataTx[5*WORD_LENGTH] = 0xF6;
            dataTx[5*WORD_LENGTH + 1] = 0xB8;

        #endif

    #endif

#endif

    /* Set the nCS pin LOW */
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);//Set CS low

    // Send NULL word, receive response word
    int i = 0;
    while (i < 6*WORD_LENGTH)
    {
        dataRx[i] = spiSendReceiveByte(dataTx[i]);
        ++i;
    }

    dataStruct->response = COMBINE_BYTES(dataRx[0], dataRx[1]);
    dataStruct->channel1 = signExtend(&dataRx[1*WORD_LENGTH]);
    dataStruct->channel2 = signExtend(&dataRx[2*WORD_LENGTH]);
    dataStruct->channel3 = signExtend(&dataRx[3*WORD_LENGTH]);
    dataStruct->channel4 = signExtend(&dataRx[4*WORD_LENGTH]);
    dataStruct->crc = COMBINE_BYTES(dataRx[5*WORD_LENGTH], dataRx[5*WORD_LENGTH + 1]);

#ifdef CRC_EN
    uint16_t crcWordOut = calculateCRC(&dataRx[0], 6*WORD_LENGTH, 0xFFFF);
#else
    // (OPTIONAL) Ignore CRC error checking (CRC = 0x00 -> indicates a valid SPI frame)
    uint16_t crcWordOut = 0x00;
#endif

    /* Set the nCS pin HIGH */
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);//Set CS low

    // Returns true when a CRC error occurs (any non-zero value for crcWordOut)
    return ((bool) crcWordOut);
}


//*****************************************************************************
//
//! Waits for the nDRDY interrupt or until the specified timeout occurs.
//!
//! \fn bool waitForDRDYinterrupt(const uint32_t timeout_ms)
//!
//! \param timeout_ms number of milliseconds to wait before timeout event.
//!
//! \return Returns 'true' if nDRDY interrupt occurred before the timeout.
//
//*****************************************************************************
bool waitForDRDY_Done(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     * Poll the nDRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the nDRDY pin is not active.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */

    // Convert ms to a # of loop iterations, OR use a timer...
    uint32_t timeout = timeout_ms * 8000;   // convert to # of loop iterations. 8000 cycles per ms

    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) && (timeout > 0));

    // Timeout counter greater than zero indicates that an interrupt occurred
    return (timeout > 0);
}


//*****************************************************************************
//
//! Example start up sequence for the ADS131A04.
//!
//! \fn void adcStartup(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI/GPIO pins of the MCU must have already been configured,
//! and (if applicable) the external clock source should be provided to CLKIN.
//!
//! NOTE: You may want to modify this function to configure the ADC's initial
//! register settings to your application's requirements.
//!
//! \return None.
//
//*****************************************************************************
void adcStartup(void)
{
    /* (OPTIONAL) Provide additional delay time for power supply settling */
    DelayMs1(50);
    /* (REQUIRED) Set nRESET pin high for ADC operation */
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);//Set P1.5 for slave reset
    /* (INTERNAL) Initialize internal 'registerMap' array with device default settings */
    restoreRegisterDefaults();
    /* (OPTION #1) Wait ~5ms for POR to complete */
    DelayMs1(5);
    /* (OPTION #2) Wait for nDRDY falling edge for indication that POR has completed */
    //  bool interruptedOccurred = waitForDRDYinterrupt(20);
    //  if (!interruptedOccurred)
    //  {
    //      // Error: Device appears to be unresponsive. Check the power supplies, clock,
    //      // and GPIO pins to make sure the device is active.
    //      assert(0);
    //  }

    /* (REQUIRED) Always send a NULL command first to establish SPI communication */
    //sendCommand(OPCODE_NULL);

    /* (OPTIONAL) Verify READY response */
    uint16_t response = sendCommand(OPCODE_NULL);//(REQUIRED) Always send a NULL command first to establish SPI communication
    if (0xFF04 != response)
    {
        // Error: Unexpected response
        //assert(0);
        Message("\n Error: Unexpected ADS131AX response");
    }
    /* (REQUIRED) Send UNLOCK to exit 'READY' state  */
    unlockRegisters();


    //
    // Configure initial register settings here...
    //

    // Default value for D_SYS_CFG_ADDRESS register
    uint8_t regVal = D_SYS_CFG_DEFAULT;

#ifdef SET_FIXED
    regVal |= D_SYS_CFG_FIXED_MASK;     // Set FIXED bit
#endif

#ifdef SET_CRC_EN
    regVal |= D_SYS_CFG_CRC_EN_MASK;    // Set CRC_EN bit
#endif

#ifdef SET_CRC_MODE
    regVal |= D_SYS_CFG_CRC_MODE_MASK;  // Set CRC_MODE bit
#endif

    // Write to D_SYS_CFG_ADDRESS
    writeSingleRegister(D_SYS_CFG_ADDRESS, regVal);

    // Configure data rate
    writeSingleRegister(CLK1_ADDRESS, CLK1_CLK_DIV_2);
    writeSingleRegister(CLK2_ADDRESS, CLK2_ICLK_DIV_2 | CLK2_OSR_2048);

    // Enable all ADC channels
    writeSingleRegister(ADC_ENA_ADDRESS, ADC_ENA_ENA_ALL_CH_PWUP);

    /* (REQUIRED) Always send a NULL command first to establish SPI communication */
    sendCommand(OPCODE_WAKEUP);

    // Ignore the first 3-4 conversion results to allow for the
    // output buffers to fill-up and the SINC3 filter to settle
    uint8_t ignore_counter = 4;
    adc_data_struct dummy_data;
    while (ignore_counter > 0)
    {
        waitForDRDY_Done(1000);
        readData(&dummy_data);
        ignore_counter--;
    }

}

void read_ads131a04(uint8_t *usbstr){
    unsigned char i;
    adc_data_struct ads_data;
    waitForDRDY_Done(1000);
    readData(&ads_data);
    hexdec_long(ads_data.channel1);
    for (i=0; i<10; i++)
        usbstr[i] = Rx_buf[i];
}
//******************************************************************************
//
//This is the USCI_B0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_B0_VECTOR)))
#endif
void USCI_B0_ISR (void)
{
    switch (__even_in_range(UCA0IV,4)){
        //Vector 2 - RXIFG
        case 2:
            //USCI_A0 TX buffer ready?
            while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT)) ;

            receiveData = USCI_B_SPI_receiveData(USCI_B0_BASE);

            //Increment data
            //transmitData++;

            //Send next value
            //USCI_B_SPI_transmitData(USCI_B0_BASE, transmitData);

            //Delay between transmissions for slave to process information
            __delay_cycles(40);



            break;
        default: break;
    }
}





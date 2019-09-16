// ----------------------------------------------------------------------
// Info
// ----------------------------------------------------------------------
/*
 *
 * Filename:	DRV8323_SPI.c
 *
 * Author: 		(c) 2019, Benjamin Prescher
 *
 * Target: 		XXX
 *
 * Notes:
 *
 */
// ----------------------------------------------------------------------
// History
// ----------------------------------------------------------------------
// 09.05.2019 - initial programming

// ----------------------------------------------------------------------
// Header Files
// ----------------------------------------------------------------------
#include "driverlib.h"
#include "DRV8323.h"

// ----------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------
#ifndef HIGH
#define HIGH                1
#endif
#ifndef LOW
#define LOW                 0
#endif

#ifndef READ
#define READ                1
#endif
#ifndef WRITE
#define WRITE               0
#endif

// ----------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------
// DRV830x SPI Input Data bit definitions:
struct  DRV830x_SPI_WRITE_WORD_BITS {       // bit      description
    uint16_t DATA:11;                       // 10:0     FIFO reset
    uint16_t ADDRESS:4;                     // 14:11    Enhancement enable
    uint16_t R_W:1;                         // 15       R/W
};

union DRV830x_SPI_WRITE_WORD_REG {
    uint16_t                           all;
    struct DRV830x_SPI_WRITE_WORD_BITS bit;
};

// ----------------------------------------------------------------------
// Functions
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// SPI driver to interface with TLE5012
// ----------------------------------------------------------------------
uint16_t SPI_Driver(DRV8323_VARS_t *v, uint16_t data)
{
    uint8_t highByte = (uint8_t)((data >> 8) & 0x00FF);
    uint8_t lowByte  = (uint8_t)(data & 0x00FF);
    uint16_t returnValue;

    //USCI_A0 TX buffer ready?
    while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    //Transmit Data to slave
    USCI_B_SPI_transmitData(USCI_B0_BASE, highByte);

    //USCI_A0 RX buffer ready?
    while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT));
    returnValue = (USCI_B_SPI_receiveData(USCI_B0_BASE) << 8) &0xFF00;

    //USCI_A0 TX buffer ready?
    while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    //Transmit Data to slave
    USCI_B_SPI_transmitData(USCI_B0_BASE, lowByte);

    //USCI_A0 RX buffer ready?
    while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT));
    returnValue |= USCI_B_SPI_receiveData(USCI_B0_BASE) &0x00FF;

    return returnValue;
}

// ----------------------------------------------------------------------
// Read from a DRV830x Register
// ----------------------------------------------------------------------
void DRV8323_SPI_Read(DRV8323_VARS_t *v, uint16_t address)
{
    union DRV830x_SPI_WRITE_WORD_REG w;
    uint16_t * cntrlReg;

    cntrlReg = (uint16_t*)&(v->Fault_Status_1);
    w.bit.R_W = READ;
    w.bit.ADDRESS = address;
    w.bit.DATA = 0;

    // Enable CS; SPI transfer; Disable CS
    GPIO_setOutputLowOnPin(v->ScsPort, v->ScsPin);
    cntrlReg[address] = SPI_Driver(v, w.all);
    GPIO_setOutputHighOnPin(v->ScsPort, v->ScsPin);
}

// ----------------------------------------------------------------------
// Write to a DRV830x Register
// ----------------------------------------------------------------------
void DRV8323_SPI_Write(DRV8323_VARS_t *v, uint16_t address)
{
    union DRV830x_SPI_WRITE_WORD_REG w;
    uint16_t * cntrlReg;

    cntrlReg = (uint16_t*)&(v->Fault_Status_1);
    w.bit.R_W = WRITE;
    w.bit.ADDRESS = address;
    w.bit.DATA = cntrlReg[address];

    // Enable CS; SPI transfer; Disable CS
    GPIO_setOutputLowOnPin(v->ScsPort, v->ScsPin);
    SPI_Driver(v, w.all);
    GPIO_setOutputHighOnPin(v->ScsPort, v->ScsPin);
}

// ----------------------------------------------------------------------
// something...
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// End of file
// ----------------------------------------------------------------------


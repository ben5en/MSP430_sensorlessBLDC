// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	main.c
//
//	MIT License
//
// 	Copyright (c) 2019 Benjamin Prescher
//
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//
//	target:		Texas Instruments MSP430
//
// ######################################################################
//  Info:
//  This software provides a simple example of a cascaded speed control as a sensorless
//  block commutation on an MSP430F5529 Launchpad in combination with a DRV8323 BoosterPack.
//
//  Other features include debugging via the serial interface (UartMonitor) and a decoupling
//  of the DC link voltage.
//
//  The underlying current control as well as the sensorless calculations are generated with
//  a frequency of max. 16kHz running. The DRV8323RS can be controlled with 6, 3 or 1 PWM signals.
//  According to the pin layout of the Launchpad it becomes clear that for the 6 or 3 PWM mode
//  several different timer peripherals would be necessary. Unfortunately, there is no way to
//  start different timers synchronized. Thus, in this case, the 1x PWM method was used.
//
//  The ADC module offers the possibility to be started in hardware from one out of three timer
//  modules. Sad to say, the Launchpad's routing in combination with the DRV BoosterPack does
//  not allow you to use one of the three triggering timer modules. For this purpose, a timer
//  interrupt is triggered at the reversal point of the PWM carrier signal and the ADC conversion
//  is started in software!
//
//  Since the start of the ADC conversion is triggered in software, all calculations must be done
//  so that the timer ISR can be triggered and called undisturbed:
//  The ADC values are provided via DMA. From the sampling start to calling the DMA ISR for motor
//  control, nearly 22us pass. The calculations needed for motor control take almost 38us at a
//  system frequency of 25MHz. Thus, about 60us pass, which represents the maximum value (62.5us at
//  16kHz PWM / ISR frequency) for the calculation time, so that the ISR / starting point of the ADC
//  conversion is not called late / not aligned.
//
//  The 1x PWM method is practicable. In a separate layout I would recommend to use one out of the
//  three timer peripherals which allow you to start ADC sampling via hardware.
//  In this way, the PWM frequency can be further increased because sufficient computation time
//  is available, or the system frequency can be lowered.
//
//  Nonetheless, the MSP430 performs very well and can be used for simple motor control tasks
//  with a suitable routing.
//
//  Source: some of the routines are based on routines of the "math blocks" found in the
//  controlSuite for C2000 controllers.
// ######################################################################
//
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	16.08.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------
#include "driverlib.h"
#include "QmathLib.h"
#include "F5529_BLDC_Settings.h"
#include "delay.h"
#include "config.h"
#include "status.h"
#include "bldc.h"
#include "fifo.h"
#include "Serial_Cmd_Monitor.h"
#include "DRV8323.h"

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
FIFO_t Fifo;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
void debugMonitor(void);
void drv8323RS_init(void);

// ----------------------------------------------------------------------
//
void main (void)
{
    // ----------------------------------------------------------------------
    //  Initialize the MSP430F5529LP
    MSP430F5529LP_init();

    // ----------------------------------------------------------------------
    //  UART Monitor is used for debugging
    //  More information can be found here: https://github.com/ben5en/MSP430_UartMonitor
    ClearBufferRelatedParam();

    //  UART RX is ISR - handled and TX is handled in main loop.
    //  The MSP430F5529 does not have a hardware FIFO, thus a SW FIFO is used
    //  to store RX'ed values and handle the debug-routine in main loop.
    FIFO_clear(&Fifo);

    // ----------------------------------------------------------------------
    //  Initialize the DRV8323
    drv8323RS_init();

    // ----------------------------------------------------------------------
    //  Initialize the motor control parameter with the help of DMA peripheral:
    BLDC_initWithDMA();

    // ----------------------------------------------------------------------
    //  Start the timer; make sure you specify the correct counting mode:
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UPDOWN_MODE);

    // ----------------------------------------------------------------------
    // TODO: enable nFault ISR + IE config for pin + ISR routine for fault state

    // ----------------------------------------------------------------------
    //
    __bis_SR_register(GIE);     // Enable interrupts globally

    // ----------------------------------------------------------------------
    //
    while(1)
    {
        BLDC_runSpeedCntl();
        BLDC_runDecoupling();

        debugMonitor();
    }
}

// ----------------------------------------------------------------------
//  Start the ADC conversion aligned to the center of the PWM period:
#pragma vector=TIMER2_A0_VECTOR
__interrupt void CCR0_ISR (void)
{
    ADC12_A_startConversion(ADC12_A_BASE, ADC12_A_MEMORY_0, ADC12_A_SEQOFCHANNELS);
}

// ----------------------------------------------------------------------
//  UartMonitor RX, write into software FIFO:
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR (void)
{
    switch (__even_in_range(UCA0IV,4)){
    case 0: break;   //Vector 0:   No interrupt
    case 2: FIFO_write(&Fifo, USCI_A_UART_receiveData(USCI_A0_BASE)); break;   //Vector 2:   RX ISR
    case 4: break;   //Vector 4:   TX ISR
    default: break;
    }
}

// ----------------------------------------------------------------------
//  ADC values are transferd to there corresponding variable, run BLDC routine:
#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR (void)
{
    switch (__even_in_range(DMAIV,16)){
    case 0: break;
    case 2: BLDC_runCmtnCntl(); break;  //DMA0IFG = DMA Channel 0
    case 4: break;  //DMA1IFG = DMA Channel 1
    case 6: break;  //DMA2IFG = DMA Channel 2
    case 8: break;  //DMA3IFG = DMA Channel 3
    case 10: break; //DMA4IFG = DMA Channel 4
    case 12: break; //DMA5IFG = DMA Channel 5
    case 14: break; //DMA6IFG = DMA Channel 6
    case 16: break; //DMA7IFG = DMA Channel 7
    default: break;
    }
}

// ----------------------------------------------------------------------
//
void debugMonitor(void)
{
    // if something is in the FIFO, handle it:
    if(Fifo.fifoLvl > 0)
    {
        // SerialCmdMonitor routine:
        receivedDataCommand(FIFO_read(&Fifo));
    }

    // reset FIFO if an overrun is detected:
    if(true == Fifo.overrun)
    {
        //Disable receive interrupt
        USCI_A_UART_clearInterrupt (USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
        USCI_A_UART_disableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);

        ClearBufferRelatedParam();
        FIFO_clear(&Fifo);

        //Enable receive interrupt
        USCI_A_UART_clearInterrupt (USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
        USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
    }
}

// ----------------------------------------------------------------------
//
void drv8323RS_init(void)
{
    // Connect chip select port and pin
    gDrv8323.ScsPort = (uint16_t)GPIO_PORT_P2;
    gDrv8323.ScsPin  = (uint16_t)GPIO_PIN3;

    // enable the DRV8323RS driver
    GPIO_setOutputHighOnPin(GPIO_DRV_ENABLE);
    delay_us(1e6);

    // hardware offset calibration:
    GPIO_setOutputHighOnPin(GPIO_DRV_CAL);
    delay_us(120);
    GPIO_setOutputLowOnPin (GPIO_DRV_CAL);
    delay_us(120);

    // read and store the default register values:
    uint16_t i;
    for(i= 0; i < DRV8323_CSA_CNTRL_ADDR + 1; i++)
    {
        DRV8323_SPI_Read(&gDrv8323, i);
        delay_us(5);
    }

    // update to PWM_mode_1:
    gDrv8323.Driver_Control.bit.PWM_MODE = drv_PWM_mode_1;
    DRV8323_SPI_Write(&gDrv8323, DRV8323_DRIVER_CNTRL_ADDR);

    // update to CSA_GAIN correspond to the settings:
#ifdef DRV8323_GAIN_5
    gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_5;
#elif DRV8323_GAIN_10
    gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_10;
#elif DRV8323_GAIN_20
    gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_20;
#elif DRV8323_GAIN_40
    gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_40;
#endif
    DRV8323_SPI_Write(&gDrv8323, DRV8323_CSA_CNTRL_ADDR);
}
// ----------------------------------------------------------------------
// 	end of file
// ----------------------------------------------------------------------


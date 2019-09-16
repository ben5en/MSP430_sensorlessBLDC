// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	config.c
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
//	target:		Texas Instruments MSP430F5529LP with BOOSTXL-DRV8323RS EVM
//
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	17.08.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------
#include "driverlib.h"
#include "F5529_BLDC_Settings.h"
#include "config.h"

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------
#define SPICLK                      500000

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
uint32_t myACLK  = 0;
uint32_t mySMCLK = 0;
uint32_t myMCLK  = 0;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
//
void GPIO_init(void)
{
    // ----------------------------------------------------------------------
    //  Configure LaunchPad Ports and Pins.
    //  The GPIO registers should be set in a specific order:
    //  PxOUT -> PxSEL or PxSELx -> PxDIR -> PxREN -> PxIES -> PxIFG -> PxIE
    //
    //  unused port pins should be configured as I/O function, output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_PA, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_PB, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_PC, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_PD, GPIO_PIN_ALL16);

    GPIO_setAsOutputPin(GPIO_PORT_PA, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_PB, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_PC, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_PD, GPIO_PIN_ALL16);

    // ----------------------------------------------------------------------
    //  Output the ACLK and MCLK signals to their respective pins - which allows you to
    //  watch them with a logic analyzer
    //    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_ACLK);       // GPIO_PORT_P1, GPIO_PIN0
    //    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_SMCLK);      // GPIO_PORT_P2, GPIO_PIN2
    //    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_MCLK);       // GPIO_PORT_P7, GPIO_PIN7

    // ----------------------------------------------------------------------
    //  configure all hall pins as inputs with pull ups enabled:
    GPIO_setAsInputPinWithPullUpResistor(GPIO_HALL_A);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_HALL_B);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_HALL_C);

    // ----------------------------------------------------------------------
    //  configure all logic pins as outputs for DRV8323 for 1x PWM Mode:
    GPIO_setOutputLowOnPin(GPIO_DRV_INLA);
    GPIO_setAsOutputPin   (GPIO_DRV_INLA);

    GPIO_setOutputLowOnPin(GPIO_DRV_INHB);
    GPIO_setAsOutputPin   (GPIO_DRV_INHB);

    GPIO_setOutputLowOnPin(GPIO_DRV_INLB);
    GPIO_setAsOutputPin   (GPIO_DRV_INLB);

    //  The INHC input controls the direction through the 6-step commutation table
    //  which is used to change the direction of the motor when Hall effect sensors
    //  are directly controlling the state of the INLA, INHB, and INLB inputs. Tie
    //  the INHC pin low if this feature is not required.
    GPIO_setOutputLowOnPin(GPIO_DRV_INHC);
    GPIO_setAsOutputPin   (GPIO_DRV_INHC);

    //  The INLC input brakes the motor by turning off all high-side MOSFETs and
    //  turning on all low-side MOSFETs when the INLC pin is pulled low. This brake
    //  is independent of the state of the other input pins. Tie the INLC pin
    //  high if this feature is not required.
    GPIO_setOutputHighOnPin(GPIO_DRV_INLC);
    GPIO_setAsOutputPin    (GPIO_DRV_INLC);

    // ----------------------------------------------------------------------
    //  configure all pins used as IO-Funktion on BOOSTXL-DRV8323RS EVM
    //  LED, ENABLE, CAL, INLA, INHB, INLB, INHC and INLC should be low and output
    //  direction, this is already done above...
    GPIO_setAsInputPin(GPIO_DRV_nFAULT);

    GPIO_setOutputLowOnPin(GPIO_DRV_CAL);
    GPIO_setAsOutputPin   (GPIO_DRV_CAL);

    GPIO_setOutputLowOnPin(GPIO_DRV_LED);
    GPIO_setAsOutputPin   (GPIO_DRV_LED);

    // ----------------------------------------------------------------------
    //  Configure LaunchPad LEDs
    GPIO_setOutputLowOnPin(GPIO_LED_RED);
    GPIO_setAsOutputPin   (GPIO_LED_RED);
    GPIO_setOutputLowOnPin(GPIO_LED_GREEN);
    GPIO_setAsOutputPin   (GPIO_LED_GREEN);

    // ----------------------------------------------------------------------
    //  Configure LaunchPad Buttons
    GPIO_setAsInputPinWithPullUpResistor(GPIO_BUTTON_S1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_BUTTON_S2);
}

// ----------------------------------------------------------------------
//
void UCS_init(void)
{
    bool returnValue = STATUS_FAIL;

    // ----------------------------------------------------------------------
    //  Configure crystal pins:
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_XOUT_XIN);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_XT2OUT_XT2IN);

    // ----------------------------------------------------------------------
    // Configure core voltage level <<page 4-37 MSP430 D.WS. Rev. 4.01>>
    //    PMM_setVCore( PMM_CORE_LEVEL_0 );  // Set core voltage level to handle <4MHz clock rate
    //    PMM_setVCore( PMM_CORE_LEVEL_1 );  // Set core voltage level to handle 8MHz - 12MHz clock rate
    //    PMM_setVCore( PMM_CORE_LEVEL_2 );  // Set core voltage level to handle 12MHz - 20MHz clock rate
    PMM_setVCore( PMM_CORE_LEVEL_3 );  // Set core voltage level to handle >20MHz clock rate

    // ----------------------------------------------------------------------
    // Configure Oscillators
    // Set the XT1/XT2 crystal frequencies used on the LaunchPad, and connected
    // to the clock pins, so that driverlib knows how fast they are (these are
    // needed for the DriverLib clock 'get' and crystal start functions)
    UCS_setExternalClockSource(
            XT1_LF_CRYSTAL_FREQUENCY_IN_HZ,  // XT1CLK input
            XT2_HF_CRYSTAL_FREQUENCY_IN_HZ   // XT2CLK input
    );

    // Verify if the default clock settings are as expected
    myACLK  = UCS_getACLK();
    mySMCLK = UCS_getSMCLK();
    myMCLK  = UCS_getMCLK();

    // Initialize the XT1 crystal oscillator (using a timeout in case there is a problem with the crystal)
    // - This requires P5.4 and P5.5 pins to be connected (and configured) as clock input pins.
    // - Another alternative is to use the non-timeout function which "hangs" if XT1 isn't configured;
    //    UCS_turnOnXT1( CS_XT1_DRIVE_0, UCS_XCAP_3 );   (in fact, we used the non-timeout function to setup XT2)
    // - The "WithTimeout" function used here will always exit, even if XT1 fails to initialize.
    //   You must check to make sure XT1 was initialized properly... in a real application, you would
    //   usually replace the while(1) with a more useful error handling function.
    returnValue = UCS_turnOnLFXT1WithTimeout(UCS_XT1_DRIVE_0, UCS_XCAP_3, XT1_TIMEOUT);

    if (returnValue == STATUS_FAIL)
    {
        while( 1 );
    }

    // Initializes the XT2 crystal oscillator with no timeout.
    // In case of failure, code hangs here.
    // For time-out instead of code hang use UCS_turnOnXT2WithTimeout().
    //UCS_turnOnXT2( UCS_XT2_DRIVE_4MHZ_8MHZ );

    //    This is an example of turning on XT2 with the the timeout option.
    returnValue = UCS_turnOnXT2WithTimeout(UCS_XT2_DRIVE_4MHZ_8MHZ, XT2_TIMEOUT);

    if (returnValue == STATUS_FAIL)
    {
        while( 1 );
    }

    // ----------------------------------------------------------------------
    // Configure Clocks
    // Set ACLK to use XT1 as its oscillator source (32KHz)
    UCS_initClockSignal(
            UCS_ACLK,                                    // Clock you're configuring
            UCS_XT1CLK_SELECT,                           // Clock source
            UCS_CLOCK_DIVIDER_1                          // Divide down clock source by this much
    );

    // Set REFO as the oscillator reference clock for the FLL
    UCS_initClockSignal(
            UCS_FLLREF,                                  // Clock you're configuring
            UCS_XT1CLK_SELECT,                           // Clock source
            UCS_CLOCK_DIVIDER_1                          // Divide down clock source by this much
    );

    // Set MCLK and SMCLK to use the DCO/FLL as their oscillator source (8MHz)
    // The function does a number of things: Calculates required FLL settings; Configures FLL and DCO,
    // and then sets MCLK and SMCLK to use the DCO (with FLL runtime calibration)
    // FLLSettle function adds time needed for the FLL feedback loop to "settle".
    // Alternatively, you could use the UCS_initFLL() function if you didn’t want
    // the function to add the clock settling time.
    UCS_initFLLSettle(
            MCLK_DESIRED_FREQUENCY_IN_KHZ,               // MCLK frequency
            MCLK_FLLREF_RATIO                            // Ratio between MCLK and FLL's reference clock source
    );

    // Verify that the modified clock settings are as expected
    myACLK  = UCS_getACLK();
    mySMCLK = UCS_getSMCLK();
    myMCLK  = UCS_getMCLK();

    // Select DCO as SMCLK source
    // The UCS initFLL functions configure both MCLK and SMCLK. A common mistake is to
    // configure SMCLK before calling the FLL init function.
    UCS_initClockSignal(
            UCS_SMCLK,
            UCS_DCOCLK_SELECT,
            UCS_CLOCK_DIVIDER_1
    );

    // Verify that the modified clock settings are as expected
    myACLK  = UCS_getACLK();
    mySMCLK = UCS_getSMCLK();
    myMCLK  = UCS_getMCLK();
}

// ----------------------------------------------------------------------
//
void ADC_init(void)
{
    // ----------------------------------------------------------------------
    // Enable A/D channel inputs
    // DRV vsenVM: P6.5
    // DRV POT   : P6.6
    // DRV VSENA : P6.0
    // DRV VSENB : P6.1
    // DRV VSENC : P6.2
    // DRV ISENC : P6.3
    // DRV ISENB : P6.4
    // DRV ISENA : P7.0
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, 0x7F); // GPIO_PIN0 - GPIO_PIN6
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P7, GPIO_PIN0);

    //Initialize the ADC12_A Module
    /*
     * Base address of ADC12_A Module
     * Use internal ADC12_A bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     * ADC12_A_SAMPLEHOLDSOURCE_1 = TA0CCR1
     * ADC12_A_SAMPLEHOLDSOURCE_2 = TBCCR0
     * ADC12_A_SAMPLEHOLDSOURCE_3 = TBCCR1
     *
     */
    ADC12_A_init(ADC12_A_BASE,
                 ADC12_A_SAMPLEHOLDSOURCE_SC,
                 ADC12_A_CLOCKSOURCE_MCLK,
                 ADC12_A_CLOCKDIVIDER_2);

    ADC12_A_enable(ADC12_A_BASE);

    /*
     * Base address of ADC12_A Module
     * For memory buffers 0-7 sample/hold for 16 clock cycles
     * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
     * Enable Multiple Sampling
     *
     * With MCLK @ 25MHz and clock divider = 2 you need minimum 16 clock
     * cycles sample time to get stable ADC results.
     * This will take ~22us from the start sampling until DMA ISR is called
     * (all ADC values written to there corresponding memory)
     *
     * Take care of this "sampling time" if you speed up the PWM / ISR
     * frequency to not overlap the next start of sequence sampling.
     * Usually the ADC12TOV condition is generated in this case.
     */
    ADC12_A_setupSamplingTimer(ADC12_A_BASE,
                               ADC12_A_CYCLEHOLD_16_CYCLES,
                               ADC12_A_CYCLEHOLD_4_CYCLES,
                               ADC12_A_MULTIPLESAMPLESENABLE);

    //Configure Memory Buffers
    ADC12_A_configureMemoryParam vsenA = {0};
    vsenA.memoryBufferControlIndex = DRV_ADC_VSEN_A;
    vsenA.inputSourceSelect = DRV_VSEN_A_CH;
    vsenA.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    vsenA.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    vsenA.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&vsenA);

    ADC12_A_configureMemoryParam vsenB = {0};
    vsenB.memoryBufferControlIndex = DRV_ADC_VSEN_B;
    vsenB.inputSourceSelect = DRV_VSEN_B_CH;
    vsenB.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    vsenB.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    vsenB.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&vsenB);

    ADC12_A_configureMemoryParam vsenC = {0};
    vsenC.memoryBufferControlIndex = DRV_ADC_VSEN_C;
    vsenC.inputSourceSelect = DRV_VSEN_C_CH;
    vsenC.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    vsenC.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    vsenC.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&vsenC);

    ADC12_A_configureMemoryParam isenA = {0};
    isenA.memoryBufferControlIndex = DRV_ADC_ISEN_A;
    isenA.inputSourceSelect = DRV_ISEN_A_CH;
    isenA.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    isenA.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    isenA.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&isenA);

    ADC12_A_configureMemoryParam isenB = {0};
    isenB.memoryBufferControlIndex = DRV_ADC_ISEN_B;
    isenB.inputSourceSelect = DRV_ISEN_B_CH;
    isenB.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    isenB.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    isenB.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&isenB);

    ADC12_A_configureMemoryParam isenC = {0};
    isenC.memoryBufferControlIndex = DRV_ADC_ISEN_C;
    isenC.inputSourceSelect = DRV_ISEN_C_CH;
    isenC.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    isenC.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    isenC.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&isenC);

    ADC12_A_configureMemoryParam vsenVM = {0};
    vsenVM.memoryBufferControlIndex = DRV_ADC_VSEN_VM;
    vsenVM.inputSourceSelect = DRV_VSEN_VM_CH;
    vsenVM.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    vsenVM.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    vsenVM.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&vsenVM);

    ADC12_A_configureMemoryParam poti = {0};
    poti.memoryBufferControlIndex = DRV_ADC_POTI;
    poti.inputSourceSelect = DRV_POTENTIOMETER_CH;
    poti.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    poti.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    poti.endOfSequence = ADC12_A_ENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&poti);
}

// ----------------------------------------------------------------------
//
void TIMER_init(void)
{
    // ----------------------------------------------------------------------
    //  1. Configure output pin
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_DRV_INHA);

    // ----------------------------------------------------------------------
    //  2. Setup Timer (TAR, TACTL)
    //  TimerA2 in up down mode using SMCLK and enable ISR @ CCR0
    //  The period is twice the value in TAxCCR0 in up down mode.
    Timer_A_initUpDownModeParam initUpDwnParam = { 0 };
    initUpDwnParam.clockSource               = TIMER_A_CLOCKSOURCE_SMCLK;       // 25MHz
    initUpDwnParam.clockSourceDivider        = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initUpDwnParam.timerPeriod               = (uint16_t)((float)UCS_getSMCLK()/(2.0*DEVICE_PWM_FREQUENCY_kHz*1e3));
    initUpDwnParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;  // Disable TAR interrupt
    initUpDwnParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;   // Enable prd interrupt
    initUpDwnParam.timerClear                = TIMER_A_DO_CLEAR;               // Clear TAR & clock divider
    initUpDwnParam.startTimer                = false;                          // Don't start the timer, yet
    Timer_A_initUpDownMode(TIMER_A2_BASE, &initUpDwnParam);

    // ----------------------------------------------------------------------
    //  3. Setup Capture & Compare features
    Timer_A_initCompareModeParam initCcr2Param = {0};
    initCcr2Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    initCcr2Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initCcr2Param.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
    initCcr2Param.compareValue = 0; // start with pwm off (output low)
    Timer_A_initCompareMode(TIMER_A2_BASE, &initCcr2Param);

    // ----------------------------------------------------------------------
    //  4. Clear/enable interrupt flags and start timer
    Timer_A_clearTimerInterrupt(TIMER_A2_BASE);                               // Clear TA0IFG
}

// ----------------------------------------------------------------------
//
void SPI_init(void)
{
    bool returnValue;

    //P3.0,1,2 option select
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 |GPIO_PIN2);

    //Initialize Master
    USCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = UCS_getSMCLK();
    param.desiredSpiClock = SPICLK;
    param.msbFirst = USCI_B_SPI_MSB_FIRST;
    param.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    returnValue =  USCI_B_SPI_initMaster(USCI_B0_BASE, &param);

    if (STATUS_FAIL == returnValue)
    {
        while(1);
    }

    //Enable SPI module
    USCI_B_SPI_enable(USCI_B0_BASE);
}

// ----------------------------------------------------------------------
//
void UART_init(void)
{
    bool returnValue = STATUS_FAIL;

    //P4.4,5 = USCI_A1 TXD/RXD
    //    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4 | GPIO_PIN5);

    //P3.3,4 = USCI_A0 TXD/RXD
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN3 | GPIO_PIN4);

    // setting UART to 115200 Baud @ 25MHz SysClk with following helper link:
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html

    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 13;
    param.firstModReg = 9;
    param.secondModReg = 0;
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    returnValue = USCI_A_UART_init(USCI_A0_BASE, &param);

    if (STATUS_FAIL == returnValue)
    {
        while(1);
    }

    //Enable UART module for operation
    USCI_A_UART_enable(USCI_A0_BASE);

    //Enable Receive Interrupt
    USCI_A_UART_clearInterrupt (USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
}

// ----------------------------------------------------------------------
//
void MSP430F5529LP_init(void)
{
    WDT_A_hold(WDT_A_BASE);
    GPIO_init();
    UCS_init();
    ADC_init();
    TIMER_init();
    SPI_init();
    UART_init();
}
// ----------------------------------------------------------------------
// 	something...
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	end of file
// ----------------------------------------------------------------------


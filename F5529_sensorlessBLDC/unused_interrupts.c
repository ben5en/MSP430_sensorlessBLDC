#include <driverlib.h>

// *****************************************************************************
// MSP430F5529 Unused Vectors
// *****************************************************************************
// UNUSED_HWI_ISR()
//
// The default linker command file created by CCS links all interrupt vectors to
// their specified address location. This gives you a warning for vectors that
// are not associated with an ISR function. The following function (and pragma's)
// handles all interrupt vectors.
//
// Just make sure you comment out the vector pragmas handled by your own code.
// For example, you will receive a "program will not fit into" error if you do
// not comment out the WDT vector below. This occurs since the linker tries to
// fit both of the vector addresses into the same memory locations ... and they
// won't fit.
// *****************************************************************************
#pragma vector=ADC12_VECTOR
#pragma vector=COMP_B_VECTOR
//#pragma vector=DMA_VECTOR
#pragma vector=PORT1_VECTOR
#pragma vector=PORT2_VECTOR
#pragma vector=RTC_VECTOR
#pragma vector=SYSNMI_VECTOR
#pragma vector=TIMER0_A0_VECTOR
#pragma vector=TIMER0_A1_VECTOR
#pragma vector=TIMER0_B0_VECTOR
#pragma vector=TIMER0_B1_VECTOR
#pragma vector=TIMER1_A0_VECTOR
#pragma vector=TIMER1_A1_VECTOR
//#pragma vector=TIMER2_A0_VECTOR
#pragma vector=TIMER2_A1_VECTOR
#pragma vector=UNMI_VECTOR
#pragma vector=USB_UBM_VECTOR
//#pragma vector=USCI_A0_VECTOR
#pragma vector=USCI_A1_VECTOR
#pragma vector=USCI_B0_VECTOR
#pragma vector=USCI_B1_VECTOR
#pragma vector=WDT_VECTOR
__interrupt void UNUSED_HWI_ISR (void)
{
    __no_operation();
}


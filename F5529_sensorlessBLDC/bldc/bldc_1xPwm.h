#ifndef BLDC_BLDC_1XPWM_H_
#define BLDC_BLDC_1XPWM_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	bldc_1xPwm.h
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
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	03.09.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------
#include "driverlib.h"
#include "F5529_BLDC_Settings.h"
#include "bldc_types.h"

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
typedef struct
{
    uint16_t TimerBaseAddress;      // Parameter: Base Address for driverlib funtion
    uint16_t TimerCompareRegister;  // Parameter: Timer Compare Register for driverlib function
    uint16_t DutyMax;               // Parameter: maximum Duty value
}BLDC_1xPWM_t;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
// function to set default values for the object
inline void BLDC_PWM_objectInit(volatile BLDC_1xPWM_t *bldc_pwm_obj,
                                uint16_t baseAddress,
                                uint16_t compareRegister)
{
    BLDC_1xPWM_t *obj = (BLDC_1xPWM_t*)bldc_pwm_obj;

    obj->TimerBaseAddress = baseAddress;
    obj->TimerCompareRegister = compareRegister;
    obj->DutyMax = 0;
}

// ----------------------------------------------------------------------
//  handles DRV8323 1x PWM mode. PWM output on TimerAx pin.
//  Sets 3 selected pins high or low corresponding to the commutation table.
inline void BLDC_1xPWM_run(volatile BLDC_1xPWM_t *bldc_pwm_obj,
                           volatile CMTN_STATE_T cmtnState,
                           uint16_t duty)
{
    BLDC_1xPWM_t *obj = (BLDC_1xPWM_t*)bldc_pwm_obj;

    // update PWM on GPIO_DRV_INHA:
    Timer_A_setCompareValue(obj->TimerBaseAddress, obj->TimerCompareRegister, duty);

    // update logic level for DRV8323 commutation table:
    switch(cmtnState)
    {
    case Cmtn_A_B: // State s1: current flows to motor windings from phase A->B, de-energized phase = C
        GPIO_setOutputHighOnPin(GPIO_DRV_INLA);
        GPIO_setOutputLowOnPin (GPIO_DRV_INHB);
        GPIO_setOutputHighOnPin(GPIO_DRV_INLB);
        break;
    case Cmtn_A_C: // State s2: current flows to motor windings from phase A->C, de-energized phase = B
        GPIO_setOutputHighOnPin(GPIO_DRV_INLA);
        GPIO_setOutputLowOnPin (GPIO_DRV_INHB);
        GPIO_setOutputLowOnPin (GPIO_DRV_INLB);
        break;
    case Cmtn_B_C: // State s3: current flows to motor windings from phase B->C, de-energized phase = A
        GPIO_setOutputHighOnPin(GPIO_DRV_INLA);
        GPIO_setOutputHighOnPin(GPIO_DRV_INHB);
        GPIO_setOutputLowOnPin (GPIO_DRV_INLB);
        break;
    case Cmtn_B_A: // State s4: current flows to motor windings from phase B->A, de-energized phase = C
        GPIO_setOutputLowOnPin (GPIO_DRV_INLA);
        GPIO_setOutputHighOnPin(GPIO_DRV_INHB);
        GPIO_setOutputLowOnPin (GPIO_DRV_INLB);
        break;
    case Cmtn_C_A: // State s5: current flows to motor windings from phase C->A, de-energized phase = B
        GPIO_setOutputLowOnPin (GPIO_DRV_INLA);
        GPIO_setOutputHighOnPin(GPIO_DRV_INHB);
        GPIO_setOutputHighOnPin(GPIO_DRV_INLB);
        break;
    case Cmtn_C_B: // State s6: current flows to motor windings from phase C->B, de-energized phase = A
        GPIO_setOutputLowOnPin (GPIO_DRV_INLA);
        GPIO_setOutputLowOnPin (GPIO_DRV_INHB);
        GPIO_setOutputHighOnPin(GPIO_DRV_INLB);
        break;
    case Cmtn_Align: // State s7: current flows to motor windings from phase A->(B+C)
        GPIO_setOutputHighOnPin(GPIO_DRV_INLA);
        GPIO_setOutputHighOnPin(GPIO_DRV_INHB);
        GPIO_setOutputHighOnPin(GPIO_DRV_INLB);
        break;
    default: break;
    }
}
// ----------------------------------------------------------------------
// 	something...
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	end of file
// ----------------------------------------------------------------------


#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /* BLDC_BLDC_1XPWM_H_ */

#ifndef BLDC_CMTN_H_
#define BLDC_CMTN_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	cmtn.h
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
//	target:		Texas Instruments C2000
//
//  This file is mainly build up on the Texas Instruments "maths-blocks" found
//  in the C2000 control suite and adopted to work with Q-Formats and MPS430 types.
//
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	03.09.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------
#include "bldc_types.h"

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
typedef volatile struct
{

    bool     ZcTrig;            // Variable: Zero-Crossing trig flag
    bool     Trigger;           // Output: commutation trigger flag
    bool     Delay30DoneFlag;   // Variable: 30 Deg delay flag
    uint16_t NoiseWindowCounter;// Variable: Noise windows counter
    uint16_t NewTimeStamp;      // Variable: Time stamp
    uint16_t OldTimeStamp;      // History: Previous time stamp
    uint16_t VirtualTimer;      // Input: Virtual timer
    uint16_t RevPeriod;         // Variable: revolution time counter
    uint16_t CmtnDelay;         // Variable: Time delay in terms of number of sampling time periods
    uint8_t  DelayTaskPointer;  // Variable: Delay task pointer
    uint16_t NoiseWindowMax;    // Variable: Maximum noise windows counter
    uint16_t CmtnDelayCounter;  // Variable: Time delay counter
    uint16_t NWDelta;           // Variable: Noise windows delta
    uint16_t NWDelayThres;      // Variable: Noise windows dynamic threshold
    uint16_t GPR1_COM_TRIG;     // Variable: Division reminder
    int16_t  Tmp;               // Variable: Temp. variable
} CMTN_t;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
// function to set default values for the object
inline void CMTN_objectInit(volatile CMTN_t *cmtn_obj)
{
    CMTN_t *obj = (CMTN_t*)cmtn_obj;

    obj->RevPeriod = 0;
    obj->ZcTrig = false;
    obj->Trigger = false;
    obj->NoiseWindowCounter = 0;
    obj->Delay30DoneFlag = false;
    obj->NewTimeStamp = 0;
    obj->OldTimeStamp = 0;
    obj->VirtualTimer = 0;
    obj->CmtnDelay = 0;
    obj->DelayTaskPointer = 1;
    obj->NoiseWindowMax = 0;    // Threshold how often bemf zero crossing has be be detected to obj->Trigger zero crossing flag
    obj->CmtnDelayCounter = 0;
    obj->NWDelta = 0;
    obj->NWDelayThres = 0;
    obj->GPR1_COM_TRIG = 0;
    obj->Tmp = 0;
}

// ----------------------------------------------------------------------
//
inline CMTN_noiseWindow_run(volatile CMTN_t *cmtn_obj)
{
    CMTN_t *obj = (CMTN_t*)cmtn_obj;

    // noise window is fixed value
    if (obj->CmtnDelay >= obj->NWDelayThres)
    {
        obj->NoiseWindowMax = obj->NWDelayThres - obj->NWDelta;
    }
    // noise window adjusted dynamically
    else
    {
        obj->NoiseWindowMax = obj->CmtnDelay - obj->NWDelta;
    }

    obj->NoiseWindowCounter += 1;

    // zc must occur max_noise_window times
    if (obj->NoiseWindowCounter == obj->NoiseWindowMax)
    {
        obj->ZcTrig = true;
        obj->NoiseWindowCounter = 0;
    }
}

// ----------------------------------------------------------------------
//
inline CMTN_delay30deg_run(volatile CMTN_t *cmtn_obj)
{
    CMTN_t *obj = (CMTN_t*)cmtn_obj;

    if(false == obj->Delay30DoneFlag)
    {
        obj->OldTimeStamp = obj->NewTimeStamp;
        obj->NewTimeStamp = obj->VirtualTimer;
        obj->Tmp = obj->NewTimeStamp - obj->OldTimeStamp;

        if(obj->Tmp > 0)
        {
            obj->RevPeriod = obj->Tmp;
        }
        else
        {
            // If Period is negative, allow "wrapping"
            obj->RevPeriod = 0x7FFF + obj->Tmp;
        }

        //  factor 1/12 comes from 30° delay at one full turn (360°):
        //  1/12 ^= 30/360
        obj->CmtnDelay = obj->RevPeriod/12; // devision quotient
        obj->GPR1_COM_TRIG = obj->RevPeriod - obj->CmtnDelay*12; // devision reminder
        if(obj->GPR1_COM_TRIG >= 6)
        {
            obj->CmtnDelay += 1;
        }

        obj->Delay30DoneFlag = true;
    }
}

// ----------------------------------------------------------------------
//
inline void CMTN_run(volatile CMTN_t *cmtn_obj,
                     MOTOR_3PHASES_t voltage,
                     CMTN_STATE_T cmtnState,
                     uint16_t timer)
{
    CMTN_t *obj = (CMTN_t*)cmtn_obj;

    _q voltage3Vn, debugBemf;

    // always clear flags on entry:
    obj->ZcTrig = 0;

    obj->VirtualTimer = timer;

    // Neutral voltage calculation (3*motor Neutral voltage):
    // Neutral is 3x the star connection voltage referenced to ground (Neutral = 3Vn)
    voltage3Vn = voltage.As + voltage.Bs + voltage.Cs;

    // Commutation State table Tasks; must correspond to bldc_1xPwm.h table!
    switch(cmtnState)
    {
    case Cmtn_A_B: // State s1: current flows to motor windings from voltage A->B, de-energized voltage = C

        debugBemf = _Qmpy(_Q(3.0),voltage.Cs) - voltage3Vn;

        //  As we are interested in the zero crossing of the Bemf it is possible to check only for
        //  the Bemf sign change

        if (debugBemf > 0)
        {
            obj->NoiseWindowCounter = 0;
        }
        else   /*  Zero crossing Noise window processing*/
        {
            CMTN_noiseWindow_run(obj);
        }
        break;
    case Cmtn_A_C: // State s2: current flows to motor windings from voltage A->C, de-energized voltage = B

        debugBemf = _Qmpy(_Q(3.0),voltage.Bs) - voltage3Vn;

        if (debugBemf < 0)
        {
            obj->NoiseWindowCounter = 0;
        }
        else   /*  Zero crossing Noise window processing*/
        {
            CMTN_noiseWindow_run(obj);
        }
        break;
    case Cmtn_B_C: // State s3: current flows to motor windings from voltage B->C, de-energized voltage = A

        debugBemf = _Qmpy(_Q(3.0),voltage.As) - voltage3Vn;

        if (debugBemf > 0)
        {
            obj->NoiseWindowCounter = 0;
        }
        else   /*  Zero crossing Noise window processing*/
        {
            CMTN_noiseWindow_run(obj);
        }
        break;
    case Cmtn_B_A: // State s4: current flows to motor windings from voltage B->A, de-energized voltage = C

        debugBemf = _Qmpy(_Q(3.0),voltage.Cs) - voltage3Vn;

        if (debugBemf < 0)
        {
            obj->NoiseWindowCounter = 0;
        }
        else   /*  Zero crossing Noise window processing*/
        {
            CMTN_noiseWindow_run(obj);
        }
        break;
    case Cmtn_C_A: // State s5: current flows to motor windings from voltage C->A, de-energized voltage = B

        obj->Delay30DoneFlag = false; // clear flag for delay calculation in State 5
        debugBemf = _Qmpy(_Q(3.0),voltage.Bs) - voltage3Vn;

        if (debugBemf > 0)
        {
            obj->NoiseWindowCounter = 0;
        }
        else   /*  Zero crossing Noise window processing*/
        {
            CMTN_noiseWindow_run(obj);
        }
        break;
    case Cmtn_C_B: // State s6: current flows to motor windings from voltage C->B, de-energized voltage = A

        debugBemf = _Qmpy(_Q(3.0),voltage.As) - voltage3Vn;

        if (debugBemf < 0)
        {
            obj->NoiseWindowCounter = 0;
        }
        else   /*  Zero crossing Noise window processing*/
        {
            CMTN_noiseWindow_run(obj);
        }

        CMTN_delay30deg_run(obj);
        break;
    }

    obj->Trigger = false;

    // obj->DelayTaskPointer = 1 for check obj->Trigger
    if(obj->DelayTaskPointer > 0)
    {
        if(obj->ZcTrig != 0)
        {
            // Subtract NoiseWindowMax to compensate the advanced zero-crossing validation point
            obj->CmtnDelayCounter = obj->CmtnDelay - obj->NoiseWindowMax;
            obj->DelayTaskPointer = 0;
        }
    }
    else     // obj->DelayTaskPointer = 0 for count down
    {
        obj->CmtnDelayCounter -= 1;
        if (obj->CmtnDelayCounter == 0)
        {
            // Yes!- Set obj->Trigger
            obj->Trigger = true;
            obj->DelayTaskPointer = 1;
        }
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
#endif /* BLDC_CMTN_H_ */

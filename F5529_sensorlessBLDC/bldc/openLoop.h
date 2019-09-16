#ifndef BLDC_OPENLOOP_H_
#define BLDC_OPENLOOP_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	openLoop.h
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
//  This file is mainly build up on the Texas Instruments "maths-blocks" found
//  in the C2000 control suite and adopted to work with Q-Formats and MPS430 types.
//
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	08.09.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
typedef volatile  struct
{
    uint16_t Delay;         // Parameter: ramp delay for next iteration
    uint16_t DelayCnt;      // Variable: Counter for delay
    uint16_t Counter;       // Variable: Period counter
    uint16_t TargetValue;   // Input: target period
    uint16_t SetpointValue; // Output: current ramp value
    uint16_t Min;           // Parameter: Minimum ramp output
    bool     Trigger;       // Output: commutation trigger flag
    bool     DoneFlag;      // Output: Flag output if ramp reached its obj->TargetValue value
} OPENLOOP_t;
// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
// function to set default values for the object
inline void OPENLOOP_objectInit(volatile OPENLOOP_t *ramp3_obj)
{
    OPENLOOP_t *obj = (OPENLOOP_t*)ramp3_obj;

    obj->Delay = 0;
    obj->DelayCnt = 0;
    obj->Counter = 0;
    obj->SetpointValue = 0;
    obj->Min = 50;
    obj->Trigger = false;
    obj->DoneFlag = false;
}

// ----------------------------------------------------------------------
//
inline void OPENLOOP_run(volatile OPENLOOP_t *ramp3_obj)
{
    OPENLOOP_t *obj = (OPENLOOP_t*)ramp3_obj;

    obj->Trigger = false;

    if (obj->SetpointValue == obj->TargetValue)
    {
        obj->DoneFlag = true;
    }
    else
    {
        obj->DelayCnt++;

        if (obj->DelayCnt >= obj->Delay)
        {
            obj->DelayCnt = 0;
            obj->SetpointValue--;

            if (obj->SetpointValue < obj->Min)
            {
                obj->SetpointValue = obj->Min;
            }
        }
    }

    if(++obj->Counter >= obj->SetpointValue)
    {
        obj->Trigger = true;
        obj->Counter = 0;
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
#endif /* BLDC_OPENLOOP_H_ */

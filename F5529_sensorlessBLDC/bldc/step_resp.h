#ifndef BLDC_STEP_RESP_H_
#define BLDC_STEP_RESP_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	step_resp.h
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
//
//  Contains routine which is used to do a current reference step.
//  This can be used to adjust the current controller gains.
//  Visualize response with the help of an oscilloscope.
//
//
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	07.09.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
typedef volatile struct
{
    uint16_t Cntr;      // actual counter value
    uint16_t CntMax;    // maximum counter value
    _q       IdcRef_pu; // holds reference current for current step [pu]
    bool     DoStep;    // flag to perform step
}STEP_RESPONSE_t;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
// function to set default values for the object
inline void STEP_objectInit(volatile STEP_RESPONSE_t *step_obj)
{
    STEP_RESPONSE_t *obj = (STEP_RESPONSE_t*)step_obj;

    obj->CntMax = 0;
    obj->Cntr = 0;
    obj->DoStep = false;
    obj->IdcRef_pu = 0;
}

// ----------------------------------------------------------------------
//  function handles step response and energizes for predefined time (ticks)
//
inline _q STEP_run(volatile STEP_RESPONSE_t *step_obj)
{
    STEP_RESPONSE_t *obj = (STEP_RESPONSE_t*)step_obj;

    _q out;

    if(true == obj->DoStep)
    {
        if(++(obj->Cntr) > obj->CntMax)
        {
            obj->DoStep = false;
            obj->Cntr = 0;
        }
        out = obj->IdcRef_pu;
    }
    else
    {
        out = 0;
    }

    return out;
}

// ----------------------------------------------------------------------
// 	end of file
// ----------------------------------------------------------------------


#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /* BLDC_STEP_RESP_H_ */

#ifndef BLDC_RAMP_H_
#define BLDC_RAMP_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	ramp.h
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
// 	02.09.2019 - initial programming

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
    _q       LowLimit_pu;       // Parameter: Minimum limit [pu]
    _q       HighLimit_pu;      // Parameter: Maximum limit [pu]
    _q       TargetValue_pu;    // Input: Target value [pu]
    _q       SetpointValue_pu;  // Parameter: Setpoint value [pu]
    _q       Out_pu;            // Output: ramp output value [pu]
    _q       StepWidth;         // Parameter: ramp step width [global Q]
    uint16_t DelayMax;          // Parameter: ramp delay for next iteration
    uint16_t DelayCnt;          // Variable: Counter for delay
    bool     EqualFlag;         // Output: Flag output (Q0) - independently with global Q
} RAMP_t;
// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
// function to set default values for the object
inline void RAMP_objectInit(volatile RAMP_t *ramp_obj)
{
    RAMP_t *obj = (RAMP_t*)ramp_obj;

    obj->LowLimit_pu  = _Q(-1.0);
    obj->HighLimit_pu = _Q(1.0);
    obj->TargetValue_pu = 0;
    obj->SetpointValue_pu = 0;
    obj->Out_pu = 0;
    obj->StepWidth = _Q(0.001);
    obj->DelayMax = 10;
    obj->DelayCnt = 0;
    obj->EqualFlag = false;
}

// ----------------------------------------------------------------------
//
inline void RAMP_run(volatile RAMP_t *ramp_obj)
{
    RAMP_t *obj = (RAMP_t*)ramp_obj;

    _q error = obj->TargetValue_pu - obj->SetpointValue_pu;

    if (_Qabs(error) >= obj->StepWidth)
    {
        obj->EqualFlag = false;

        if (++obj->DelayCnt >= obj->DelayMax)
        {
            if (obj->TargetValue_pu >= obj->SetpointValue_pu)
            {
                obj->SetpointValue_pu += obj->StepWidth;
            }
            else
            {
                obj->SetpointValue_pu -= obj->StepWidth;
            }
            obj->SetpointValue_pu  = _Qsat(obj->SetpointValue_pu, obj->HighLimit_pu, obj->LowLimit_pu);
            obj->DelayCnt = 0;
        }
    }
    else
    {
        obj->EqualFlag = true;
    }

    obj->Out_pu = obj->SetpointValue_pu;
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
#endif /* BLDC_RAMP_H_ */

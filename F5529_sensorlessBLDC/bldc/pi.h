#ifndef BLDC_PI_H_
#define BLDC_PI_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	pi.h
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

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
typedef volatile struct
{
    _q Kp;          // the proportional gain for the PI controller
    _q Ki;          // the integral gain for the PI controller

    _q Ref_pu;      // the reference value [pu]
    _q Fbk_pu;      // the feedback value [pu]

    _q OutMax_pu;   // the saturation high limit for the controller output [pu]
    _q OutMin_pu;   // the saturation low limit for the controller output [pu]
    _q Out_pu;      // the controller output [pu]

    _q Ui;          // the integrator value for the PI controller

    bool SatFlag;   // flag to signal controller saturation
} PI_t;


// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
// function to set default values for the object
inline void PI_objectInit(volatile PI_t *pPi_obj)
{
    PI_t *obj = (PI_t *)pPi_obj;

    // Function initializes the object with default values
    obj->Kp = _Q(1.0);
    obj->Ki = 0;
    obj->Fbk_pu = 0;
    obj->Ref_pu = 0;
    obj->Out_pu = 0;
    obj->OutMax_pu = 0;
    obj->OutMin_pu = 0;
    obj->Ui = 0;
    obj->SatFlag = false;
}
// ----------------------------------------------------------------------
//
inline void PI_run(volatile PI_t *pPi_obj)
{
    PI_t *obj = (PI_t *)pPi_obj;
    _q up, error, preOut;

    // Compute the controller error
    error = obj->Ref_pu - obj->Fbk_pu;

    // Compute the proportional term
    up    = _Qmpy(obj->Kp, error);

    // Compute the integral term in parallel form and saturate
    obj->Ui = _Qsat(obj->Ui + _Qmpy(obj->Ki, up), obj->OutMax_pu, obj->OutMin_pu);

    preOut = up + obj->Ui;;

    // Saturate the output
    obj->Out_pu = _Qsat(preOut, obj->OutMax_pu, obj->OutMin_pu);

    // if saturation flag is needed, comment out:
    //    obj->SatFlag = (out == preOut) ? false : true;
}

// ----------------------------------------------------------------------
//
inline _q PI_calcKp(float Ls_H, float deviceCurrent_A, float deviceVoltage_V,
                    float deviceCtrlPeriode_Sec)
{
    // calculation is based on "Betragsoptimum"
    // Kp = Ls/(2*tau)
    float x1;
    float y1;
    _q Kp;

    // multiplication with deviceCurrent_A is to get per unit values
    x1 = Ls_H * deviceCurrent_A;

    y1 = 2.0 * deviceCtrlPeriode_Sec;

    // multiplication with deviceVoltage_V is to get per unit values
    y1 = y1 * deviceVoltage_V;

    Kp = _Q(x1 / y1);

    return Kp;
}

// ----------------------------------------------------------------------
//
inline _q PI_calcKi(float Rs_Ohm, float Ls_H, float deviceCtrlPeriode_Sec)
{
    // calculation is based on "TI - MotorWare's documentation"
    float RsByLs = Rs_Ohm / Ls_H;
    _q Ki = _Q(RsByLs * deviceCtrlPeriode_Sec);
    return Ki;
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
#endif /* BLDC_PI_H_ */

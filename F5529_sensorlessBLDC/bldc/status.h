#ifndef BLDC_STATUS_H_
#define BLDC_STATUS_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	status.h
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
typedef enum
{
    STATUS_stop = 0,        // state = stop
    STATUS_run              // state = start
}STAUTS_pwr_flag_e;

typedef enum
{
    STATUS_stepResponse = 0, // operation mode = stepResponse, only for adjustments
    STATUS_lock,             // operation mode = lock
    STATUS_alignment,        // operation mode = alignment
    STATUS_spin              // operation mode = spin
}STAUTS_operation_mode_e;

typedef volatile struct
{
    STAUTS_pwr_flag_e       PwrFlag;
    STAUTS_operation_mode_e OperationMode;
    bool                    Aligned;
}STATUS_t;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
// function to set default values for the object
inline void STATUS_objectInit(volatile STATUS_t *status_obj)
{
    STATUS_t *obj = (STATUS_t*)status_obj;

    obj->Aligned = false;
    obj->OperationMode = STATUS_lock;
    obj->PwrFlag = STATUS_stop;
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
#endif /* BLDC_STATUS_H_ */

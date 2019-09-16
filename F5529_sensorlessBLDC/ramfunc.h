#ifndef RAMFUNC_H_
#define RAMFUNC_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	ramfunc.h
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
// 	08.09.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------
#include "string.h"
#include "stdlib.h"

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------
#define RAMFUNCS_SIZE   0x0800
#define FLASH_START_ADD 0x4400 + (0xBB80 - RAMFUNCS_SIZE)       // Flash code starting address
#define FLASH_CODE_SIZE RAMFUNCS_SIZE                           // Function segment size to be copied
#define RAM_START_ADD   0x2400 + (0x2000 - RAMFUNCS_SIZE)       // RAM code starting address

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
extern unsigned char _RamfuncsLoadStart;
extern unsigned char _RamfuncsRunStart;
extern unsigned char _RamfuncsLoadSize;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
void copyFlashToRam (void)
{
    unsigned char *flash_start_ptr;           // Initialize pointers
    unsigned char *RAM_start_ptr;

    flash_start_ptr = (unsigned char *)FLASH_START_ADD;
    RAM_start_ptr   = (unsigned char *)RAM_START_ADD;

    // Copy flash function to RAM
    memcpy(RAM_start_ptr,flash_start_ptr,FLASH_CODE_SIZE);
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
#endif /* RAMFUNC_H_ */

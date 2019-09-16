#ifndef FIFO_H_
#define FIFO_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	fifo.h
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
//	target:		general C code
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
#define FIFO_SIZE       128
// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
typedef struct
{
    uint8_t readIdx;
    uint8_t writeIdx;
    uint8_t fifoLvl;
    uint8_t fifo[FIFO_SIZE];

    bool full;
    bool overrun;
} FIFO_t;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
//
inline void FIFO_clear(FIFO_t *fifo_obj)
{
    FIFO_t *obj = (FIFO_t*)fifo_obj;

    // get buffer size:
    uint8_t ctr;

    // clear buffer:
    for(ctr = 0; ctr < FIFO_SIZE; ctr++)
    {
        obj->fifo[ctr] = 0;
    }

    // reset pointer index:
    obj->readIdx = 0;
    obj->writeIdx = 0;

    // reset level ctr:
    obj->fifoLvl = 0;

    // reset flags:
    obj->full = false;
    obj->overrun = false;
}

// ----------------------------------------------------------------------
//
inline void FIFO_write(FIFO_t *fifo_obj, uint8_t data)
{
    FIFO_t *obj = (FIFO_t*)fifo_obj;

    obj->fifo[obj->writeIdx] = data;

    if(++obj->writeIdx > FIFO_SIZE-1)
    {
        obj->writeIdx = 0;
    }

    // increment fifo level
    if(++obj->fifoLvl > FIFO_SIZE)
    {
        obj->overrun = true;;
    }

}

// ----------------------------------------------------------------------
//
inline uint8_t FIFO_read(FIFO_t *fifo_obj)
{
    FIFO_t *obj = (FIFO_t*)fifo_obj;
    uint8_t retrunValue;

    retrunValue = obj->fifo[obj->readIdx];

    if(++obj->readIdx > FIFO_SIZE-1)
    {
        obj->readIdx = 0;
    }

    if(obj->fifoLvl > 0)
    {
        // decrement fifo level
        obj->fifoLvl -= 1;
    }

    return retrunValue;
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
#endif /* FIFO_H_ */

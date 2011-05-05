/*

Copyright (c) 2011 Adrian Gin (adrian.gin[at]gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/



#ifndef _EDRUM_AVR_SHARED_FUNCTIONS
#define _EDRUM_AVR_SHARED_FUNCTIONS

#include <stdint.h>
#include "hardwareSpecific.h"


typedef void (*PF_VOID)(void);
typedef void (*PF_usbFunctionWriteOut)(uint8_t*, uint8_t);
typedef void (*PF_USBMIDI_PutByte)(uint8_t, uint8_t);
typedef void (*PF_usbSetInterrupt)(uint8_t*, uint8_t);
typedef uint8_t (*PF_USBMIDI_GetByte)(uint8_t*, uint8_t);
typedef void (*PF_flashmem_erase)(uint32_t);
typedef void (*PF_flashmem_finalise)(uint32_t);
typedef void (*PF_flashmem_writeWord)(uint32_t, uint16_t);

#define JUMP_TABLE_FUNCTION(x) ((JUMP_TABLE_LOCATION + (4*x))/2)


static __inline__ void usbPoll(void)                                          {((PF_VOID)(JUMP_TABLE_FUNCTION(0)))();}
static __inline__ void usbFunctionWriteOut(uint8_t* data, uint8_t len)        {((PF_usbFunctionWriteOut)(JUMP_TABLE_FUNCTION(1)))(data, len);}
static __inline__ void usbSetInterrupt(uint8_t* data, uint8_t len)            {((PF_usbSetInterrupt)(JUMP_TABLE_FUNCTION(2)))(data, len);}
static __inline__ void bootloader_enter()                                     {((PF_VOID)(JUMP_TABLE_FUNCTION(3)))();}
static __inline__ void usbInit()                                              {((PF_VOID)(JUMP_TABLE_FUNCTION(4)))();}
static __inline__ void USBMIDI_PutByte(uint8_t inbyte, uint8_t cableNo)                        {((PF_USBMIDI_PutByte)(JUMP_TABLE_FUNCTION(5)))(inbyte, cableNo);}
static __inline__ void USBMIDI_OutputData()                                   {((PF_VOID)(JUMP_TABLE_FUNCTION(6)))();}
static __inline__ void USBMIDI_EnableRequests()                               {((PF_VOID)(JUMP_TABLE_FUNCTION(7)))();}
static __inline__ uint8_t USBMIDI_GetByte(uint8_t* inbyte, uint8_t cableNo)                                   {return ((PF_USBMIDI_GetByte)(JUMP_TABLE_FUNCTION(8)))(inbyte, cableNo);}
static __inline__ void _flashmem_erase(uint32_t address)                      {((PF_flashmem_erase)(JUMP_TABLE_FUNCTION(9)))(address);}
static __inline__ void _flashmem_finalise(uint32_t address)                   {((PF_flashmem_finalise)(JUMP_TABLE_FUNCTION(10)))(address);}
static __inline__ void _flashmem_writeWord(uint32_t address, uint16_t data)   {((PF_flashmem_writeWord)(JUMP_TABLE_FUNCTION(11)))(address, data);}
static __inline__ void _flashmem_release()                                    {((PF_VOID)(JUMP_TABLE_FUNCTION(12)))();}
static __inline__ void hardwareReset()                                        {((PF_VOID)(JUMP_TABLE_FUNCTION(13)))();}
 

#endif

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



/** \file UI_KP.h
	 \brief UART Commands Interface
*/

/**	 

	\ingroup edrum
 	\defgroup UI_KP Keypad Interface
 	\code #include "UI_KP.h" \endcode

 	 
	 \par Description
*/
//@{
   


#ifndef	_UI_KP_ROUTINES
#define	_UI_KP_ROUTINES

#include "version.h"

/* KeyPad Defines */
/* 1st Row */
enum 
{
   KP_0 = 0x00,
   KP_1,
   KP_2,
   KP_3,
   KP_4,
   KP_5,
   KP_6,
   KP_7,
   KP_8,
   KP_9,
   KP_A,
   KP_B,
   KP_C,
   KP_D,
   KP_STAR,
   KP_HASH,
} keypad16;

#define KP_ButtonCount  (16)
#define KP_INVALID   (0xFF)
#define KP_UPDATE    (0xFE)

#define UI_KP_GetState() UI_KP_SetState(KP_GET_STATE)

enum
{

   KP_NO_REPEAT,
   KP_REPEATING,
   KP_REPEATING5X,
   KP_REPEATING10X,
   KP_GET_STATE,

} keypadStates;

void UI_KP_Init(void);

/** Returns the KP button pressed */
uint8_t UI_KP_GetPress(void);

/* Returns 0->15 depending on input keypad input*/
uint8_t UI_KP_ReturnID(uint8_t KP_Button);

uint8_t UI_KP_SetState(uint8_t newState);

#endif

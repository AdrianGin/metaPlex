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

#include <stdint.h>
#include <stdlib.h>

#include "hardwareSpecific.h"
#include "Softtimer.h"
#include "TimerCallbacks/TimerCallbacks.h"


#define NUMBER_OF_TIMERS   (4)

/* Holds functions for the timers and is an example of implementation*/




void RunCriticalTimer(void)
{
   uint8_t i;
   for( i = 0; i < TIMER1_COUNT; i++)
   {
      /* MIDI output is in 1ms steps */
   	if(SoftTimerInterrupt(SoftTimer1[i]))
   	{
         SoftTimerReset(SoftTimer1[i]);
         SoftTimerCallback(SoftTimer1[i]);
      }	   
   }
}



void RunAuxTimers(void)
{
   uint8_t i;
   for( i = 0; i < TIMER2_COUNT; i++)
   {
      /* MIDI output is in 1ms steps */
   	if(SoftTimerInterrupt(SoftTimer2[i]))
   	{
         SoftTimerReset(SoftTimer2[i]);
         SoftTimerCallback(SoftTimer2[i]);
      }	   
   }
}


uint8_t SoftTimer_IsTimer2Active(void)
{
   uint8_t i;
   
   for( i = 0; i < TIMER2_COUNT; i++)
   {
      if( SoftTimer2[i].timerEnable )
      {
         return 1;  
      }
   }
   return 0;
}

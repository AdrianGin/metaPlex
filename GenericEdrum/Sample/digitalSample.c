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

/* Has the Channel sampling code in it */

#include <stdint.h>
#include "hardwareSpecific.h"
#include "sample.h"
#include "digitalSample.h"

/* Digital Channel Velocity Outputs */
uint8_t GetDigitalVelocity(uint8_t DigitalChannel)
{
   return DigitalSettings->DigitalVelocity[DigitalChannel];
}

void SetDigitalVelocity(uint8_t DigitalChannel, int8_t velocity)
{
	
	if( velocity < -50)
	{
		velocity = 0;	
	}
   if( velocity < 0)
   {
		velocity = MAX_VELOCITY;	
	}

   DigitalSettings->DigitalVelocity[DigitalChannel] = velocity;
}

/* To alter the switch type from Active Low/High */
uint8_t GetActivePolarity(uint8_t DigitalChannel)
{
   return (GET_BIT_FIELD(DigitalSettings->DigitalActivePolarity, DigitalChannel) != 0);
}
/* This is the only place where channel settings can be adjusted */
void SetActivePolarity(uint8_t DigitalChannel, uint8_t polarity)
{
   SET_BIT_FIELD(DigitalSettings->DigitalActivePolarity, DigitalChannel, polarity);
}

void ActivePolarityToggle(uint8_t DigitalChannel)
{
   SetActivePolarity( DigitalChannel, !GetActivePolarity(DigitalChannel) );
}


/* Trigger mode is either Single shot (needs to reset before next retrigger)
 * or continuous is triggering while switch is in active state. */
uint8_t GetTriggerMode(uint8_t DigitalChannel)
{
   return (GET_BIT_FIELD(DigitalSettings->DigitalTriggerMode, DigitalChannel) != 0);
}

/* This is the only place where channel settings can be adjusted */
void SetTriggerMode(uint8_t DigitalChannel, uint8_t mode)
{
   SET_BIT_FIELD(DigitalSettings->DigitalTriggerMode, DigitalChannel, mode);
}

void TriggerModeToggle(uint8_t DigitalChannel)
{
   SetTriggerMode( DigitalChannel, !GetTriggerMode(DigitalChannel) );
}

/* When the digital input is released, it will also trigger the closed key */
uint8_t GetActiveRelease(uint8_t DigitalChannel)
{
   return (GET_BIT_FIELD(DigitalSettings->DigitalReleaseMode, DigitalChannel) != 0);
}

/* This is the only place where channel settings can be adjusted */
void SetGetActiveRelease(uint8_t DigitalChannel, uint8_t activeRelease)
{
   SET_BIT_FIELD(DigitalSettings->DigitalReleaseMode, DigitalChannel, activeRelease);
}

void ActiveReleaseToggle(uint8_t DigitalChannel)
{
   SetGetActiveRelease( DigitalChannel, !GetActiveRelease(DigitalChannel) );
}

/* Updates the SignalPeak Variable regardless of whether the Digital Input is
 * Active,
 * 20110416 Here we shall implement a Trigger on release */
void ScanDigitalInputs(void)
{
	uint8_t i;
	uint8_t selectedDigitalInput;
	
	for( i = 0; i < DIGITAL_INPUTS; i++)
	{
	   selectedDigitalInput = i + ANALOGUE_INPUTS;
		if( GetChannelState(selectedDigitalInput) )
		{
			if( GetTriggerMode(i) == SINGLE_SHOT )
			{
				/* Schmitt Trigger Type Operation */
				if( DigitalCycle[i] == INPUT_HAS_BEEN_CYCLED )
				{
					SignalPeak[selectedDigitalInput] = INPUT_IS_DOWN;
					DigitalCycle[i] = INPUT_IS_DOWN;	
				}
				else
				{
					/* Reset Value */
					SignalPeak[selectedDigitalInput] = INPUT_IS_NOT_READY;
				}
			}
			else
			{
				SignalPeak[selectedDigitalInput] = INPUT_IS_DOWN;
				DigitalCycle[i] = INPUT_IS_DOWN;
			}
		}
		else
		{
		   /* Implement the Release function here, but
		    * if the switch is continuous, the release
		    * may not trigger, thus we need to reset the switch
		    * retrigger for this one. */
		   if( DigitalCycle[i] == INPUT_IS_DOWN )
		   {
		      SoftTimerStop(RetriggerPeriod[selectedDigitalInput]);
		      SignalPeak[selectedDigitalInput] = INPUT_IS_RELEASED;
		   }
		   else
		   {
		      /* Reset Value */
            SignalPeak[selectedDigitalInput] = 0;
		   }
		   DigitalCycle[i] = INPUT_HAS_BEEN_CYCLED;
		}

	}
	
}








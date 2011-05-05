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
#include "analogueSample.h"
#include "MIDI/midi.h"
#include "Profiles/profiles.h"
#include "SoftTimer/Softtimer.h"

#include "mmculib/log2.h"

/* Signal Peak also holds the value of the Digital State */
volatile uint16_t SignalPeak[NUMBER_OF_INPUTS];

/* The Digital Triggers must be cycled for the singleshot triggers */
uint8_t DigitalCycle[DIGITAL_INPUTS];

/* For Retrigger Timers */
volatile SoftTimer_8   RetriggerPeriod[NUMBER_OF_INPUTS];

/* This stores which analogue channels are active, this streamlines the polling process */
volatile uint8_t ActiveChannels[NUMBER_OF_INPUTS+1];

/* The functions below all work on the 'current profile */

ChannelSettings_t* ChannelSettings;

DigitalSettings_t* DigitalSettings;

GainSettings_t* GainSettings;

static uint16_t LastSampleValue[ANALOGUE_INPUTS];

void UpdateActiveChannels(void)
{
   uint8_t i;
   uint8_t ChannelIndex = 0;
   
   for( i = 0; i < ANALOGUE_INPUTS; i++)
   {
      if( GetChannelStatus(i) != 0 )
      {
         ActiveChannels[ChannelIndex++] = i;
      }
   }
   
   ActiveChannels[ChannelIndex] = LAST_CHANNEL_INDEX;
}


void ChannelToggle(uint8_t channel)
{
   SetChannelStatus( channel, !GetChannelStatus(channel) );
}

uint8_t GetChannelStatus(uint8_t channel)
{
   return GET_BIT_FIELD(ChannelSettings->ChannelStatus, channel) != 0;
}
/* This is the only place where channel settings can be adjusted */
void SetChannelStatus(uint8_t channel, uint8_t status)
{
   SET_BIT_FIELD(ChannelSettings->ChannelStatus, channel, status);
}

/* Channel Codes  */
uint8_t GetChannelCommand(uint8_t channel)
{
   return ChannelSettings->ChannelCommand[channel];
}

void SetChannelCommand(uint8_t channel, uint8_t command)
{
   if( command == 0)
   {
      command = MIDI_NOTE_OFF;  
   }
   if( command < MIDI_NOTE_OFF)
   {
      command = MIDI_RT_RESET;
   }   

   ChannelSettings->ChannelCommand[channel] = command;
}

uint8_t GetClosedChannelCommand(uint8_t channel)
{
   return ChannelSettings->ClosedChannelCommand[channel];
}

void SetClosedChannelCommand(uint8_t channel, uint8_t command)
{
   if( command == 0)
   {
      command = MIDI_NOTE_OFF;
   }
   if( command < MIDI_NOTE_OFF)
   {
      command = MIDI_RT_RESET;
   }

   ChannelSettings->ClosedChannelCommand[channel] = command;
}


/* General Channel Key Notes, and also the Dual Input Open Notes */
uint8_t GetChannelKey(uint8_t channel)
{
   return ChannelSettings->ChannelKey[channel];
}

void SetChannelKey(uint8_t channel, uint8_t key)
{
	/* Coming from 127 + 10 will be less than -50 */
   if( key == UINT8_MAX)
   {
      key = MIDI_MAX_KEY;
   }
   /* zero - 15 will be less than 0, but not less than -50 */
   if( key == MIDI_MAX_KEY + 1)
   {
      key = 0;
   }   

   ChannelSettings->ChannelKey[channel] = key;
}

/* Close Channel Key Notes for Dual Input*/
uint8_t GetChannelKeyClosed(uint8_t channel)
{
   return ChannelSettings->ClosedChannelKey[channel];
}

void SetChannelKeyClosed(uint8_t channel, int8_t key)
{
   if( key < -50)
   {
      key = 0;  
   }
   if( key < 0)
   {
      key = MIDI_MAX_KEY;  
   }   

   ChannelSettings->ClosedChannelKey[channel] = key;
}



/* Setup Channel Retrigger times */
uint8_t GetChannelReTrig(uint8_t channel)
{
   return ChannelSettings->ChannelRetrigger[channel];
}

void SetChannelReTrig(uint8_t channel, int16_t retrig)
{
   if( retrig > MAX_RETRIGGER)
   {
      retrig = MIN_RETRIGGER;  
   }
   if( retrig < MIN_RETRIGGER)
   {
		retrig = MAX_RETRIGGER;	
	}   
   ChannelSettings->ChannelRetrigger[channel] = retrig;
}




/* Setup Dual Trigger Inputs */
uint8_t GetDualMode(uint8_t channel)
{
   return GET_BIT_FIELD(ChannelSettings->HasDualInput, channel) != 0;
}

void DualModeToggle(uint8_t channel)
{
   SetDualMode( channel, !GetDualMode(channel) );
}

void SetDualMode(uint8_t channel, uint8_t dualInputMode)
{
   SET_BIT_FIELD(ChannelSettings->HasDualInput, channel, dualInputMode);
}

/* Dual Input digital triggering map */
/* The channel passed is the Analogue channel */
uint8_t GetTrigger(uint8_t channel)
{
	return ChannelSettings->AltTrigger[channel];
}

void SetTrigger(uint8_t channel, uint8_t triggerChannel)
{
   if( triggerChannel == 0xFF )
   {
      triggerChannel = NUMBER_OF_INPUTS-1;
   }
   if( triggerChannel == NUMBER_OF_INPUTS)
   {
      triggerChannel = 0;
   }
   ChannelSettings->AltTrigger[channel] = triggerChannel;
}




/* Returns either 1 or 0 to represent whether there is an active signal on
 * the channel.
 */
uint8_t GetChannelState(uint8_t channel)
{
   if( channel >= ANALOGUE_INPUTS )
   {
      /* Need to minus the offset */
      return GetDigitalState(channel-ANALOGUE_INPUTS) == GetActivePolarity(channel-ANALOGUE_INPUTS);
   }
   else
   {
      return (SignalPeak[channel] != 0);
   }
}

/* Initiate changes */
void UpdateChannelRetriggers(void)
{
   uint8_t i;
   /* Update the Retrigger periods */
   for( i = 0; i < NUMBER_OF_INPUTS; i++ )
   {
      RetriggerPeriod[i].timeCompare = GetChannelReTrig(i);
      SoftTimerReset(RetriggerPeriod[i]);
   }
}



void SetLastSampleValue(uint8_t channel, uint16_t value)
{
		/* Update the last sample */
   LastSampleValue[channel] = value;
}

uint16_t GetLastSampleValue(uint8_t channel)
{
	return LastSampleValue[channel];	
}

void ResetValues(void)
{
   uint8_t i;
   for( i = 0; i < NUMBER_OF_INPUTS; i++)
   {
      SignalPeak[i] = 0;  
   }
}

/* Peaks must be larger than the threshold to trigger, else 0
 * The sample is compared against the previous samples of the channel */
/* Channel must be <= CH15 */ 


uint8_t ObtainPeak(uint8_t channel, uint16_t sample)
{
   /* This must come first, otherwise, two consecutive samples
    * either side of the threshold will not result in a trigger
    */
   if( sample < SignalPeak[channel] )
   {
      return SAMPLE_IS_FALLING;
   }

   if( sample < ChannelSettings->ChannelThreshold[channel] )
   {
      return SAMPLE_BELOW_THRESHOLD;  
   }

   /* Update the largest sample */
   SignalPeak[channel] = sample;
   return SAMPLE_IS_PEAK;
}








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
#include "analogueSample.h"

PROGRAM_CHAR PRESET_1[] = "Exponential X";
PROGRAM_CHAR PRESET_2[] = "Exponential";
PROGRAM_CHAR PRESET_3[] = "Logorithmic";
PROGRAM_CHAR PRESET_4[] = "Logorithmic X";
PROGRAM_CHAR PRESET_5[] = "Custom";

PROGRAM_PTR PresetGainStrings[] = {PRESET_1, PRESET_2, PRESET_3, PRESET_4, PRESET_5};

#define EXPONENTIALX_CROSSOVER      (((1 << ADC_RESOLUTION)*24)/30)
#define EXPONENTIAL_CROSSOVER       (((1 << ADC_RESOLUTION)*17)/30)
#define LOGARITHMIC_CROSSOVER       (((1 << ADC_RESOLUTION)*10)/30)
#define LOGARITHMIC_X_CROSSOVER     (((1 << ADC_RESOLUTION)*6)/30)

const int8_t PresetGain1[] = {5, 5 , 8, 9};
const int8_t PresetGain2[] = {9, 8 , 6, 5};

const int16_t PresetGainCrossover[] = 
{
   EXPONENTIALX_CROSSOVER, 
   EXPONENTIAL_CROSSOVER,
   LOGARITHMIC_CROSSOVER, 
   LOGARITHMIC_X_CROSSOVER
};



/* Channel Threshold */
uint16_t GetChannelThresh(uint8_t channel)
{
   return ChannelSettings->ChannelThreshold[channel];
}

void SetChannelThresh(uint8_t channel, int16_t thresh)
{
   if( thresh > MAX_THRESHOLD)
   {
      thresh = MIN_THRESHOLD;  
   }
   if( thresh < MIN_THRESHOLD )
   {
		thresh = MAX_THRESHOLD;	
	}
   ChannelSettings->ChannelThreshold[channel] = thresh;
}


/****** GAIN SETTINGS ******/
/* Channel Gain 1st Slope*/
int8_t GetChannelGain(uint8_t channel)
{
   return GainSettings->ChannelGain[channel];
}

void SetChannelGain(uint8_t channel, int8_t Gain)
{
   if( Gain > MAX_GAIN)
   {
      Gain = MIN_GAIN;  
   }
   if( Gain < MIN_GAIN)
   {
      Gain = MAX_GAIN;  
   }   

   GainSettings->ChannelGain[channel] = Gain;
}


/* Channel Gain 2nd Slope*/
int8_t GetSlope2Gain(uint8_t channel)
{
   return GainSettings->ChannelGain2[channel];
}

void SetSlope2Gain(uint8_t channel, int8_t Gain)
{
   if( Gain > MAX_GAIN)
   {
      Gain = MIN_GAIN;  
   }
   if( Gain < MIN_GAIN)
   {
      Gain = MAX_GAIN;  
   }   

   GainSettings->ChannelGain2[channel] = Gain;
}



/* Gain Type setup */
uint8_t GetGainType(uint8_t channel)
{
   return (GET_BIT_FIELD(GainSettings->GainType, channel) != 0);
}

/* This is the only place where channel settings can be adjusted */
void SetGainType(uint8_t channel, uint8_t status)
{
   SET_BIT_FIELD(GainSettings->GainType, channel, status);
}


void GainTypeToggle(uint8_t channel)
{
   SetGainType( channel, !GetGainType(channel) );
}



/* Gain Crossover Levels */
int16_t GetCrossover(uint8_t channel)
{
   return GainSettings->Crossover[channel];
}

void SetCrossover(uint8_t channel, int16_t crossover)
{
   if( crossover > MAX_CROSSOVER)
   {
      crossover = MIN_CROSSOVER;  
   }
   if( crossover < MIN_CROSSOVER)
   {
		crossover = MAX_CROSSOVER;	
	}
   
   GainSettings->Crossover[channel] = crossover;
}

/* Returns the conditioned signal after being passed through the
 * configured gain settings */
uint16_t GainFunction(uint8_t channel, uint16_t signalValue)
{
	if( GetGainType(channel) == NON_LINEAR_GAIN )
	{
		/* Signal > Crossover */
		uint16_t crossover = GetCrossover(channel);
		
      int16_t signalOffset = signalValue - crossover;
		if( (signalOffset > 0) )
		{
			return ApplyGain(signalOffset , GetSlope2Gain(channel)) +
					 ApplyGain(crossover, GetChannelGain(channel));	
		}	
	}

	return ApplyGain(signalValue, GetChannelGain(channel));		
}

/* The signal is multiplied by 2^ (gain - ADCBitResolution) */
uint16_t ApplyGain(uint16_t signalValue, int8_t gain)
{
   int8_t gainToApply = gain - GAIN_OFFSET;
   
	if( (gainToApply) > 0 )
	{
		signalValue = signalValue << gainToApply;
	}
	else
	{
		signalValue = signalValue >> (-gainToApply);	
	}
	
	return signalValue;
	
}






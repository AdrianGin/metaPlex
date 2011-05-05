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

/* Sensory and channel control */



#include <stdint.h>
#include "hardwareSpecific.h"
//#include "ADC/adc12.h"
#include "sensor.h"


SensorSettings_t* SensorSettings;


/* Init the ports as low outputs */
void SensorInit(void)
{
   CHSELDIR |=  (CHSELPINS);
//   CHSELSEL &= ~(CHSELPINS);
   CHSELOUT &= ~(CHSELPINS);    
}

/* Changes the sampling channel to channel, CH0 -> CH15 
 * If a valid sample is taken, set the input port to an output and
 * pull it down.
 */
/* This should be in hardwareSpecific.c
void SensorChannel(uint8_t channel)
{
	
	uint8_t channelState;
	
   if( channel >= CHANNEL_COUNT )
   {
      return;    
   }
   
   channel = ~channel;
   
   channelState = (((channel & (0x08)) ? 1 : 0) << CHSELD) |
                  (((channel & (0x04)) ? 1 : 0) << CHSELC) |
                  (((channel & (0x02)) ? 1 : 0) << CHSELB) |
                  (((channel & (0x01)) ? 1 : 0) << CHSELA);

   CHSELOUT = (CHSELOUT & ~(CHSELPINS)) | channelState;
   return;
}
*/

/* New Port must be within INCH_A0 -> INCH_Ax */
void SensorInputSelect(uint8_t newPort)
{
   SensorSettings->SensorInputPort = newPort;
   ADC_SetupAddress(newPort);
}

uint8_t GetSensorInput(void)
{
	return SensorSettings->SensorInputPort;
}



/* Crosstalk controls the delay between changing the channel */
int16_t GetCrossTalkDelay(void)
{
	return SensorSettings->CrosstalkDelay;
}


void SetCrossTalkDelay(int16_t newCrosstalk)
{
	SensorSettings->CrosstalkDelay = newCrosstalk;
}


uint16_t SensorPotValue(void)
{
   ADC_SetupAddress(POT_INPUT);
   uint16_t PotValue = ADC_Sample();
   ADC_SetupAddress(ADC_CHANNEL(DEFAULT_ADC_CHANNEL)); 

	return PotValue;
}

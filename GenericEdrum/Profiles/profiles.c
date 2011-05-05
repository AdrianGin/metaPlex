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

#include <string.h>
#include "hardwareSpecific.h"
#include "profiles.h"
#include "MIDICodes/MIDICodes.h"
#include "flashmem/flashmem.h"

Profile_t CurrentProfile; 


/* Sets up the clock with MCLK*/
void ProfileInit(void)
{
   /* Divide 8MHZ by 20 as clk range must be within 257 -> 457kHz*/
   //FCTL2 = FWKEY | FSSEL_MCLK | ((F_CPU / 250000) - 1);
   
   MIDISettings = &CurrentProfile.midiSettings;
   ChannelSettings = &CurrentProfile.channelSettings;
   DigitalSettings = &CurrentProfile.digitalSettings;
   GainSettings = &CurrentProfile.gainSettings;
   SensorSettings = &CurrentProfile.sensorSettings;

}



/* Writes profile data to the profile Index,
   Profile Index (0 -> 3) */
void Profile_Write(Profile_t* profile, uint8_t profileIndex)
{
   void* memPtr = (void*)PROFILE(profileIndex);

#if PROFILE_MEMORY == PROFILE_EEPROM
   eeprom_update_block((void*)profile, memPtr, sizeof(Profile_t));
#else
   flashmem_bufferedWrite( (uint32_t)memPtr, (void*)profile, sizeof(Profile_t), 0 );
#endif

}

/* Reads the passed profileIndex into the settings */
void Profile_Read(uint8_t profileIndex)
{
	PROFILE_COPY((void *)&CurrentProfile,
		          (void *)PROFILE(profileIndex), sizeof(Profile_t));
			 	  		 
}

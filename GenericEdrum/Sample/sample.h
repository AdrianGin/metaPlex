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



#ifndef  SAMPLE_H
#define  SAMPLE_H


#include "hardwareSpecific.h"
#include "SoftTimer/Softtimer.h"

/* Defined in Hardware Specific */
//#define ANALOGUE_INPUTS	(16)
//#define METRONOME_INPUTS (8)
//#define DIGITAL_INPUTS	(8)
//#define KEYPAD_INPUTS (16)
#define NUMBER_OF_REAL_INPUTS (ANALOGUE_INPUTS + DIGITAL_INPUTS)
#define NUMBER_OF_SWITCH_INPUTS (DIGITAL_INPUTS + METRONOME_INPUTS + KEYPAD_INPUTS)
#define NUMBER_OF_INPUTS   (ANALOGUE_INPUTS + DIGITAL_INPUTS + METRONOME_INPUTS + KEYPAD_INPUTS)


#define LAST_CHANNEL_INDEX (0xFF)

#define MAX_VELOCITY			(127)
#define DEFAULT_VELOCITY (0x64)

/* Retrigger Defines , in 10ms increments */
#define MAX_RETRIGGER      (255)
/* For BPM conversion */
#define RETRIGGER_RESOLUTION (100*60)
#define DEFAULT_RETRIGGER	(2)
#define MIN_RETRIGGER         (1)
#define RETRIGGER_ADJUST   (ADC_RESOLUTION - 8)

#define CHANNEL_ON 	1
#define CHANNEL_OFF 	0

/* Dual Input Defines */
#define HAS_DUAL_INPUT	(1)
#define NO_DUAL_INPUT	(0)

/* Obtain Peak defines */
#define SAMPLE_IS_PEAK (0)
#define SAMPLE_IS_FALLING (1)
#define SAMPLE_BELOW_THRESHOLD (2)

#define GET_BIT_FIELD(FIELD, BIT) (FIELD[BIT>>3] & (1<<(BIT-((BIT>>3)<<3))))
#define SET_BIT_FIELD(FIELD, BIT, STATE)  FIELD[BIT>>3] &= ~(1<<(BIT-((BIT>>3)<<3)));  FIELD[BIT>>3] |= (STATE<<(BIT-((BIT>>3)<<3)))


typedef struct {

	/* For both Analogue and Digital Channels
	 * Bitmap (ON / OFF) of the Channel status. */
   uint8_t  ChannelStatus[(NUMBER_OF_INPUTS+8)/8];

   /* Each Channel key has a corresponding channel command */
   uint8_t   ChannelCommand[NUMBER_OF_INPUTS];
   
   /* For Open notes */
   uint8_t   ChannelKey[NUMBER_OF_INPUTS];

   uint16_t  ChannelThreshold[ANALOGUE_INPUTS];
   
   /* in 10ms resolution for both Analogue and Digital Settings */
   uint8_t	 ChannelRetrigger[NUMBER_OF_INPUTS];
   
   /* Associating an input with another one */
   uint8_t	 HasDualInput[(NUMBER_OF_INPUTS+8)/8];

   /* The channels can have any input as it's dual trigger */
   uint8_t	 AltTrigger[NUMBER_OF_INPUTS];

   /* For the 'closed' notes */
   uint8_t	 ClosedChannelKey[NUMBER_OF_INPUTS];

   /* Each Channel key has a corresponding channel command */
   uint8_t   ClosedChannelCommand[NUMBER_OF_INPUTS];
	
} ChannelSettings_t;


typedef struct {

	/* Slope 1 Gradient */
   int8_t   ChannelGain[ANALOGUE_INPUTS];	
   /* Slope 2 Gradient */
   int8_t	ChannelGain2[ANALOGUE_INPUTS];
   
   /* Crossover Level from Gain 1 to Gain 2*/
   int16_t  Crossover[ANALOGUE_INPUTS];
	
   /* Linear or non linear gain */
	uint16_t   GainType[(ANALOGUE_INPUTS+8)/8];
	
} GainSettings_t;

typedef struct {

   /* The velocity of the digital outputs is constant */
   uint8_t	 DigitalVelocity[NUMBER_OF_SWITCH_INPUTS];
   
   /* Sets the input to either active high/low */
   uint8_t 	 DigitalActivePolarity[(NUMBER_OF_SWITCH_INPUTS+8)/8];
   /* Either single shot (needs to be reset) or continuous */
   uint8_t	 DigitalTriggerMode[(NUMBER_OF_SWITCH_INPUTS+8)/8];
   /* Trigger the closed key on release */
   uint8_t   DigitalReleaseMode[(DIGITAL_INPUTS+8)/8];


} DigitalSettings_t;


extern volatile uint16_t SignalPeak[];
extern volatile SoftTimer_8   RetriggerPeriod[];
extern uint8_t DigitalCycle[];
extern volatile uint8_t ActiveChannels[];

extern ChannelSettings_t* ChannelSettings;
extern DigitalSettings_t* DigitalSettings;
extern GainSettings_t*	 GainSettings;

void ResetValues(void);

void UpdateActiveChannels(void);

uint8_t ObtainPeak(uint8_t channel, uint16_t sample);

void ChannelToggle(uint8_t channel);
uint8_t GetChannelStatus(uint8_t channel);
void SetChannelStatus(uint8_t channel, uint8_t status);

/* Channel Commands */
uint8_t GetChannelCommand(uint8_t channel);
void SetChannelCommand(uint8_t channel, uint8_t command);

uint8_t GetClosedChannelCommand(uint8_t channel);
void SetClosedChannelCommand(uint8_t channel, uint8_t command);

/* Channel Key */
uint8_t GetChannelKey(uint8_t channel);
void SetChannelKey(uint8_t channel, uint8_t key);

/* Closed Channel Keys */
uint8_t GetChannelKeyClosed(uint8_t channel);
void SetChannelKeyClosed(uint8_t channel, int8_t key);

/* Channel Thresholds */
uint16_t GetChannelThresh(uint8_t channel);
void SetChannelThresh(uint8_t channel, int16_t thresh);

/* Channel Retrigger */
uint8_t GetChannelReTrig(uint8_t channel);
void SetChannelReTrig(uint8_t channel, int16_t retrig);
void UpdateChannelRetriggers(void);

/* Dual Input */
uint8_t GetDualMode(uint8_t channel);
void DualModeToggle(uint8_t channel);
void SetDualMode(uint8_t channel, uint8_t dualInputMode);


/* The channel passed is the Analogue channel */
uint8_t GetTrigger(uint8_t channel);
void SetTrigger(uint8_t channel, uint8_t triggerChannel);

/* Returns either 1 or 0 to represent whether there is an active signal on
 * the channel.
 */
uint8_t GetChannelState(uint8_t channel);

/* Channel Gain */
int8_t GetChannelGain(uint8_t channel);
void SetChannelGain(uint8_t channel, int8_t Gain);

/* Channel Gain 2nd Slope*/
int8_t GetSlope2Gain(uint8_t channel);
void SetSlope2Gain(uint8_t channel, int8_t Gain);

/* Gain Type setup */
uint8_t GetGainType(uint8_t channel);
void GainTypeToggle(uint8_t channel);
void SetGainType(uint8_t channel, uint8_t status);

/* Gain Settings */
int16_t GetCrossover(uint8_t channel);
void SetCrossover(uint8_t channel, int16_t  crossover);

/* Returns the conditioned signal after being passed through the
 * configured gain settings */
uint16_t GainFunction(uint8_t channel, uint16_t signalValue);
/* The signal is multiplied by 2^ (gain) */
uint16_t ApplyGain(uint16_t signalValue, int8_t gain);

void SetLastSampleValue(uint8_t channel, uint16_t value);
uint16_t GetLastSampleValue(uint8_t channel);


#endif


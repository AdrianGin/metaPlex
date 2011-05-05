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

#ifndef  ANALOGUESAMPLE_H
#define  ANALOGUESAMPLE_H


#include "hardwareSpecific.h"

/* The number of bits in the ADC Reading */
#define MAX_THRESHOLD      (1 << ADC_RESOLUTION)
/* Set to x% of the ADC input */
#define DEFAULT_THRESHOLD	(MAX_THRESHOLD / 50)
#define MIN_THRESHOLD      (0)
/* Number of levels actually equals 2^(ADC_RESOLUTION - THRESHOLD_LEVELS) */
#define THRESHOLD_LEVELS	(ADC_RESOLUTION - 5)
#define THRESHOLD_ADJUST   (ADC_RESOLUTION - THRESHOLD_LEVELS)

/* Gain Curve Defines */
#define LINEAR_GAIN		(0)
#define NON_LINEAR_GAIN	(1)

/* Amplification */
#define MAX_GAIN           (ADC_RESOLUTION + 7)
/* Attenuation */
#define MIN_GAIN				(1)
#define GAIN_OFFSET        (ADC_RESOLUTION) //Effective number of bits of 
#define MAX_CROSSOVER	(1 << ADC_RESOLUTION)
#define MIN_CROSSOVER	(0)

/* Most significant seven bits */
#define DEFAULT_GAIN		(7)

enum {
	EXPONENTIAL_1 = 0,
	LOGORITHMIC_1,
	EXPONENTIAL_2,
	LOGORITHMIC_2,
	CUSTOM,
	NUMBER_OF_GAIN_PRESETS
};

extern PROGRAM_PTR PresetGainStrings[];
extern const int8_t PresetGain1[];
extern const int8_t PresetGain2[];
extern const int16_t PresetGainCrossover[];

/* Channel Thresholds */
uint16_t GetChannelThresh(uint8_t channel);
void SetChannelThresh(uint8_t channel, int16_t thresh);

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


#endif


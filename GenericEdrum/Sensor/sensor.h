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



#ifndef SENSOR_H
#define SENSOR_H



#define MAX_CROSSTALK	(1000)
#define DEFAULT_CROSSTALK (36)
#define MIN_CROSSTALK	(0)

#define CH0        (0)
#define CH1        (1)
#define CH2        (2)
#define CH3        (3)
#define CH4        (4)
#define CH5        (5)
#define CH6        (6)
#define CH7        (7)
#define CH8        (8)
#define CH9        (9)
#define CH10       (10)
#define CH11       (11)
#define CH12       (12)
#define CH13       (13)
#define CH14       (14)
#define CH15       (15)



/*
// Channel Select Defines 
// To be defined in the hardwareSpecific.h
#define CHSELOUT  PORTB
#define CHSELDIR  DDRB
#define CHSELSEL  P4SEL
#define CHSELIN   PINB

#define CHSELA     (1<<0)
#define CHSELB     (1<<1)
#define CHSELC     (1<<2)
#define CHSELD     (1<<3)

#define CHANNEL_COUNT (15)

#define CHSELPINS  ( CHSELA | CHSELB | CHSELC | CHSELD )
*/


typedef struct {

   uint16_t	SensorInputPort;
	int16_t CrosstalkDelay;
} SensorSettings_t;

extern SensorSettings_t* SensorSettings;

void SensorInit(void);
/* Changes the sampling channel to channel, CH0 -> CH15 */
//void SensorChannel(uint8_t channel);


/* New Port must be within INCH_A0 -> INCH_Ax */
void SensorInputSelect(uint8_t newPort);
uint8_t GetSensorInput(void);

/* Crosstalk controls the delay between changing the channel */
int16_t GetCrossTalkDelay(void);
void SetCrossTalkDelay(int16_t newCrosstalk);

uint16_t SensorPotValue(void);




#endif

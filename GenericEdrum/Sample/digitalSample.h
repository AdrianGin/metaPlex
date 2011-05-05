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

#ifndef  DIGITALSAMPLE_H
#define  DIGITALSAMPLE_H


#include "hardwareSpecific.h"

#define ACTIVE_HIGH	(1)
#define ACTIVE_LOW	(0)

#define SINGLE_SHOT	(0)
#define CONTINUOUS	(1)

#define ACTIVE_RELEASE_DISABLED (0)
#define ACTIVE_RELEASE_ENABLED (1)


enum {
   D0 = 0,
   D1,
   D2,
   D3,
   D4,
   D5,
   D6,
   D7
} digitalPort1;

enum {
   INPUT_HAS_BEEN_CYCLED = 0,
   INPUT_IS_NOT_READY = 0,
   INPUT_IS_DOWN = 1,
   INPUT_IS_RELEASED = 2,
} digitalStates;

#define GetDigitalState(x) getDigitalState(x)

uint8_t GetDigitalVelocity(uint8_t DigitalChannel);
void SetDigitalVelocity(uint8_t DigitalChannel, int8_t velocity);

/* Switch Type */
uint8_t GetActivePolarity(uint8_t DigitalChannel);
void ActivePolarityToggle(uint8_t DigitalChannel);
void SetActivePolarity(uint8_t DigitalChannel, uint8_t polarity);

/* Trigger mode is either Single shot (needs to reset before next retrigger)
 * or continuous is triggering while switch is in active state. */
uint8_t GetTriggerMode(uint8_t DigitalChannel);
void TriggerModeToggle(uint8_t DigitalChannel);
void SetTriggerMode(uint8_t DigitalChannel, uint8_t mode);

/* Active Release */
uint8_t GetActiveRelease(uint8_t DigitalChannel);
void ActiveReleaseToggle(uint8_t DigitalChannel);
void SetGetActiveRelease(uint8_t DigitalChannel, uint8_t activeRelease);



void ScanDigitalInputs(void);


#endif


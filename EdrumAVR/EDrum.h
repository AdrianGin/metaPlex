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





#ifndef EDRUM_H
#define EDRUM_H

#include <stdint.h>

#include "hardwareSpecific.h"
#include "profiles/profiles.h"




extern volatile uint8_t ActiveProcess;
extern uint16_t BenchMarkCount;

extern const char VersionId[];

#define SET_BENCHMARK (0)
#define HAS_CONTROLLER_MODE (0)

enum {
   PLAY_MODE = 0,
   RECEIVE_SYSEX,
   SENDING_SYSEX,
   CONTROLLER_MODE,
   FIRMWARE_UPGRADE,
   USB_MIDI_THRU,
} processes;

#if PROFILE_MEMORY == PROFILE_EEPROM
   #define APP_END (FLASH_TEMP_BUFFER - 1)
#else
   #define APP_END (PROFILE_START - 1)
#endif

#define DEFAULT_PROCESS (PLAY_MODE)

void StartUp(void);
void Shutdown(void);
void Play(void);
void Benchmark(void);
void MidiControllerMode(void);

uint8_t VerifyDownload(void);

/* Interrupt Macros */
#define UI_INT_PORT  P1IN
#define UI_INT_PIN   (1<<3)


#endif

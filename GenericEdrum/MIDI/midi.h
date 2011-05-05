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



#ifndef MIDI_H
#define MIDI_H

#include <stdint.h>
#include "MIDICodes/MIDICodes.h"


/* In 1ms resolution */
#define MAX_MIDI_RATE   (50)

/*#if F_CPU == 8000000
#define BAUD_31250      (0x0100)
#define BAUD_38400      (0x00D0)
#define BAUD_115200		(0x0045)
#define BAUD_1M			(0x0008)
#else*/
/* To be defined in hardwareSpecific */
/*
#define MIDI_BAUD_31250      (F_CPU / 31250)
#define MIDI_BAUD_38400      (F_CPU / 38400)
#define MIDI_BAUD_115200		(F_CPU / 115200)
#define MIDI_BAUD_1M			(F_CPU / 1000000)
*/
//#endif


#define DEFAULT_BAUD_RATE	MIDI_BAUD_31250
#define DEFAULT_OUTPUT_RATE (1)
#define DEFAULT_MIDI_CHANNEL (9)
#define MAX_MIDI_CHANNEL (0x0F)
#define MIDI_MANUFACTURER (0x7D) /* Educational Use */
#define MIDI_DEVICE_CODE       (0x01)  /* LAD Drum */

#define MIDI_NEXT_CONTROL_CODE      (1)
#define MIDI_PREVIOUS_CONTROL_CODE  (0)

extern const char MIDI_NOTES[];
extern const char MIDI_BAUD[][11];

enum {
	B31250 = 0,
	B38400,
	B115200,
	B1M
} MIDIbauds;

typedef struct {

   uint8_t StatusCode;
   uint8_t Data[2];

} MIDI_MSG_t;

typedef struct {

   uint8_t MIDI_Commands;
   PROGRAM_CHAR* MIDI_CommandString;
   uint8_t MIDI_MsgSize;

} MidiLookup_t;

typedef struct {

	/* In 100us resolution */
   uint16_t  MIDI_OutputRate;
   uint16_t  MIDI_BaudRate;
   uint16_t	 MIDI_ChannelCode;
	
} MidiSettings_t;


extern MidiSettings_t* MIDISettings;

void MIDI_SendMsg(MIDI_MSG_t* msg);
void midiTx(uint8_t inbyte);

void MIDI_Output(void);
void MIDI_OutputAnalogueChannel(uint8_t channel);
void MIDI_DigitalOutput(void);
void MIDI_KeypadOutput(uint8_t kpButton);

uint16_t MIDI_GetBaud(void);
void MIDI_SetBaud(uint16_t newBaud);


uint8_t MIDI_GetChannelCode(void);
void MIDI_SetChannelCode(uint8_t newCode);

uint8_t MIDI_GetControlCode(uint8_t command, uint8_t direction);
void MIDI_ControlString(uint8_t command, char* buffer);
void MIDI_NoteString(uint8_t note, char* buffer);
/* Obtain the control code size */
uint8_t MIDI_CommandSize(uint8_t command);

uint8_t MIDI_Octave(uint8_t note);


void SetLastMIDIValue(uint8_t channel, uint8_t value);
uint8_t GetLastMIDIValue(uint8_t channel);

#endif

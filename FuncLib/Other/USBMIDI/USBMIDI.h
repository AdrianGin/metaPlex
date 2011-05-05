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



/* These functions convert 32bit USB data to MIDI data */
#ifndef 	_USB_MIDI
#define	_USB_MIDI

#include <stdint.h>
#include "MIDICodes/MIDICodes.h"

/* To be defined in hardwareSpecific.h */
#ifndef __HARDWARE_SPECIFIC
#define PROGRAM_SPACE PROGMEM
#define FLASH_GET_PGM_WORD(address) pgm_read_word(address)
#define FLASH_GET_PGM_BYTE(address) pgm_read_byte(address)
#endif

 /* Needs to be a power of 2 */
#define MIDI_OUT_BUFFER    (8)
#define MIDI_OUT_MASK      (MIDI_OUT_BUFFER - 1)
#define SYS_COMMON_MSG (22)
#define NO_DATA_BYTE    (0xFF)

typedef struct usbMIDIMessage
{
   uint8_t  header;
   uint8_t  MIDIData[3];
} usbMIDIMessage_t;

typedef struct {

	uint8_t byteCount;
	uint8_t bytesToReceive;
	uint8_t runningStatus;
	uint8_t lastDataByte;
	uint8_t receivingSysEx;
	usbMIDIMessage_t msg;

} usbMIDIcable_t;


/* Hold enough data for a few messages,
 */
extern usbMIDIMessage_t MIDImsgComplete[];
extern uint8_t wMIDImsgCount;
extern uint8_t rMIDImsgCount; 

extern uint8_t MIDIResponseMap[] PROGRAM_SPACE;


/* Returns the number of USB-MIDIMessages in the buffer */
uint8_t usbMIDI_bufferLen(void);

/* Returns 1 is the buffer is Ready to accept new data*/
uint8_t usbMIDI_bufferIsReady(void);

/* Returns the number of bytes of MIDI data */
/* data: Parameter: Pointer to 1 or more USB MIDI Messages */
/* RAW Midi Data is put back into data */
uint8_t usbMIDI_ParseData(uint8_t* data, uint8_t len);

/* Given the first MIDI Message, we return the
 * corresponding index value in MIDILookupTable
 * Returns: UNKNOWN_VOICE_MSG or UNKNOWN_SYS_MSG;
 * if it does not exist in the table */
uint8_t LookupMIDIMessage(uint8_t MIDIStatusCode);

/* When MIDI Data is ready
 * The function will return MIDI_DATA_READY and the
 * USB_MIDI packet will be in MIDIMessage
 * This is the MIDI parser, just keep throwing bytes into this function */
//uint8_t MIDIDataReady(uint8_t inByte, usbMIDIMessage_t* MIDIMessage);
uint8_t MIDIDataReady(uint8_t inByte, usbMIDIcable_t* usbMIDIcable);

/* Parameter: The result from 'LookupMIDIMessage' */
uint8_t GetNumberOfBytesToRead(uint8_t messageIndex);

/* Returns the CIN number, based on result from 'LookupMIDIMessage' */
uint8_t GetCINNumber(uint8_t messageIndex);



#endif

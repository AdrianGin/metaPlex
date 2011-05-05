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



#ifndef USERFUNCTIONS_H
#define USERFUNCTIONS_H

#include "Menu/menu.h"
#include "Profiles/profiles.h"
#include "hardwareSpecific.h"

#define GET (0xFF)
#define ANALOGUE_ADJUST (0)
#define DIGITAL_ADJUST (1)

#define DOWNLOAD_FAILED (1)
#define WITH_DOTS (1)

#define OPEN_KEY  (0)
#define CLOSED_KEY (1)

enum {
	MAIN_SCREEN = 0,
   TECH_SPECS,
   CONTACT_INFO,
	SIZEOFABOUT
};

extern uint8_t SelectedChannel;
extern char outputString[];

void UF_MenuSetInput(uint8_t NewInput);
void UF_stateMachine(void);
void UF_executeState(uint8_t state);
void UF_MenuReset(void);
void UF_MenuNewLine(void);
void UF_MenuChar(uint8_t data);
void UF_MenuPrint_P(PGM_P string);
void UF_MenuPrint(char* string);
void UF_MenuUpOneLevel(Menu_t* menu);

void delayWithUSBPoll(uint8_t hundredms, uint8_t withDots);
void reset(void* data);
void about(void* data);
void aboutScroll(uint8_t nameIndex);
/* Pass GET to obtain the index, otherwise the index is set to the passed
 * parameter */
uint8_t ThanksIndex(uint8_t mode);

/* ======== UTILITY FUNCTIONS ======== */
void DisableEdrum(void* data);

void USBMIDIThru(void* data);
void USBMIDI_Monitor(void *data);

void SysExDisplay(void* data);
void DumpSysEx(void* data);
void GetSysEx(void* data);

void FirmwareInstructions(void* data);
void FirmwareUpgrade(void* data);

/* Play mode disables TimerB2 */
void ControllerMode(void* data);

/* Saves the passed profile into the profileIndex */
void SaveProfile(void* data);
void LoadProfile(void* data);
void ShowProfile(void* data);
void Profile_Error(void);


void SetMIDIRate(void* data);
void EditMIDIRate(void* data);
void PrintMIDIRate(void);

void NoteEditor(void* data, uint8_t keyType);
void KeySelectFunction(void* data);
void ChannelToggleFunction(void* data);
void ChannelSetup(void* data);
void PrintNoteFormat(uint8_t keyType);
void PrintAnalogueChannelSetup(uint8_t UpDownPosition);

void SetThreshold(void* data);
void SetRetrigger(void* data);
void MonitorChannel(void* data);

void lcdProgressBar(uint16_t progress, uint16_t maxprogress, uint8_t length);
void LCD_Load_ProgressBar(void);
void SetDualInput(void* data);


void ChangeChannelCode(void* data);

void PrintDigitalChannelSetup(uint8_t UpDownPosition);
/** Function to setup each individual digial channel */
void DigitalChannelSettings(void* data);
void VelocityAdjustFunction(void* data);

/** Keypad setups */
void KeypadButtonSettings(void* data);


/* Amplifer Input Select */
void AmpInputSelect(void* data);
void SensorInputChange(void* data);
void AdjustCrosstalk(void* data);

/* VU Meter */
void VUMeterSetup(void* data);
//void DigitalVUMeterSetup(void* data);

#endif

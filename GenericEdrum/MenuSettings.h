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



#ifndef _MENU_SETTINGS_H
#define _MENU_SETTINGS_H

#include "hardwareSpecific.h"
#include "Menu/Menu.h"

/* Menu Texts */

enum {  
   ST_MAIN = VISIBLE_STATE,
   ST_CONTROLLER_MODE,
   ST_PROFILES,
   ST_OPTIONS,
   ST_MIDI_OUTPUT_RATE,
   ST_SET_RATE,
   ST_INPUT_SELECT,
   ST_CHANNEL_SETUP,
   ST_ANALOGUE_INPUTS,
   ST_DIGITAL_INPUTS,
   ST_METRONOME_INPUTS,
   ST_KEYPAD_INPUTS,
   ST_CHANNEL_1,
   ST_CHANNEL_2,
   ST_CHANNEL_3,
   ST_CHANNEL_4,
   ST_CHANNEL_5,
   ST_CHANNEL_6,
   ST_CHANNEL_7,
   ST_CHANNEL_8,
   ST_CHANNEL_9,
   ST_CHANNEL_10,
   ST_CHANNEL_11,
   ST_CHANNEL_12,
   ST_CHANNEL_13,
   ST_CHANNEL_14,
   ST_CHANNEL_15,   
   ST_CHANNEL_16,
#if ANALOGUE_INPUTS > 16
   ST_CHANNEL_17,
   ST_CHANNEL_18,
   ST_CHANNEL_19,
   ST_CHANNEL_20,
   ST_CHANNEL_21,
   ST_CHANNEL_22,
   ST_CHANNEL_23,
   ST_CHANNEL_24,
   ST_CHANNEL_25,
   ST_CHANNEL_26,
   ST_CHANNEL_27,
   ST_CHANNEL_28,
   ST_CHANNEL_29,
   ST_CHANNEL_30,
   ST_CHANNEL_31,
   ST_CHANNEL_32,
   ST_CHANNEL_33,
   ST_CHANNEL_34,
   ST_CHANNEL_35,
   ST_CHANNEL_36,
   ST_CHANNEL_37,
   ST_CHANNEL_38,
   ST_CHANNEL_39,
   ST_CHANNEL_40,
#endif
/* Effectively Channel 17 -> 32 */   
	ST_DIGITAL_1,
	ST_DIGITAL_2,
	ST_DIGITAL_3,
	ST_DIGITAL_4,
	ST_DIGITAL_5,
	ST_DIGITAL_6,
	ST_DIGITAL_7,
	ST_DIGITAL_8,
/* Metronome inputs are basically Digital inputs CH9->16 */	
   ST_METRONOME_1,
   ST_METRONOME_2,
   ST_METRONOME_3,
   ST_METRONOME_4,
   ST_METRONOME_5,
   ST_METRONOME_6,
   ST_METRONOME_7,
   ST_METRONOME_8,
   ST_METRONOME_9,
   ST_METRONOME_10,
   ST_METRONOME_11,
   ST_METRONOME_12,
   ST_METRONOME_13,
   ST_METRONOME_14,
   ST_METRONOME_15,
   ST_METRONOME_16,
/* Keypad Setting Channels */   
   ST_KEYPAD_1,
   ST_KEYPAD_2,
   ST_KEYPAD_3,
   ST_KEYPAD_4,
   ST_KEYPAD_5,
   ST_KEYPAD_6,
   ST_KEYPAD_7,
   ST_KEYPAD_8,
   ST_KEYPAD_9,
   ST_KEYPAD_10,
   ST_KEYPAD_11,
   ST_KEYPAD_12,
   ST_KEYPAD_13,
   ST_KEYPAD_14,
   ST_KEYPAD_15,   
   ST_KEYPAD_16,		
/* End of Metronome inputs */   
   ST_LOAD_PROFILE,
   ST_SAVE_PROFILE,
   ST_LOAD_PROFILE_DEF,
   ST_LOAD_PROFILE_1,
   ST_LOAD_PROFILE_2,
   ST_LOAD_PROFILE_3,
   ST_LOAD_PROFILE_4,
   ST_SAVE_PROFILE_1,
   ST_SAVE_PROFILE_2,
   ST_SAVE_PROFILE_3,
   ST_SAVE_PROFILE_4,
   ST_CHANGE_CHANNEL_CODE,
   ST_CROSSTALK,
   ST_FIXED_GAIN,
   ST_VARIABLE_GAIN,
   
   ST_VUMETER,
   ST_DIGITAL_VUMETER,
   ST_SYSTEM_SETUP,
   ST_SYSTEM_RESET,     
   ST_SYSTEM_ABOUT,
   ST_SYSTEM_UPGRADE,
   ST_SYSEX,
   
   ST_SYSEX_DUMP,
   ST_SYSEX_RECEIVE,

   ST_UTILITIES,
   ST_DISABLE_EDRUM,
   ST_USB_MIDI_THRU,
   ST_MIDI_MONITOR,
 
   
} menuIds;

/* Channel Setup Menus */
enum {
   ST_CHANNEL_TOGGLE = FIRST_STATE,
   ST_NOTE_ADJUST,
   ST_GAIN_ADJUST,
   ST_GAIN_TYPE,
   ST_GAIN_SLOPE1,
   ST_GAIN_SLOPE2,
   ST_GAIN_CROSSOVER,
   ST_GAIN_PRESET,
   ST_VELOCITY_ADJUST,
   ST_SWITCH_POLARITY,
   ST_SWITCH_MODE,
   ST_SWITCH_RELEASE,
   /* The 'main' sub menu */
   ST_ANALOGUE_SETUP = VISIBLE_STATE,
   ST_DIGITAL_SETUP,
   ST_THRESHOLD,
   ST_RETRIGGER,
   ST_DUALINPUT,
   ST_MONITORCHANNEL,
   /* For digital sub menu */
   ST_TRIGGER_TYPE,
   
} channelSetupIds;


/* Channel Setup Menus */
enum {
   /* The 'main' sub menu */
   ST_DUAL_MODE_TOGGLE = FIRST_STATE,
   ST_SET_OPEN_NOTE,
   ST_SET_CLOSED_NOTE,
   ST_SET_TRIGGER_CHANNEL,
   ST_DUAL_INPUT = VISIBLE_STATE,
} dualTrigIds;

extern Menu_t primaryMenu;
extern Menu_t analogueMenu;
extern Menu_t digitalMenu;
extern Menu_t dualTrigMenu;

extern Menu_t* ActiveMenu;
extern Menu_t* SelectedSubMenu;

void LCD_NewLine(void);
void LCD_Reset(void);
void MenuLCD_SetPos(uint8_t row, uint8_t col);
void MenuLCD_String(char* string);
void MenuLCD_String_P(const char* string);
void MenuLCD_Char(uint8_t data);

/* Menu Uart Functions */
void MenuUart_NewLine(void);
void MenuUartTxString(char* string);
void MenuUartTxString_P(const char* string);
void MenuUart_Reset(void);
void MenuUart_Tx(char byte);

void MenuSetDisplay(Menu_t* menu, uint8_t display);
void Menu_UpOneLevel(Menu_t* menu);
void HandleSubMenu(void* data);

#endif

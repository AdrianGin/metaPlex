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

/* Project Specific Menus */
#include <stdint.h>
#include "MenuSettings.h"
#include "hardwareSpecific.h"
#include "userFunctions/userFunctions.h"
#include "userFunctions/gainAdjustFunctions.h"
#include "userFunctions/dualTriggerAdjustFunctions.h"
#include "userFunctions/switchTypeAdjustFunctions.h"
#include "LCDSettings.h"

#include "hardUart/hardUart.h"

#if MENU_DEBUG == 0
#include "UI_LCD/UI_LCD.h"
#include "UI_KP/UI_KP.h"
#include <string.h>
#endif





const MENU_TEXT  MT_PROFILES[] = "Profiles";
const MENU_TEXT  MT_LOAD_PROFILE[] = "Load Profile";
const MENU_TEXT  MT_SAVE_PROFILE[] = "Save Profile As...";

const MENU_TEXT  MT_PROFILE_1[] = "Dance/Techno";
const MENU_TEXT  MT_PROFILE_2[] = "Hip-Hop";
const MENU_TEXT  MT_PROFILE_3[] = "MeTaL Kit";
const MENU_TEXT  MT_PROFILE_4[] = "Special Fx";
const MENU_TEXT  MT_PROFILE_DEF[] = "Pop Kit";


const MENU_TEXT  MT_VUMETER[] = "Analogue VU Meter";
const MENU_TEXT  MT_DIGITAL_VUMETER[] = "Digital VU Meter";
const MENU_TEXT  MT_UTILITIES[] = "Utilities";
const MENU_TEXT  MT_SYSTEM_SETUP[] = "System Setup";

const MENU_TEXT  MT_DISABLE_EDRUM[] = "eDrum Output";
const MENU_TEXT  MT_USB_MIDI_THRU[] = "USB-MIDI Thru";
const MENU_TEXT  MT_MIDI_MONITOR[] = "MIDI Monitor";

const MENU_TEXT  MT_SYSTEM_RESET[] = "Reset eDrum";
const MENU_TEXT  MT_SYSTEM_ABOUT[] = "About";
const MENU_TEXT  MT_SYSTEM_SYSEX[] = "System Exclusive";
const MENU_TEXT  MT_SYSEX_DUMP[] = "Send Current Setup";
const MENU_TEXT  MT_SYSEX_RECEIVE[] = "Get SysEx Settings";
const MENU_TEXT  MT_SYSTEM_UPGRADE[] = "Upgrade Firmware";

const MENU_TEXT  MT_OPTIONS[] = "Options";
const MENU_TEXT  MT_MIDI_OUTPUT_RATE[] = "MIDI Output Rate";
const MENU_TEXT  MT_SET_RATE[] = "Edit Output Rate";
const MENU_TEXT  MT_CHANGE_CHANNEL_CODE[] = "Edit Channel Code";
const MENU_TEXT  MT_CHANNEL_SETUP[] = "Channel Settings";
const MENU_TEXT  MT_INPUT_SELECT[] = "Amp. Input Select";
const MENU_TEXT  MT_CROSSTALK[] = "Adjust Crosstalk";

const MENU_TEXT  MT_ANALOGUE_INPUTS[] = "Analogue Inputs";
const MENU_TEXT  MT_DIGITAL_INPUTS[] = "Digital Inputs";
const MENU_TEXT  MT_METRONOME_INPUTS[] = "Metronome Inputs";
const MENU_TEXT  MT_KEYPAD_INPUTS[] = "Keypad Buttons";

const MENU_TEXT  MT_CHANNEL_1[] =  "Ch 1:(Hi-Hat)";
const MENU_TEXT  MT_CHANNEL_2[] =  "Ch 2:(Hi-Hat2)";
const MENU_TEXT  MT_CHANNEL_3[] =  "Ch 3:(Snare)";
const MENU_TEXT  MT_CHANNEL_4[] =  "Ch 4:(RimShot)";
const MENU_TEXT  MT_CHANNEL_5[] =  "Ch 5:(Hi-Tom)";
const MENU_TEXT  MT_CHANNEL_6[] =  "Ch 6:(Mid-Tom)";
const MENU_TEXT  MT_CHANNEL_7[] =  "Ch 7:(Low-Tom)";
const MENU_TEXT  MT_CHANNEL_8[] =  "Ch 8:(Cymbal Z1)";
const MENU_TEXT  MT_CHANNEL_9[] =  "Ch 9:(Cymbal Z2)";
const MENU_TEXT  MT_CHANNEL_10[] = "Ch 10:(Cymbal Z3)";
const MENU_TEXT  MT_CHANNEL_11[] = "Ch 11:(Ride Z1)";
const MENU_TEXT  MT_CHANNEL_12[] = "Ch 12:(Ride Z2)";
const MENU_TEXT  MT_CHANNEL_13[] = "Ch 13:(Ride/Bell)";
const MENU_TEXT  MT_CHANNEL_14[] = "Ch 14:";
const MENU_TEXT  MT_CHANNEL_15[] = "Ch 15:";
const MENU_TEXT  MT_CHANNEL_16[] = "Ch 16:";
#if ANALOGUE_INPUTS >= 16
const MENU_TEXT  MT_CHANNEL_17[] = "Ch 17:";
const MENU_TEXT  MT_CHANNEL_18[] = "Ch 18:";
const MENU_TEXT  MT_CHANNEL_19[] = "Ch 19:";
const MENU_TEXT  MT_CHANNEL_20[] = "Ch 20:";
const MENU_TEXT  MT_CHANNEL_21[] = "Ch 21:";
const MENU_TEXT  MT_CHANNEL_22[] = "Ch 22:";
const MENU_TEXT  MT_CHANNEL_23[] = "Ch 23:";
const MENU_TEXT  MT_CHANNEL_24[] = "Ch 24:";
const MENU_TEXT  MT_CHANNEL_25[] = "Ch 25:";
const MENU_TEXT  MT_CHANNEL_26[] = "Ch 26:";
const MENU_TEXT  MT_CHANNEL_27[] = "Ch 27:";
const MENU_TEXT  MT_CHANNEL_28[] = "Ch 28:";
const MENU_TEXT  MT_CHANNEL_29[] = "Ch 29:";
const MENU_TEXT  MT_CHANNEL_30[] = "Ch 30:";
const MENU_TEXT  MT_CHANNEL_31[] = "Ch 31:";
const MENU_TEXT  MT_CHANNEL_32[] = "Ch 32:";
const MENU_TEXT  MT_CHANNEL_33[] = "Ch 33:";
const MENU_TEXT  MT_CHANNEL_34[] = "Ch 34:";
const MENU_TEXT  MT_CHANNEL_35[] = "Ch 35:";
const MENU_TEXT  MT_CHANNEL_36[] = "Ch 36:";
const MENU_TEXT  MT_CHANNEL_37[] = "Ch 37:";
const MENU_TEXT  MT_CHANNEL_38[] = "Ch 38:";
const MENU_TEXT  MT_CHANNEL_39[] = "Ch 39:";
const MENU_TEXT  MT_CHANNEL_40[] = "Ch 40:";
#endif


const MENU_TEXT  MT_DIGITAL_1[] = "Ch 1:(HiHat Pedal)";
const MENU_TEXT  MT_DIGITAL_2[] = "Ch 2:(Bass Drum)";
const MENU_TEXT  MT_DIGITAL_3[] = "Ch 3:(Bass Drum2)";
const MENU_TEXT  MT_DIGITAL_4[] = "Ch 4:";
const MENU_TEXT  MT_DIGITAL_5[] = "Ch 5:";
const MENU_TEXT  MT_DIGITAL_6[] = "Ch 6:";
const MENU_TEXT  MT_DIGITAL_7[] = "Ch 7:";
const MENU_TEXT  MT_DIGITAL_8[] = "Ch 8:";

const MENU_TEXT  MT_METRONOME_1[] = "Metronome Input 1";
const MENU_TEXT  MT_METRONOME_2[] = "Metronome Input 2";
const MENU_TEXT  MT_METRONOME_3[] = "Metronome Input 3";
const MENU_TEXT  MT_METRONOME_4[] = "Metronome Input 4";
const MENU_TEXT  MT_METRONOME_5[] = "Metronome Input 5";
const MENU_TEXT  MT_METRONOME_6[] = "Metronome Input 6";
const MENU_TEXT  MT_METRONOME_7[] = "Metronome Input 7";
const MENU_TEXT  MT_METRONOME_8[] = "Metronome Input 8";
#if METRONOME_INPUTS > 8
const MENU_TEXT  MT_METRONOME_9[] = "Metronome Input 9";
const MENU_TEXT  MT_METRONOME_10[] = "Metronome Input 10";
const MENU_TEXT  MT_METRONOME_11[] = "Metronome Input 11";
const MENU_TEXT  MT_METRONOME_12[] = "Metronome Input 12";
const MENU_TEXT  MT_METRONOME_13[] = "Metronome Input 13";
const MENU_TEXT  MT_METRONOME_14[] = "Metronome Input 14";
const MENU_TEXT  MT_METRONOME_15[] = "Metronome Input 15";
const MENU_TEXT  MT_METRONOME_16[] = "Metronome Input 16";
#endif

#if HAS_CONTROLLER_MODE
const MENU_TEXT  MT_CONTROLLER_MODE[] = "Controller Mode";
const MENU_TEXT  MT_KEYPAD_1[] = "Key 0";
const MENU_TEXT  MT_KEYPAD_2[] = "Key 1";
const MENU_TEXT  MT_KEYPAD_3[] = "Key 2";
const MENU_TEXT  MT_KEYPAD_4[] = "Key 3";
const MENU_TEXT  MT_KEYPAD_5[] = "Key 4";
const MENU_TEXT  MT_KEYPAD_6[] = "Key 5";
const MENU_TEXT  MT_KEYPAD_7[] = "Key 6";
const MENU_TEXT  MT_KEYPAD_8[] = "Key 7";
const MENU_TEXT  MT_KEYPAD_9[] = "Key 8";
const MENU_TEXT  MT_KEYPAD_10[] = "Key 9";
const MENU_TEXT  MT_KEYPAD_11[] = "Key A";
const MENU_TEXT  MT_KEYPAD_12[] = "Key B";
const MENU_TEXT  MT_KEYPAD_13[] = "Key C";
const MENU_TEXT  MT_KEYPAD_14[] = "Key D";
const MENU_TEXT  MT_KEYPAD_15[] = "Key *";
const MENU_TEXT  MT_KEYPAD_16[] = "Key #";
#endif

const MENU_TEXT  MT_THRESHOLD[] = "Set Threshold";
const MENU_TEXT  MT_TRIGGER_TYPE[] = "Switch Settings";
const MENU_TEXT  MT_RETRIGGER[] = "Set Retrigger";
const MENU_TEXT  MT_DUALINPUT[] = "Set Dual Trigger";
const MENU_TEXT  MT_MONITORCHANNEL[] = "Monitor Channel";

const MENU_TEXT  MT_FIXED_GAIN[] = "Fixed Gain (1x)";
const MENU_TEXT  MT_VARIABLE_GAIN[] = "Variable Gain";

/* Once a menu item is selected, it will either end in
 * 1) Another Menu or
 * 2) A function
 * */
const menu_list MenuState[] MENU_SPACE = {

#if HAS_CONTROLLER_MODE
	{ST_MAIN,	ST_CONTROLLER_MODE,  0},
   {ST_MAIN,   ST_OPTIONS,   1},
   {ST_MAIN,	ST_VUMETER,	  2},
   {ST_MAIN,   ST_DIGITAL_VUMETER, 3},
   {ST_MAIN, 	ST_PROFILES,  4},
   {ST_MAIN,   ST_SYSTEM_SETUP, 5},
#else
   {ST_MAIN,   ST_OPTIONS,   0},
   {ST_MAIN,	ST_VUMETER,	  1},
   {ST_MAIN,   ST_DIGITAL_VUMETER, 2},
   {ST_MAIN, 	ST_PROFILES,  3},
   {ST_MAIN,   ST_UTILITIES, 4},
   {ST_MAIN,   ST_SYSTEM_SETUP, 5},
#endif



   {ST_OPTIONS, ST_MIDI_OUTPUT_RATE,  0},
   {ST_OPTIONS, ST_CHANNEL_SETUP, 1},
   {ST_OPTIONS, ST_CHANGE_CHANNEL_CODE, 2},
   {ST_OPTIONS, ST_INPUT_SELECT, 3},
   {ST_OPTIONS, ST_CROSSTALK, 4},
	
	{ST_INPUT_SELECT, ST_FIXED_GAIN, 2},
	{ST_INPUT_SELECT, ST_VARIABLE_GAIN, 3},		
   		
   {ST_MIDI_OUTPUT_RATE, ST_SET_RATE, 1},

   {ST_CHANNEL_SETUP, ST_ANALOGUE_INPUTS,  0},
#if DIGITAL_INPUTS > 0
   {ST_CHANNEL_SETUP, ST_DIGITAL_INPUTS,  1},
   {ST_CHANNEL_SETUP, ST_METRONOME_INPUTS, 2},
#else
   {ST_CHANNEL_SETUP, ST_METRONOME_INPUTS, 1},
#endif

#if HAS_CONTROLLER_MODE
	{ST_CHANNEL_SETUP, ST_KEYPAD_INPUTS,  3},  
#endif
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_1,  0},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_2,  1},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_3,  2},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_4,  3},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_5,  4},            
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_6,  5},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_7,  6},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_8,  7},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_9,  8},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_10, 9},    
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_11, 10},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_12, 11},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_13, 12},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_14, 13},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_15, 14}, 
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_16, 15},

#if ANALOGUE_INPUTS > 16
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_17,  16},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_18,  17},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_19,  18},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_20,  19},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_21,  20},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_22,  21},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_23,  22},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_24,  23},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_25,  24},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_26, 25},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_27, 26},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_28, 27},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_29, 28},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_30, 29},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_31, 30},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_32, 31},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_33, 32},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_34, 33},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_35, 34},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_36, 35},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_37, 36},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_38, 37},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_39, 38},
   {ST_ANALOGUE_INPUTS, ST_CHANNEL_40, 39},
#endif

#if DIGITAL_INPUTS > 0
	/* Digital Config Menu's */
   {ST_DIGITAL_INPUTS, ST_DIGITAL_1,  0},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_2,  1},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_3,  2},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_4,  3},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_5,  4},            
   {ST_DIGITAL_INPUTS, ST_DIGITAL_6,  5},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_7,  6},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_8,  7},
#endif

   {ST_METRONOME_INPUTS, ST_METRONOME_1, 0},
   {ST_METRONOME_INPUTS, ST_METRONOME_2, 1},
   {ST_METRONOME_INPUTS, ST_METRONOME_3, 2},
   {ST_METRONOME_INPUTS, ST_METRONOME_4, 3},
   {ST_METRONOME_INPUTS, ST_METRONOME_5, 4},
   {ST_METRONOME_INPUTS, ST_METRONOME_6, 5},
   {ST_METRONOME_INPUTS, ST_METRONOME_7, 6},                  
   {ST_METRONOME_INPUTS, ST_METRONOME_8, 7},
#if METRONOME_INPUTS > 8
   {ST_METRONOME_INPUTS, ST_METRONOME_9, 8},
   {ST_METRONOME_INPUTS, ST_METRONOME_10, 9},
   {ST_METRONOME_INPUTS, ST_METRONOME_11, 10},
   {ST_METRONOME_INPUTS, ST_METRONOME_12, 11},
   {ST_METRONOME_INPUTS, ST_METRONOME_13, 12},
   {ST_METRONOME_INPUTS, ST_METRONOME_14, 13},
   {ST_METRONOME_INPUTS, ST_METRONOME_15, 14},
   {ST_METRONOME_INPUTS, ST_METRONOME_16, 15},
#endif

#if HAS_CONTROLLER_MODE
   {ST_KEYPAD_INPUTS, ST_KEYPAD_1,  0},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_2,  1},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_3,  2},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_4,  3},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_5,  4},            
   {ST_KEYPAD_INPUTS, ST_KEYPAD_6,  5},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_7,  6},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_8,  7},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_9,  8},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_10, 9},    
   {ST_KEYPAD_INPUTS, ST_KEYPAD_11, 10},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_12, 11},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_13, 12},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_14, 13},
   {ST_KEYPAD_INPUTS, ST_KEYPAD_15, 14}, 
   {ST_KEYPAD_INPUTS, ST_KEYPAD_16, 15},   
#endif
   
   {ST_PROFILES, ST_LOAD_PROFILE, 2},
   {ST_PROFILES, ST_SAVE_PROFILE, 3},
	
   {ST_LOAD_PROFILE, ST_LOAD_PROFILE_DEF, 0},	
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_1, 1},	   
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_2, 2},
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_3, 3},
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_4, 4},
	
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_1, 0},	   
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_2, 1},
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_3, 2},
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_4, 3},
		
   {ST_UTILITIES, ST_USB_MIDI_THRU, 0},
   //{ST_USB_MIDI_THRU, ST_MIDI_MONITOR, 1},

   {ST_UTILITIES, ST_DISABLE_EDRUM, 1},
   
	{ST_SYSTEM_SETUP, ST_SYSTEM_RESET, 0},
	{ST_SYSTEM_SETUP, ST_SYSTEM_ABOUT, 1},
	{ST_SYSTEM_SETUP, ST_SYSEX, 2},
	{ST_SYSTEM_SETUP, ST_SYSTEM_UPGRADE, 3},
	
	{ST_SYSEX, ST_SYSEX_DUMP, 1},
	{ST_SYSEX, ST_SYSEX_RECEIVE, 2},
			   
   {0, 0, 0}
};


const menu_data MenuData[] MENU_SPACE = {
   {ST_MAIN, 0, 0},
#if HAS_CONTROLLER_MODE
   {ST_CONTROLLER_MODE, MT_CONTROLLER_MODE, ControllerMode},
#endif
   {ST_PROFILES, MT_PROFILES, ShowProfile},   
   {ST_OPTIONS, MT_OPTIONS, 0},
	{ST_VUMETER, MT_VUMETER, VUMeterSetup},
	{ST_DIGITAL_VUMETER, MT_DIGITAL_VUMETER, VUMeterSetup},

   {ST_UTILITIES,     MT_UTILITIES, 0},
   {ST_DISABLE_EDRUM, MT_DISABLE_EDRUM, DisableEdrum},
   {ST_USB_MIDI_THRU, MT_USB_MIDI_THRU, USBMIDIThru},
   {ST_MIDI_MONITOR,  MT_MIDI_MONITOR, USBMIDI_Monitor},

	{ST_SYSTEM_SETUP, MT_SYSTEM_SETUP, 0},
	{ST_SYSTEM_RESET, MT_SYSTEM_RESET, reset},
	{ST_SYSTEM_ABOUT, MT_SYSTEM_ABOUT, about},
	{ST_SYSEX, MT_SYSTEM_SYSEX, SysExDisplay},
   {ST_SYSTEM_UPGRADE, MT_SYSTEM_UPGRADE, FirmwareUpgrade},
	{ST_SYSEX_DUMP, MT_SYSEX_DUMP, DumpSysEx},
	{ST_SYSEX_RECEIVE, MT_SYSEX_RECEIVE, GetSysEx},
	
   {ST_MIDI_OUTPUT_RATE, MT_MIDI_OUTPUT_RATE, SetMIDIRate},
   {ST_SET_RATE, MT_SET_RATE, EditMIDIRate},
   {ST_INPUT_SELECT, MT_INPUT_SELECT, AmpInputSelect},
   {ST_CROSSTALK, MT_CROSSTALK, AdjustCrosstalk},
 	{ST_CHANNEL_SETUP, MT_CHANNEL_SETUP, 0},
 	
   {ST_ANALOGUE_INPUTS, MT_ANALOGUE_INPUTS,  0},
   {ST_DIGITAL_INPUTS, MT_DIGITAL_INPUTS,  0},    	
   {ST_METRONOME_INPUTS, MT_METRONOME_INPUTS,  0},
#if HAS_CONTROLLER_MODE
 	{ST_KEYPAD_INPUTS, MT_KEYPAD_INPUTS,  0},
#endif 	
 	/* Analogue Menus */
   {ST_CHANNEL_1, MT_CHANNEL_1,   ChannelSetup},
   {ST_CHANNEL_2, MT_CHANNEL_2,   ChannelSetup},
   {ST_CHANNEL_3, MT_CHANNEL_3,   ChannelSetup},
   {ST_CHANNEL_4, MT_CHANNEL_4,   ChannelSetup},
   {ST_CHANNEL_5, MT_CHANNEL_5,   ChannelSetup},            
   {ST_CHANNEL_6, MT_CHANNEL_6,   ChannelSetup},
   {ST_CHANNEL_7, MT_CHANNEL_7,   ChannelSetup},
   {ST_CHANNEL_8, MT_CHANNEL_8,   ChannelSetup},
   {ST_CHANNEL_9, MT_CHANNEL_9,   ChannelSetup},
   {ST_CHANNEL_10, MT_CHANNEL_10, ChannelSetup},    
   {ST_CHANNEL_11, MT_CHANNEL_11, ChannelSetup},
   {ST_CHANNEL_12, MT_CHANNEL_12, ChannelSetup},
   {ST_CHANNEL_13, MT_CHANNEL_13, ChannelSetup},
   {ST_CHANNEL_14, MT_CHANNEL_14, ChannelSetup},
   {ST_CHANNEL_15, MT_CHANNEL_15, ChannelSetup}, 
   {ST_CHANNEL_16, MT_CHANNEL_16, ChannelSetup},
#if ANALOGUE_INPUTS > 16
   {ST_CHANNEL_17, MT_CHANNEL_17,   ChannelSetup},
   {ST_CHANNEL_18, MT_CHANNEL_18,   ChannelSetup},
   {ST_CHANNEL_19, MT_CHANNEL_19,   ChannelSetup},
   {ST_CHANNEL_20, MT_CHANNEL_20,   ChannelSetup},
   {ST_CHANNEL_21, MT_CHANNEL_21,   ChannelSetup},
   {ST_CHANNEL_22, MT_CHANNEL_22,   ChannelSetup},
   {ST_CHANNEL_23, MT_CHANNEL_23,   ChannelSetup},
   {ST_CHANNEL_24, MT_CHANNEL_24,   ChannelSetup},
   {ST_CHANNEL_25, MT_CHANNEL_25,   ChannelSetup},
   {ST_CHANNEL_26, MT_CHANNEL_26, ChannelSetup},
   {ST_CHANNEL_27, MT_CHANNEL_27, ChannelSetup},
   {ST_CHANNEL_28, MT_CHANNEL_28, ChannelSetup},
   {ST_CHANNEL_29, MT_CHANNEL_29, ChannelSetup},
   {ST_CHANNEL_30, MT_CHANNEL_30, ChannelSetup},
   {ST_CHANNEL_31, MT_CHANNEL_31, ChannelSetup},
   {ST_CHANNEL_32, MT_CHANNEL_32, ChannelSetup},
   {ST_CHANNEL_33, MT_CHANNEL_33,   ChannelSetup},
   {ST_CHANNEL_34, MT_CHANNEL_34, ChannelSetup},
   {ST_CHANNEL_35, MT_CHANNEL_35, ChannelSetup},
   {ST_CHANNEL_36, MT_CHANNEL_36, ChannelSetup},
   {ST_CHANNEL_37, MT_CHANNEL_37, ChannelSetup},
   {ST_CHANNEL_38, MT_CHANNEL_38, ChannelSetup},
   {ST_CHANNEL_39, MT_CHANNEL_39, ChannelSetup},
   {ST_CHANNEL_40, MT_CHANNEL_40, ChannelSetup},
#endif

   
	/* Digital Menus */
   {ST_DIGITAL_1, MT_DIGITAL_1,   DigitalChannelSettings},
   {ST_DIGITAL_2, MT_DIGITAL_2,   DigitalChannelSettings},
   {ST_DIGITAL_3, MT_DIGITAL_3,   DigitalChannelSettings},
   {ST_DIGITAL_4, MT_DIGITAL_4,   DigitalChannelSettings},
   {ST_DIGITAL_5, MT_DIGITAL_5,   DigitalChannelSettings},            
   {ST_DIGITAL_6, MT_DIGITAL_6,   DigitalChannelSettings},
   {ST_DIGITAL_7, MT_DIGITAL_7,   DigitalChannelSettings},
   {ST_DIGITAL_8, MT_DIGITAL_8,   DigitalChannelSettings},

	/* Digital Menus */
   {ST_METRONOME_1, MT_METRONOME_1,   DigitalChannelSettings},
   {ST_METRONOME_2, MT_METRONOME_2,   DigitalChannelSettings},
   {ST_METRONOME_3, MT_METRONOME_3,   DigitalChannelSettings},
   {ST_METRONOME_4, MT_METRONOME_4,   DigitalChannelSettings},
   {ST_METRONOME_5, MT_METRONOME_5,   DigitalChannelSettings},            
   {ST_METRONOME_6, MT_METRONOME_6,   DigitalChannelSettings},
   {ST_METRONOME_7, MT_METRONOME_7,   DigitalChannelSettings},
   {ST_METRONOME_8, MT_METRONOME_8,   DigitalChannelSettings},
#if METRONOME_INPUTS > 8
   {ST_METRONOME_9, MT_METRONOME_9,   DigitalChannelSettings},
   {ST_METRONOME_10, MT_METRONOME_10,   DigitalChannelSettings},
   {ST_METRONOME_11, MT_METRONOME_11,   DigitalChannelSettings},
   {ST_METRONOME_12, MT_METRONOME_12,   DigitalChannelSettings},
   {ST_METRONOME_13, MT_METRONOME_13,   DigitalChannelSettings},
   {ST_METRONOME_14, MT_METRONOME_14,   DigitalChannelSettings},
   {ST_METRONOME_15, MT_METRONOME_15,   DigitalChannelSettings},
   {ST_METRONOME_16, MT_METRONOME_16,   DigitalChannelSettings},
#endif


#if HAS_CONTROLLER_MODE 	
   /* Keypad Menus */
   {ST_KEYPAD_1, MT_KEYPAD_1,   KeypadButtonSettings},
   {ST_KEYPAD_2, MT_KEYPAD_2,   KeypadButtonSettings},
   {ST_KEYPAD_3, MT_KEYPAD_3,   KeypadButtonSettings},
   {ST_KEYPAD_4, MT_KEYPAD_4,   KeypadButtonSettings},
   {ST_KEYPAD_5, MT_KEYPAD_5,   KeypadButtonSettings},            
   {ST_KEYPAD_6, MT_KEYPAD_6,   KeypadButtonSettings},
   {ST_KEYPAD_7, MT_KEYPAD_7,   KeypadButtonSettings},
   {ST_KEYPAD_8, MT_KEYPAD_8,   KeypadButtonSettings},
   {ST_KEYPAD_9, MT_KEYPAD_9,   KeypadButtonSettings},
   {ST_KEYPAD_10, MT_KEYPAD_10, KeypadButtonSettings},    
   {ST_KEYPAD_11, MT_KEYPAD_11, KeypadButtonSettings},
   {ST_KEYPAD_12, MT_KEYPAD_12, KeypadButtonSettings},
   {ST_KEYPAD_13, MT_KEYPAD_13, KeypadButtonSettings},
   {ST_KEYPAD_14, MT_KEYPAD_14, KeypadButtonSettings},
   {ST_KEYPAD_15, MT_KEYPAD_15, KeypadButtonSettings}, 
   {ST_KEYPAD_16, MT_KEYPAD_16, KeypadButtonSettings},
#endif
	 
	{ST_FIXED_GAIN, MT_FIXED_GAIN, SensorInputChange},
	{ST_VARIABLE_GAIN, MT_VARIABLE_GAIN, SensorInputChange},
		
   {ST_SAVE_PROFILE, MT_SAVE_PROFILE, 0},
   {ST_LOAD_PROFILE, MT_LOAD_PROFILE, 0},
   
   {ST_LOAD_PROFILE_DEF, MT_PROFILE_DEF, LoadProfile},
   {ST_LOAD_PROFILE_1, MT_PROFILE_1, LoadProfile},
   {ST_LOAD_PROFILE_2, MT_PROFILE_2, LoadProfile},
   {ST_LOAD_PROFILE_3, MT_PROFILE_3, LoadProfile},
   {ST_LOAD_PROFILE_4, MT_PROFILE_4, LoadProfile},
   
   
   {ST_SAVE_PROFILE_1, MT_PROFILE_1, SaveProfile},
   {ST_SAVE_PROFILE_2, MT_PROFILE_2, SaveProfile},
   {ST_SAVE_PROFILE_3, MT_PROFILE_3, SaveProfile},
   {ST_SAVE_PROFILE_4, MT_PROFILE_4, SaveProfile},   
   
   {ST_CHANGE_CHANNEL_CODE, MT_CHANGE_CHANNEL_CODE, ChangeChannelCode},
   
   {0, 0, 0}
};
  
const menu_list AnalogueChannelSetupMenu[] MENU_SPACE = {

   {ST_ANALOGUE_SETUP, ST_CHANNEL_TOGGLE, 0},
   {ST_ANALOGUE_SETUP, ST_NOTE_ADJUST, 1},
   {ST_ANALOGUE_SETUP, ST_GAIN_ADJUST, 2},
   {ST_ANALOGUE_SETUP, ST_THRESHOLD, 3},
   {ST_ANALOGUE_SETUP, ST_RETRIGGER, 4},
   {ST_ANALOGUE_SETUP, ST_DUALINPUT, 5},
   {ST_ANALOGUE_SETUP, ST_MONITORCHANNEL, 6},

   {ST_GAIN_ADJUST, ST_GAIN_TYPE, 0},
   {ST_GAIN_ADJUST, ST_GAIN_SLOPE1, 1},
   {ST_GAIN_ADJUST, ST_GAIN_SLOPE2, 2},
   {ST_GAIN_ADJUST, ST_GAIN_CROSSOVER, 3},
   {ST_GAIN_ADJUST, ST_GAIN_PRESET, 4},
   {0, 0, 0} 
};

const menu_list DigitalChannelSetupMenu[] MENU_SPACE = {
   {ST_DIGITAL_SETUP, ST_CHANNEL_TOGGLE, 0},
   {ST_DIGITAL_SETUP, ST_NOTE_ADJUST, 1},
   {ST_DIGITAL_SETUP, ST_VELOCITY_ADJUST, 2},
   {ST_DIGITAL_SETUP, ST_RETRIGGER, 3},
   {ST_DIGITAL_SETUP, ST_TRIGGER_TYPE, 4}, 
   {ST_DIGITAL_SETUP, ST_DUALINPUT, 5},

   {ST_TRIGGER_TYPE, ST_SWITCH_POLARITY, 0},
   {ST_TRIGGER_TYPE, ST_SWITCH_MODE, 1},
   {ST_TRIGGER_TYPE, ST_SWITCH_RELEASE, 2},
   {0, 0, 0} 
};

const menu_data ChannelSetupMenu[] MENU_SPACE = {
   
   {ST_ANALOGUE_SETUP,  0, HandleSubMenu},
   {ST_DIGITAL_SETUP,   0, HandleSubMenu},
   {ST_CHANNEL_TOGGLE,  0, ChannelToggleFunction},
   {ST_NOTE_ADJUST,     0, KeySelectFunction},
   {ST_GAIN_ADJUST,     0, GainAdjustFunction},
   {ST_GAIN_TYPE,       0, GainTypeAdjustFunction},
   {ST_GAIN_SLOPE1,     0, GainSlopeAdjustFunction},
   {ST_GAIN_SLOPE2,     0, GainSlopeAdjustFunction},
   {ST_GAIN_CROSSOVER,  0, GainCrossOverAdjustFunction},
   {ST_GAIN_PRESET,     0, GainPresetAdjustFunction},
   {ST_VELOCITY_ADJUST, 0,  VelocityAdjustFunction},
	{ST_THRESHOLD,       MT_THRESHOLD, SetThreshold},
	{ST_RETRIGGER, MT_RETRIGGER, SetRetrigger},   
	{ST_DUALINPUT, MT_DUALINPUT, SetDualInput},
   {ST_TRIGGER_TYPE, MT_TRIGGER_TYPE, SwitchTypeAdjustFunction},
   {ST_SWITCH_POLARITY, 0, SwitchPolarityAdjustFunction},
   {ST_SWITCH_MODE, 0, SwitchModeAdjustFunction},
   {ST_SWITCH_RELEASE, 0, SwitchReleaseAdjustFunction},

   {ST_MONITORCHANNEL, MT_MONITORCHANNEL, MonitorChannel},
   {0, 0, 0}
};  


const menu_list DualTriggerMenu[] MENU_SPACE = {
   {ST_DUAL_INPUT, ST_DUAL_MODE_TOGGLE, 0},
   {ST_DUAL_INPUT, ST_SET_OPEN_NOTE, 1},
   {ST_DUAL_INPUT, ST_SET_CLOSED_NOTE, 2},
   {ST_DUAL_INPUT, ST_SET_TRIGGER_CHANNEL, 3},
   {0, 0, 0}
};

const menu_data DualTriggerFnTable[] MENU_SPACE = {

   {ST_DUAL_INPUT,            0, HandleSubMenu},
   {ST_DUAL_MODE_TOGGLE,      0, DualTriggerToggle},
   {ST_SET_OPEN_NOTE,         0, DualTriggerNoteAdjust},
   {ST_SET_CLOSED_NOTE,       0, DualTriggerNoteAdjust},
   {ST_SET_TRIGGER_CHANNEL,   0, DualTriggerControlAdjust},
   {0, 0, 0}
};


Menu_t primaryMenu = {ST_MAIN, KP_INVALID, 0, WINDOW_SIZE, WINDOW_SIZE, 0 , 1 , &primaryMenu, 0, (menu_list*)MenuState, (menu_data*)MenuData};
Menu_t analogueMenu = {ST_ANALOGUE_SETUP, KP_INVALID, 0, WINDOW_SIZE, WINDOW_SIZE, 3 , 1 , &primaryMenu, 0,
                       (menu_list*)&AnalogueChannelSetupMenu, (menu_data*)&ChannelSetupMenu};
                       
Menu_t digitalMenu = {ST_DIGITAL_SETUP, KP_INVALID, 0, WINDOW_SIZE, WINDOW_SIZE, 3 , 1 , &primaryMenu, 0,
                       (menu_list*)&DigitalChannelSetupMenu, (menu_data*)&ChannelSetupMenu};

Menu_t dualTrigMenu = {ST_DUAL_INPUT, KP_INVALID, 0, WINDOW_SIZE, WINDOW_SIZE, 0 , 1 , &analogueMenu, 0,
                       (menu_list*)&DualTriggerMenu, (menu_data*)&DualTriggerFnTable};



Menu_t* ActiveMenu = &primaryMenu;
Menu_t* SelectedSubMenu = &primaryMenu;

void MenuSetDisplay(Menu_t* menu, uint8_t display)
{
   if(display == MENU_LCD)
   {
      menu->MenuPrint = MenuLCD_String;
      menu->MenuPrint_P = MenuLCD_String_P;
      menu->MenuNewLine = LCD_NewLine;
      menu->MenuReset = LCD_Reset;
      menu->MenuChar = MenuLCD_Char;
      menu->MenuSetPos = MenuLCD_SetPos;
   }
   /* Route output to UART */
   else
   {
      menu->MenuPrint = MenuUartTxString;
      menu->MenuPrint_P = MenuUartTxString_P;
      menu->MenuNewLine = MenuUart_NewLine;
      menu->MenuReset = MenuUart_Reset;
		menu->MenuChar = MenuUart_Tx;      
   }
}

void MenuLCD_SetPos(uint8_t row, uint8_t col)
{
   UI_LCD_Pos(&PrimaryDisplay, row, col);
}

void  MenuLCD_String(char* string)
{
   UI_LCD_String(&PrimaryDisplay, string);
}


void  MenuLCD_String_P(const char* string)
{
   UI_LCD_String_P(&PrimaryDisplay, string);
}

void MenuLCD_Char(uint8_t data)
{
   UI_LCD_Char(&PrimaryDisplay , data);
}

void LCD_NewLine(void)
{
   UI_LCD_Pos(&PrimaryDisplay, PrimaryDisplay.RowPos + 1, 0);
}

void LCD_Reset(void)
{
   UI_LCD_Clear(&PrimaryDisplay);
   UI_LCD_Pos(&PrimaryDisplay, 0, 0);
}


void MenuUart_Tx(char byte)
{
   UART_Tx(byte);
}

/* Uart Functions also route to the LCD */
void MenuUart_NewLine(void)
{
   UART_TxString_P( PSTR("\n\r") );
   LCD_NewLine();
}


void MenuUartTxString(char* string)
{
   UART_TxString(string);
   UI_LCD_String(&PrimaryDisplay, string);
}

void MenuUartTxString_P(const char* string)
{
   UART_TxString_P(string);
   UI_LCD_String_P(&PrimaryDisplay, string);
}

void MenuUart_Reset(void)
{
   uint8_t i;
 
   for( i = 0; i < 30; i++)
   {
      MenuUart_NewLine();
   }
   LCD_Reset();
}

/* Goes up one level and displays the fixed SubMenu Text */
void Menu_UpOneLevel(Menu_t* menu)
{	
   MenuSetInput(menu, KB_BACK);
   stateMachine(menu, menu->currentState);
   MenuSetInput(menu, MENU_UPDATE);   
   executeState(menu, menu->currentState); 	
   //menu->firstEnter = 1;
   ActiveMenu = menu->parentMenu;
}

void HandleSubMenu(void* data)
{
   uint8_t* input = data;

   /* This is effectively the KP Back routine which is invoked by
    * the submenu functions via MenuUpOneLevel (provides a seamless UI
    * going the up direction */
   if( *input == MENU_UPDATE )
   {
      /* Stop looping from occuring */
      MenuSetInput(ActiveMenu, KP_INVALID);
      ActiveMenu->firstEnter = 1;

      /* Show the parent options */
      MenuSetInput(ActiveMenu->parentMenu, MENU_UPDATE);
      MenuUpdate(ActiveMenu->parentMenu, RESET_MENU);

      ActiveMenu->updateOptions = HIDE_CHILDREN;
      ActiveMenu->firstEnter = 0;
      return;
   }

   Menu_t* selectedSubMenu = SelectedSubMenu;
   /* Effectively passes the KP_ENTER from the parent menu to the sub menu
      thus executing the submenu's function (seamless between primaryMenu & submenu) */
   if( *input != KP_BACK && !selectedSubMenu->firstEnter )
   {
      stateMachine(selectedSubMenu, selectedSubMenu->currentState);
      switch( *input )
      {
         case KP_ENTER:
            MenuUpdate(selectedSubMenu, !RESET_MENU);
            selectedSubMenu->firstEnter = 1;
            return;
         break;
      }
   }

   selectedSubMenu->firstEnter = 0;
}


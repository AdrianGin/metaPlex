/* Project Specific Menus */
#include <stdint.h>
#include "UART/uart.h"
#include "UserFunctions/userFunctions.h"
#include "MenuSettings.h"

#if MENU_DEBUG == 0
#include "UI_LCD/UI_LCD.h"
#include "UI_KP/UI_KP.h"
#include <string.h>
#endif


const MENU_TEXT  MT_PLAYMODE[] = "Optimise for Play!";


const MENU_TEXT  MT_PROFILES[] = "Profiles";
const MENU_TEXT  MT_LOAD_PROFILE[] = "Load Profile";
const MENU_TEXT  MT_SAVE_PROFILE[] = "Save Profile As...";

const MENU_TEXT  MT_PROFILE_1[] = "Profile 1";
const MENU_TEXT  MT_PROFILE_2[] = "Profile 2";
const MENU_TEXT  MT_PROFILE_3[] = "Profile 3";
const MENU_TEXT  MT_PROFILE_4[] = "Profile 4";
const MENU_TEXT  MT_PROFILE_DEF[] = "Default Profile";


const MENU_TEXT  MT_VUMETER[] = "VU Meter";
const MENU_TEXT  MT_DIGITAL_VUMETER[] = "Digital VU Meter";
const MENU_TEXT  MT_SYSTEM_SETUP[] = "System Setup";

const MENU_TEXT  MT_SYSTEM_RESET[] = "Reset eDrum";
const MENU_TEXT  MT_SYSTEM_ABOUT[] = "About";
const MENU_TEXT  MT_SYSTEM_SYSEX[] = "System Exclusive";
const MENU_TEXT  MT_SYSEX_DUMP[] = "Dump Current Setup";
const MENU_TEXT  MT_SYSEX_RECEIVE[] = "Get SysEx Settings";

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

const MENU_TEXT  MT_CHANNEL_1[] = "Channel 1";
const MENU_TEXT  MT_CHANNEL_2[] = "Channel 2";
const MENU_TEXT  MT_CHANNEL_3[] = "Channel 3";
const MENU_TEXT  MT_CHANNEL_4[] = "Channel 4";
const MENU_TEXT  MT_CHANNEL_5[] = "Channel 5";
const MENU_TEXT  MT_CHANNEL_6[] = "Channel 6";
const MENU_TEXT  MT_CHANNEL_7[] = "Channel 7";
const MENU_TEXT  MT_CHANNEL_8[] = "Channel 8";
const MENU_TEXT  MT_CHANNEL_9[] = "Channel 9";
const MENU_TEXT  MT_CHANNEL_10[] = "Channel 10";
const MENU_TEXT  MT_CHANNEL_11[] = "Channel 11";
const MENU_TEXT  MT_CHANNEL_12[] = "Channel 12";
const MENU_TEXT  MT_CHANNEL_13[] = "Channel 13";
const MENU_TEXT  MT_CHANNEL_14[] = "Channel 14";
const MENU_TEXT  MT_CHANNEL_15[] = "Channel 15";
const MENU_TEXT  MT_CHANNEL_16[] = "Channel 16";

const MENU_TEXT  MT_DIGITAL_1[] = "Digital Channel 1";
const MENU_TEXT  MT_DIGITAL_2[] = "Digital Channel 2";
const MENU_TEXT  MT_DIGITAL_3[] = "Digital Channel 3";
const MENU_TEXT  MT_DIGITAL_4[] = "Digital Channel 4";
const MENU_TEXT  MT_DIGITAL_5[] = "Digital Channel 5";
const MENU_TEXT  MT_DIGITAL_6[] = "Digital Channel 6";
const MENU_TEXT  MT_DIGITAL_7[] = "Digital Channel 7";
const MENU_TEXT  MT_DIGITAL_8[] = "Digital Channel 8";

const MENU_TEXT  MT_METRONOME_1[] = "Metronome Input 1";
const MENU_TEXT  MT_METRONOME_2[] = "Metronome Input 2";
const MENU_TEXT  MT_METRONOME_3[] = "Metronome Input 3";
const MENU_TEXT  MT_METRONOME_4[] = "Metronome Input 4";
const MENU_TEXT  MT_METRONOME_5[] = "Metronome Input 5";
const MENU_TEXT  MT_METRONOME_6[] = "Metronome Input 6";
const MENU_TEXT  MT_METRONOME_7[] = "Metronome Input 7";
const MENU_TEXT  MT_METRONOME_8[] = "Metronome Input 8";

const MENU_TEXT  MT_THRESHOLD[] = "Set Threshold";
const MENU_TEXT  MT_TRIGGER_TYPE[] = "Switch Settings";
const MENU_TEXT  MT_RETRIGGER[] = "Set Retrigger";
const MENU_TEXT  MT_DUALINPUT[] = "Set Dual Triggering";
const MENU_TEXT  MT_SETGAIN[] = "Set Gain Curves";

const MENU_TEXT  MT_FIXED_GAIN[] = "Fixed Gain (1x)";
const MENU_TEXT  MT_VARIABLE_GAIN[] = "Variable Gain";

/* Once a menu item is selected, it will either end in
 * 1) Another Menu or
 * 2) A function
 * */
const menu_list MenuState[] = {

	{ST_MAIN,	ST_PLAYMODE,  0},
   {ST_MAIN,   ST_OPTIONS,   1},
   {ST_MAIN,	ST_VUMETER,	  2},
   {ST_MAIN,   ST_DIGITAL_VUMETER, 3},
   {ST_MAIN, 	ST_PROFILES,  4},
   {ST_MAIN,   ST_SYSTEM_SETUP, 5},

   {ST_OPTIONS, ST_MIDI_OUTPUT_RATE,  0},
   {ST_OPTIONS, ST_CHANNEL_SETUP, 1},
   {ST_OPTIONS, ST_CHANGE_CHANNEL_CODE, 2},
   {ST_OPTIONS, ST_INPUT_SELECT, 3},
   {ST_OPTIONS, ST_CROSSTALK, 4},
	
	{ST_INPUT_SELECT, ST_FIXED_GAIN, 2},
	{ST_INPUT_SELECT, ST_VARIABLE_GAIN, 3},		
		
   {ST_MIDI_OUTPUT_RATE, ST_SET_RATE, 1},

   {ST_CHANNEL_SETUP, ST_ANALOGUE_INPUTS,  0},
   {ST_CHANNEL_SETUP, ST_DIGITAL_INPUTS,  1}, 
   {ST_CHANNEL_SETUP, ST_METRONOME_INPUTS, 2},  

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
		   
	{ST_CHANNEL_1, ST_THRESHOLD_1, 3},
	{ST_CHANNEL_2, ST_THRESHOLD_2, 3},
	{ST_CHANNEL_3, ST_THRESHOLD_3, 3},
	{ST_CHANNEL_4, ST_THRESHOLD_4, 3},
	{ST_CHANNEL_5, ST_THRESHOLD_5, 3},
	{ST_CHANNEL_6, ST_THRESHOLD_6, 3},
	{ST_CHANNEL_7, ST_THRESHOLD_7, 3},
	{ST_CHANNEL_8, ST_THRESHOLD_8, 3},
	{ST_CHANNEL_9, ST_THRESHOLD_9, 3},
	{ST_CHANNEL_10, ST_THRESHOLD_10, 3},
	{ST_CHANNEL_11, ST_THRESHOLD_11, 3},
	{ST_CHANNEL_12, ST_THRESHOLD_12, 3},
	{ST_CHANNEL_13, ST_THRESHOLD_13, 3},
	{ST_CHANNEL_14, ST_THRESHOLD_14, 3},
	{ST_CHANNEL_15, ST_THRESHOLD_15, 3},
	{ST_CHANNEL_16, ST_THRESHOLD_16, 3},

	/* Analogue Retrigger Menu's */	
	{ST_CHANNEL_1, ST_RETRIGGER_1, 4},
	{ST_CHANNEL_2, ST_RETRIGGER_2, 4},
	{ST_CHANNEL_3, ST_RETRIGGER_3, 4},
	{ST_CHANNEL_4, ST_RETRIGGER_4, 4},
	{ST_CHANNEL_5, ST_RETRIGGER_5, 4},
	{ST_CHANNEL_6, ST_RETRIGGER_6, 4},
	{ST_CHANNEL_7, ST_RETRIGGER_7, 4},
	{ST_CHANNEL_8, ST_RETRIGGER_8, 4},
	{ST_CHANNEL_9, ST_RETRIGGER_9, 4},
	{ST_CHANNEL_10, ST_RETRIGGER_10, 4},
	{ST_CHANNEL_11, ST_RETRIGGER_11, 4},
	{ST_CHANNEL_12, ST_RETRIGGER_12, 4},
	{ST_CHANNEL_13, ST_RETRIGGER_13, 4},
	{ST_CHANNEL_14, ST_RETRIGGER_14, 4},
	{ST_CHANNEL_15, ST_RETRIGGER_15, 4},
	{ST_CHANNEL_16, ST_RETRIGGER_16, 4},
	
	/* Gain Curve Screens */
	{ST_CHANNEL_1, ST_SETGAIN_1, 5},
	{ST_CHANNEL_2, ST_SETGAIN_2, 5},
	{ST_CHANNEL_3, ST_SETGAIN_3, 5},
	{ST_CHANNEL_4, ST_SETGAIN_4, 5},
	{ST_CHANNEL_5, ST_SETGAIN_5, 5},
	{ST_CHANNEL_6, ST_SETGAIN_6, 5},
	{ST_CHANNEL_7, ST_SETGAIN_7, 5},
	{ST_CHANNEL_8, ST_SETGAIN_8, 5},
	{ST_CHANNEL_9, ST_SETGAIN_9, 5},
	{ST_CHANNEL_10, ST_SETGAIN_10, 5},
	{ST_CHANNEL_11, ST_SETGAIN_11, 5},
	{ST_CHANNEL_12, ST_SETGAIN_12, 5},
	{ST_CHANNEL_13, ST_SETGAIN_13, 5},
	{ST_CHANNEL_14, ST_SETGAIN_14, 5},
	{ST_CHANNEL_15, ST_SETGAIN_15, 5},
	{ST_CHANNEL_16, ST_SETGAIN_16, 5},	
	
	/* Analgoue Dual Input Menu's */
	{ST_CHANNEL_1, ST_DUALINPUT_1, 6},
	{ST_CHANNEL_2, ST_DUALINPUT_2, 6},
	{ST_CHANNEL_3, ST_DUALINPUT_3, 6},
	{ST_CHANNEL_4, ST_DUALINPUT_4, 6},
	{ST_CHANNEL_5, ST_DUALINPUT_5, 6},
	{ST_CHANNEL_6, ST_DUALINPUT_6, 6},
	{ST_CHANNEL_7, ST_DUALINPUT_7, 6},
	{ST_CHANNEL_8, ST_DUALINPUT_8, 6},
	{ST_CHANNEL_9, ST_DUALINPUT_9, 6},
	{ST_CHANNEL_10, ST_DUALINPUT_10, 6},
	{ST_CHANNEL_11, ST_DUALINPUT_11, 6},
	{ST_CHANNEL_12, ST_DUALINPUT_12, 6},
	{ST_CHANNEL_13, ST_DUALINPUT_13, 6},
	{ST_CHANNEL_14, ST_DUALINPUT_14, 6},
	{ST_CHANNEL_15, ST_DUALINPUT_15, 6},
	{ST_CHANNEL_16, ST_DUALINPUT_16, 6},


	/* Digital Config Menu's */
   {ST_DIGITAL_INPUTS, ST_DIGITAL_1,  0},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_2,  1},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_3,  2},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_4,  3},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_5,  4},            
   {ST_DIGITAL_INPUTS, ST_DIGITAL_6,  5},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_7,  6},
   {ST_DIGITAL_INPUTS, ST_DIGITAL_8,  7},

	{ST_DIGITAL_1,ST_TRIGGER_TYPE_D1, 3},
	{ST_DIGITAL_2,ST_TRIGGER_TYPE_D2, 3},
	{ST_DIGITAL_3,ST_TRIGGER_TYPE_D3, 3},
	{ST_DIGITAL_4,ST_TRIGGER_TYPE_D4, 3},
	{ST_DIGITAL_5,ST_TRIGGER_TYPE_D5, 3},
	{ST_DIGITAL_6,ST_TRIGGER_TYPE_D6, 3},
	{ST_DIGITAL_7,ST_TRIGGER_TYPE_D7, 3},
	{ST_DIGITAL_8,ST_TRIGGER_TYPE_D8, 3},

	/* Digital Retrigger Menu's */
	{ST_DIGITAL_1,ST_RETRIGGER_D1, 4},
	{ST_DIGITAL_2,ST_RETRIGGER_D2, 4},
	{ST_DIGITAL_3,ST_RETRIGGER_D3, 4},
	{ST_DIGITAL_4,ST_RETRIGGER_D4, 4},
	{ST_DIGITAL_5,ST_RETRIGGER_D5, 4},
	{ST_DIGITAL_6,ST_RETRIGGER_D6, 4},
	{ST_DIGITAL_7,ST_RETRIGGER_D7, 4},
	{ST_DIGITAL_8,ST_RETRIGGER_D8, 4},

   {ST_METRONOME_INPUTS, ST_METRONOME_1, 0},
   {ST_METRONOME_INPUTS, ST_METRONOME_2, 1},
   {ST_METRONOME_INPUTS, ST_METRONOME_3, 2},
   {ST_METRONOME_INPUTS, ST_METRONOME_4, 3},
   {ST_METRONOME_INPUTS, ST_METRONOME_5, 4},
   {ST_METRONOME_INPUTS, ST_METRONOME_6, 5},
   {ST_METRONOME_INPUTS, ST_METRONOME_7, 6},                  
   {ST_METRONOME_INPUTS, ST_METRONOME_8, 7},
   
	/* Metronome (Digital) Retrigger Menu's */
	{ST_METRONOME_1, ST_RETRIGGER_M1, 3},
	{ST_METRONOME_2, ST_RETRIGGER_M2, 3},
	{ST_METRONOME_3, ST_RETRIGGER_M3, 3},
	{ST_METRONOME_4, ST_RETRIGGER_M4, 3},
	{ST_METRONOME_5, ST_RETRIGGER_M5, 3},
	{ST_METRONOME_6, ST_RETRIGGER_M6, 3},
	{ST_METRONOME_7, ST_RETRIGGER_M7, 3},
	{ST_METRONOME_8, ST_RETRIGGER_M8, 3},   
   
   {ST_PROFILES, ST_LOAD_PROFILE, 2},
   {ST_PROFILES, ST_SAVE_PROFILE, 3},
		
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_1, 0},	   
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_2, 1},
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_3, 2},
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_4, 3},
	{ST_LOAD_PROFILE, ST_LOAD_PROFILE_DEF, 4},
	
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_1, 0},	   
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_2, 1},
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_3, 2},
	{ST_SAVE_PROFILE, ST_SAVE_PROFILE_4, 3},
		
	{ST_SYSTEM_SETUP, ST_SYSTEM_RESET, 0},
	{ST_SYSTEM_SETUP, ST_SYSTEM_ABOUT, 1},
	{ST_SYSTEM_SETUP, ST_SYSEX, 2},
	
	{ST_SYSEX, ST_SYSEX_DUMP, 0},
	{ST_SYSEX, ST_SYSEX_RECEIVE, 1},
			   
   {0, 0, 0}
};





const menu_data MenuData[] = {
   {ST_MAIN, 0, 0},
   {ST_PLAYMODE, MT_PLAYMODE, PlayMode},
   {ST_PROFILES, MT_PROFILES, ShowProfile},   
   {ST_OPTIONS, MT_OPTIONS, 0},
	{ST_VUMETER, MT_VUMETER, VUMeterSetup},
	{ST_DIGITAL_VUMETER, MT_DIGITAL_VUMETER, VUMeterSetup},
	{ST_SYSTEM_SETUP, MT_SYSTEM_SETUP, 0},
	{ST_SYSTEM_RESET, MT_SYSTEM_RESET, reset},
	{ST_SYSTEM_ABOUT, MT_SYSTEM_ABOUT, about},
	{ST_SYSEX, MT_SYSTEM_SYSEX, 0},
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
   
   /* Analogue Threshold Menu */
	{ST_THRESHOLD_1, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_2, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_3, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_4, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_5, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_6, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_7, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_8, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_9, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_10, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_11, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_12, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_13, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_14, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_15, MT_THRESHOLD, SetThreshold},
	{ST_THRESHOLD_16, MT_THRESHOLD, SetThreshold},
	
	/* Analogue Retrigger Menus */
	{ST_RETRIGGER_1, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_2, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_3, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_4, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_5, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_6, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_7, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_8, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_9, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_10, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_11, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_12, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_13, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_14, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_15, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_16, MT_RETRIGGER, SetRetrigger},

	/* Gain Set Screens */
	{ST_SETGAIN_1, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_2, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_3, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_4, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_5, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_6, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_7, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_8, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_9, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_10, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_11, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_12, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_13, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_14, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_15, MT_SETGAIN, SetGainCurves},
	{ST_SETGAIN_16, MT_SETGAIN, SetGainCurves},
	
	/* Analogue Dual Input Menu */
	{ST_DUALINPUT_1, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_2, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_3, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_4, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_5, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_6, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_7, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_8, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_9, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_10, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_11, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_12, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_13, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_14, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_15, MT_DUALINPUT, SetDualInput},
	{ST_DUALINPUT_16, MT_DUALINPUT, SetDualInput},

	
	/* Digital Menus */
   {ST_DIGITAL_1, MT_DIGITAL_1,   DigitalChannelSettings},
   {ST_DIGITAL_2, MT_DIGITAL_2,   DigitalChannelSettings},
   {ST_DIGITAL_3, MT_DIGITAL_3,   DigitalChannelSettings},
   {ST_DIGITAL_4, MT_DIGITAL_4,   DigitalChannelSettings},
   {ST_DIGITAL_5, MT_DIGITAL_5,   DigitalChannelSettings},            
   {ST_DIGITAL_6, MT_DIGITAL_6,   DigitalChannelSettings},
   {ST_DIGITAL_7, MT_DIGITAL_7,   DigitalChannelSettings},
   {ST_DIGITAL_8, MT_DIGITAL_8,   DigitalChannelSettings},

	/* Digital Switch type setup */
	{ST_TRIGGER_TYPE_D1, MT_TRIGGER_TYPE, SetSwitchType},
	{ST_TRIGGER_TYPE_D2, MT_TRIGGER_TYPE, SetSwitchType},
	{ST_TRIGGER_TYPE_D3, MT_TRIGGER_TYPE, SetSwitchType},
	{ST_TRIGGER_TYPE_D4, MT_TRIGGER_TYPE, SetSwitchType},
	{ST_TRIGGER_TYPE_D5, MT_TRIGGER_TYPE, SetSwitchType},
	{ST_TRIGGER_TYPE_D6, MT_TRIGGER_TYPE, SetSwitchType},
	{ST_TRIGGER_TYPE_D7, MT_TRIGGER_TYPE, SetSwitchType},
	{ST_TRIGGER_TYPE_D8, MT_TRIGGER_TYPE, SetSwitchType},
		
	/* Digital Retriggers */
	{ST_RETRIGGER_D1, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_D2, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_D3, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_D4, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_D5, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_D6, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_D7, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_D8, MT_RETRIGGER, SetRetrigger},

	/* Digital Menus */
   {ST_METRONOME_1, MT_METRONOME_1,   DigitalChannelSettings},
   {ST_METRONOME_2, MT_METRONOME_2,   DigitalChannelSettings},
   {ST_METRONOME_3, MT_METRONOME_3,   DigitalChannelSettings},
   {ST_METRONOME_4, MT_METRONOME_4,   DigitalChannelSettings},
   {ST_METRONOME_5, MT_METRONOME_5,   DigitalChannelSettings},            
   {ST_METRONOME_6, MT_METRONOME_6,   DigitalChannelSettings},
   {ST_METRONOME_7, MT_METRONOME_7,   DigitalChannelSettings},
   {ST_METRONOME_8, MT_METRONOME_8,   DigitalChannelSettings},

	/* Digital Retriggers */
	{ST_RETRIGGER_M1, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_M2, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_M3, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_M4, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_M5, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_M6, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_M7, MT_RETRIGGER, SetRetrigger},
	{ST_RETRIGGER_M8, MT_RETRIGGER, SetRetrigger},


	{ST_FIXED_GAIN, MT_FIXED_GAIN, SensorInputChange},
	{ST_VARIABLE_GAIN, MT_VARIABLE_GAIN, SensorInputChange},
		
   {ST_SAVE_PROFILE, MT_SAVE_PROFILE, 0},
   {ST_LOAD_PROFILE, MT_LOAD_PROFILE, 0},
   
   {ST_LOAD_PROFILE_1, MT_PROFILE_1, LoadProfile},
   {ST_LOAD_PROFILE_2, MT_PROFILE_2, LoadProfile},
   {ST_LOAD_PROFILE_3, MT_PROFILE_3, LoadProfile},
   {ST_LOAD_PROFILE_4, MT_PROFILE_4, LoadProfile},
   {ST_LOAD_PROFILE_DEF, MT_PROFILE_DEF, LoadProfile},			   
   
   {ST_SAVE_PROFILE_1, MT_PROFILE_1, SaveProfile},
   {ST_SAVE_PROFILE_2, MT_PROFILE_2, SaveProfile},
   {ST_SAVE_PROFILE_3, MT_PROFILE_3, SaveProfile},
   {ST_SAVE_PROFILE_4, MT_PROFILE_4, SaveProfile},   
   
   {ST_CHANGE_CHANNEL_CODE, MT_CHANGE_CHANNEL_CODE, ChangeChannelCode},
   
   {0, 0, 0}
};

//   uint8_t currentState = ST_MAIN;
//   uint8_t RowPosition = 0;
//   static uint8_t MenuInput;
//   uint8_t selectedItem = 0;
//   uint8_t upperLimit = WINDOW_SIZE;
//   uint8_t lowerLimit = 0;
//   uint8_t firstEnter = 1;   

Menu_t primaryMenu = {ST_MAIN, 0, 0, 0, WINDOW_SIZE, WINDOW_SIZE, 0 , 1 };

void MenuSetDisplay(uint8_t display)
{
   if(display == MENU_LCD)
   {
      primaryMenu->MenuPrint = MenuLCD_String;
      primaryMenu->MenuPrint_P = MenuLCD_String_P;
      primaryMenu->MenuNewLine = LCD_NewLine;
      primaryMenu->MenuReset = LCD_Reset;
      primaryMenu->MenuChar = MenuLCD_Char;
      primaryMenu->MenuSetPos = MenuLCD_SetPos;
   }
   /* Route output to UART */
   else
   {
      primaryMenu->MenuPrint = MenuUartTxString;
      primaryMenu->MenuPrint_P = MenuUartTxString_P;
      primaryMenu->MenuNewLine = MenuUart_NewLine;
      primaryMenu->MenuReset = MenuUart_Reset;
		primaryMenu->MenuChar = UART_Tx;      
   }
}

void MenuLCD_SetPos(uint8_t row, uint8_t col)
{
   UI_LCD_Pos(PrimaryDisplay, row, col);
}

void  MenuLCD_String(char* string)
{
   UI_LCD_String(PrimaryDisplay, string);
}


void  MenuLCD_String_P(const char* string)
{
   UI_LCD_String_P(PrimaryDisplay, string);
}

void MenuLCD_Char(char data)
{
   UI_LCD_Char(PrimaryDisplay , data);
}


void LCD_NewLine(void)
{
   UI_LCD_Pos(PrimaryDisplay, ++RowPosition, 0);
}

void LCD_Reset(void)
{
   UI_LCD_Clear(PrimaryDisplay);
   UI_LCD_Pos(PrimaryDisplay, 0, 0);
   RowPosition = 0;
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
   UI_LCD_String(string);
}

void MenuUartTxString_P(const char* string)
{
   UART_TxString_P(string);
   UI_LCD_String_P(string);
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


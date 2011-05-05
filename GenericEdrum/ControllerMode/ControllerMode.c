/* Has the Controller Mode handler functions */

#include <stdint.h>
#include "UI_KP/UI_KP.h"
#include "ControllerMode.h"
#include "MIDI/midi.h"
#include "RingBuffer/ringbuffer.h"
#include "UserFunctions/userFunctions.h"
#include "Sample/sample.h"
#include "mmculib/uint8toa.h"

#include "hardwareSpecific.h"

static uint8_t inputMode = 0;
static uint8_t lastChannel = CM_MIN_CHANNEL;
static uint8_t lastMIDICode = CM_MIN_MIDI_CODE;
static uint8_t lastMIDIKey = 0;
static uint8_t menuMode = CM_SETTINGS_MODE;

static char outputString[21];

const INPUT_TEXT IT_CHANNEL[] = "Channel ";
const INPUT_TEXT IT_CODE[] = "MIDI Message: ";
const INPUT_TEXT IT_KEY[] = "MIDI Key/CC: ";
const INPUT_TEXT IT_DATA[] = "MIDI Data: ";

const CM_InputType_t InputModeData[] = {

	{CM_CHANNEL_SELECT, IT_CHANNEL, CM_ChannelParse},
	{CM_MIDI_CODE_CHANGE, IT_CODE, CM_SetMIDICode},
	{CM_MIDI_KEY_CHANGE, IT_KEY, CM_SetMIDIKey},
	{CM_SEND_MIDI_MESSAGE, IT_DATA, CM_ChannelParse},
				
};


void CM_ChannelParse(void* data)
{

/*	uint8_t* input;
	static uint8_t lastInput = 0;
	
	input = (uint8_t*)data;
	
	if( *input == CM_PRINT_STATUS)
	{
		UF_MenuPrint_P( IT_CHANNEL );
		printChannelInfo();
		return;
	}
	
	if( *input == CM_ACCEPT_DATA )
	{
		lastInput = 0;
		*input = 0;
		return;
	}*/
	
	/* Update the channel display as it is entered */
/*	if( (*input != 0) && (lastInput == 0) )
	{
		lastChannel = *input;
		lastInput = *input;
	}
	else
	{
		lastChannel = (lastInput * 10) + (*input);
		lastInput = 0;
	}
	
	if( lastChannel > NUMBER_OF_REAL_INPUTS || lastChannel == 0 )
	{
		lastChannel = CM_MIN_CHANNEL;
		lastInput = 0;
	}*/
}

void printChannelInfo(void)
{
/*	uint8_t printChannel;
	printChannel = lastChannel;
	if( lastChannel > ANALOGUE_INPUTS)
	{
		UF_MenuPrint_P(PSTR("(D)"));
		printChannel = lastChannel-ANALOGUE_INPUTS;
	}
	uint8toa(printChannel, outputString);
	UF_MenuPrint(outputString);
	UF_MenuNewLine();*/
}


void CM_SetChannel(uint8_t channel)
{
	if( channel < NUMBER_OF_REAL_INPUTS )
	{
		lastChannel = channel;
	}
	
}



void CM_SetMIDICode(void* data)
{
	uint8_t* input;
	
	input = (uint8_t*)data;

	if( *input == CM_PRINT_STATUS)
	{
		UF_MenuPrint_P( IT_CODE );
		uint8toa(lastMIDICode >> 4, outputString);
		UF_MenuPrint(outputString);
		UF_MenuNewLine();
		return;
	}
	
	if( *input == CM_ACCEPT_DATA )
	{
		return;
	}
	
	/* Update the channel display as it is entered */
	lastMIDICode = (*input) + MIDI_NOTE_OFF - 1;
	
	
	if( lastMIDICode > MIDI_PITCH_CHANGE )
	{
		lastMIDICode = MIDI_NOTE_OFF;
	}
	if( lastMIDICode < MIDI_NOTE_OFF )
	{
		lastMIDICode = MIDI_NOTE_OFF;
	}
	
	SetChannelCommand(lastChannel-1, lastMIDICode);
}



void CM_SetMIDIKey(void* data)
{
	uint8_t* input;
	
	input = (uint8_t*)data;
	

	if( *input == CM_PRINT_STATUS)
	{
		UF_MenuPrint_P( IT_KEY );
		
		if( lastMIDICode == MIDI_NOTE_ON )
		{
			MIDI_NoteString(lastMIDIKey, outputString);
			UF_MenuPrint(outputString);
			uint8toa( MIDI_Octave(lastMIDIKey), outputString);
		}
		else
		{
			uint8toa(lastMIDIKey, outputString);
		}
		UF_MenuPrint(outputString);
		UF_MenuNewLine();
		return;
	}
	
	if( *input == CM_ACCEPT_DATA )
	{
		lastMIDIKey = 0;
		return;
	}

	/* Update the channel display as it is entered */
	lastMIDIKey = (lastMIDIKey * 10) + (*input);
	
	
	if( lastMIDIKey > MIDI_MAX_KEY )
	{
		lastMIDIKey = 0;
	}
	
	SetChannelKey(lastChannel-1, lastMIDIKey);
}


/* Changes the input type */
void CM_InputSelect(uint8_t inputCode)
{
	if(inputCode < CM_INPUT_TYPE_COUNT)
	{
		inputMode = inputCode;
	}
	
	return;
}

/* for list of inputKeys refer to UI_KP.c and KP_ButtonIndex */
/* inputKey must be between 0 and 15 inclusive */
void CM_ReceiveInput(uint8_t inputKey)
{
	
	if( inputKey > CM_MAX_INPUT)
	{
		return;	
	}
	
	if((inputKey < 10) || (inputKey == CM_ACCEPT_DATA))
	{
		void (*funcPtr)(void* data);
	   funcPtr = (void*)InputModeData[inputMode].function;
      funcPtr(&inputKey);
	}

	
	if(inputKey > 9)
	{
		CM_InputSelect(inputKey-CM_CHANNEL_CHANGE);
		return;	
	}
	
	CM_printEnteredData();

}


void CM_printEnteredData(void)
{
	uint8_t inputKey;
	
	inputKey = CM_PRINT_STATUS;
	UF_MenuReset();
	CM_ChannelParse(&inputKey);
	
	lastMIDICode = GetChannelCommand(lastChannel-1);
	lastMIDIKey = GetChannelKey(lastChannel-1);
	
	CM_SetMIDICode(&inputKey);
	CM_SetMIDIKey(&inputKey);
	
}


uint8_t CM_GetMenuMode(void)
{
	return menuMode;
}

void CM_SetMenuMode(uint8_t mode)
{
	if( menuMode <= CM_MENU_MODE)
	{
		menuMode = mode; 
	}
}


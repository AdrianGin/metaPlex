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

#include <stdint.h>
#include <string.h>
#include "hardwareSpecific.h"
#include "Sample/sample.h"
#include "Sample/digitalSample.h"
#include "midi.h"
#include "VUMeter/vumeter.h"

#include "TimerCallbacks/TimerCallbacks.h"
#include "edrumAVRsharedfunctions.h"


const char MIDI_NOTES[] = "C C#D D#E F F#G G#A A#B ";
const PROGRAM_CHAR MIDI_BAUD[][11] PROGRAM_SPACE = {"31.25k",
									                         "38.4k",
									                         "115.2kUSB",
									                         "1.0M USB"};


PROGRAM_CHAR MIDI_NOTE_OFF_STRING[] PROGRAM_SPACE = "NOTE OFF";
PROGRAM_CHAR MIDI_NOTE_ON_STRING[] PROGRAM_SPACE = "NOTE ON";
PROGRAM_CHAR MIDI_POLY_STRING[] PROGRAM_SPACE = "AFTRTCH";
PROGRAM_CHAR MIDI_CONTROL_CHANGE_STRING[] PROGRAM_SPACE = "CONT.CHNG";
PROGRAM_CHAR MIDI_PROGRAM_CHANGE_STRING[] PROGRAM_SPACE = "PROG.CHNG";
PROGRAM_CHAR MIDI_CHANNEL_PRESSURE_STRING[] PROGRAM_SPACE= "CHAN.PRES";
PROGRAM_CHAR MIDI_PITCH_WHEEL_STRING[] PROGRAM_SPACE = "PITCH W.";
PROGRAM_CHAR MIDI_SYSEX_STRING[] PROGRAM_SPACE = "SYSEX";

PROGRAM_CHAR MIDI_TIME_CODE_STRING[] PROGRAM_SPACE = "TIMECODE";
PROGRAM_CHAR MIDI_SONG_POS_STRING[] PROGRAM_SPACE = "SONG.POS";
PROGRAM_CHAR MIDI_SONG_SEL_STRING[] PROGRAM_SPACE = "SONG.SEL";

PROGRAM_CHAR MIDI_TUNE_REQUEST_STRING[] PROGRAM_SPACE = "TUNE REQ";
PROGRAM_CHAR MIDI_END_SYSEX_STRING[] PROGRAM_SPACE = "END SYSX";

PROGRAM_CHAR MIDI_TIMING_CLOCK_STRING[] PROGRAM_SPACE = "CLOCK";
PROGRAM_CHAR MIDI_SEQ_START_STRING[] PROGRAM_SPACE = "START";
PROGRAM_CHAR MIDI_SEQ_CONTINUE_STRING[] PROGRAM_SPACE = "CONTIN.";
PROGRAM_CHAR MIDI_SEQ_STOP_STRING[] PROGRAM_SPACE = "STOP";

PROGRAM_CHAR MIDI_ACTIVE_SENSING_STRING[] PROGRAM_SPACE = "ACT.SENS";
PROGRAM_CHAR MIDI_RESET_STRING[] PROGRAM_SPACE = "RESET";

PROGRAM_CHAR MIDI_UNDEFINED_STRING[] PROGRAM_SPACE = "UNDEF";

static uint16_t MIDI_LastMIDIValue[ANALOGUE_INPUTS];

const MidiLookup_t MidiLookUpTable[] PROGRAM_SPACE =
{
      {MIDI_NOTE_ON, MIDI_NOTE_ON_STRING,                   3},
      {MIDI_NOTE_OFF, MIDI_NOTE_OFF_STRING,                 3},
      {MIDI_AFTERTOUCH, MIDI_POLY_STRING,                   3},
      {MIDI_CONTROL_CHANGE, MIDI_CONTROL_CHANGE_STRING,     3},
      {MIDI_PROGRAM_CHANGE, MIDI_PROGRAM_CHANGE_STRING,     2},
      {MIDI_CHANNEL_PRESSURE, MIDI_CHANNEL_PRESSURE_STRING, 2},
      {MIDI_PITCH_CHANGE, MIDI_PITCH_WHEEL_STRING,          3},
      {MIDI_SYSEX_START, MIDI_SYSEX_STRING,                 3},
      {MIDI_TIME_CODE,MIDI_TIME_CODE_STRING,                2},
      {MIDI_SONG_POSITION, MIDI_SONG_POS_STRING,            3},
      {MIDI_SONG_SELECT, MIDI_SONG_SEL_STRING,              2},
      {MIDI_TUNE_REQUEST, MIDI_TUNE_REQUEST_STRING,         1},
      {MIDI_SYSEX_STOP, MIDI_END_SYSEX_STRING,              1},
      {MIDI_TIMING_CLOCK, MIDI_TIMING_CLOCK_STRING,         1},
      {MIDI_RT_TICK, MIDI_UNDEFINED_STRING,                 1},
      {MIDI_RT_START, MIDI_SEQ_START_STRING,                1},
      {MIDI_RT_CONTINUE, MIDI_SEQ_CONTINUE_STRING,          1},
      {MIDI_RT_STOP, MIDI_SEQ_STOP_STRING,                  1},
      {MIDI_RT_ACTIVE_SENSE, MIDI_ACTIVE_SENSING_STRING,    1},
      {MIDI_RT_RESET, MIDI_RESET_STRING,                    1},
      {0, 0}
};

MidiSettings_t* MIDISettings;

/* This checks to format the messages, ie, ChannelCode has no effect
 * if StatusCodes > MIDI_PITCH_CHANGE, number of bytes to send also depends on
 * the StatusCode, see MidiLookUpTable[] */
void MIDI_SendMsg(MIDI_MSG_t* msg)
{
   uint8_t StatusCode = msg->StatusCode;
   uint8_t bytesToSend = MIDI_CommandSize(StatusCode);
   if(StatusCode <= MIDI_PITCH_CHANGE)
   {
      StatusCode |= MIDISettings->MIDI_ChannelCode;;
   }
   midiTx(StatusCode);
   bytesToSend--;

   if( bytesToSend-- )
   {
      midiTx(msg->Data[0]);
      if( bytesToSend )
      {
         midiTx(msg->Data[1]);
      }
   }

}

/* This is a RAW send */
void midiTx(uint8_t inbyte)
{
   USBMIDI_PutByte(inbyte, 0);
   UART_Tx(inbyte);
}


void MIDI_OutputAnalogueChannel(uint8_t channel)
{
   MIDI_MSG_t msg;
   if( GetChannelStatus(channel) && 
       (RetriggerPeriod[channel].timerEnable == SOFTTIMER_DISABLED) && 
       (SignalPeak[channel]) )
   {
   
   	//uint16_t conditionedSignal = (SignalPeak[i] - GetChannelThresh(i));
      /* Make the conditioned signal start at the Threshold */
      uint16_t conditionedSignal = (SignalPeak[channel]);			
   	conditionedSignal = GainFunction(channel, conditionedSignal);
   	if( conditionedSignal )
   	{
         /* Make it slightly larger, so we can reach 0x7F */
         conditionedSignal++;

         /* Output the correct Closed or Open Key */
         if( GetDualMode(channel) && GetChannelState(GetTrigger(channel)) )
   		{
            /* The channel code is appended at the midi_send function */
            msg.StatusCode = GetClosedChannelCommand(channel);
            msg.Data[0] = GetChannelKeyClosed(channel);
   		}
   		else
   		{
   		   msg.StatusCode = GetChannelCommand(channel);
   		   msg.Data[0] = GetChannelKey(channel);
   		}
      
   		if( conditionedSignal > MIDI_MAX_DATA )
         {
   			conditionedSignal = MIDI_MAX_DATA;
         }

   		msg.Data[1] = (uint8_t)(conditionedSignal);
   		MIDI_SendMsg(&msg);

         /* Auxiliary MIDI functions */
         MIDI_LastMIDIValue[channel] = conditionedSignal;

   		if( SoftTimer2[SC_VUMeterUpdate].timerEnable )
         {
            if( SignalPeak[channel] > VUValues[channel] )
         	{
               VUValues[channel] = SignalPeak[channel];
            }
   		}

   		SoftTimerStart(RetriggerPeriod[channel]);
   	}

   }  
}


void MIDI_Output(void)
{
   uint8_t i;
   /* do analogue inputs then do digital inputs  */
   for( i = 0; i < ANALOGUE_INPUTS; i++)
   {      
      MIDI_OutputAnalogueChannel(i);
   }  
}


void MIDI_DigitalOutput(void)
{
   uint8_t i;
   MIDI_MSG_t msg;
   uint8_t selectedDigitalInput;

   for( i = ANALOGUE_INPUTS; i < NUMBER_OF_INPUTS; i++)
   {      
      selectedDigitalInput = i - ANALOGUE_INPUTS;
      if( GetChannelStatus(i) && 
          (RetriggerPeriod[i].timerEnable == SOFTTIMER_DISABLED))
      {
         /* If the channel selected in a MetroNome one, automatically
          * trigger a signal.
          */
   		if( SignalPeak[i] || (i >= ANALOGUE_INPUTS + DIGITAL_INPUTS))
   		{	
	         /* Send a NOTE ON (default) | Channel */
   		   if( GetDualMode(i) && GetChannelState(GetTrigger(i)) )
   		   {
   		      msg.StatusCode = GetClosedChannelCommand(i);
   		      msg.Data[0] = GetChannelKeyClosed(i);
   		   }
   		   else
   		   {
               msg.StatusCode = GetChannelCommand(i);
               msg.Data[0] = GetChannelKey(i);
               /* If the channel is a metronome input, do not check
                * to see if it has an active release */
               if((i >= ANALOGUE_INPUTS + DIGITAL_INPUTS) )
               {
               }
               else
               {
                  if( GetActiveRelease(selectedDigitalInput) && SignalPeak[i] == INPUT_IS_RELEASED )
                  {
                      msg.StatusCode = GetClosedChannelCommand(i);
                      msg.Data[0] = GetChannelKeyClosed(i);
                  }
               }
   		   }
   		   msg.Data[1] = GetDigitalVelocity(selectedDigitalInput);

   		   MIDI_SendMsg(&msg);

				if( SoftTimer2[SC_DigitalVUUpdate].timerEnable )
	         {
	         	VUValues[selectedDigitalInput] = GetDigitalVelocity(selectedDigitalInput);
				}

				SoftTimerStart(RetriggerPeriod[i]);
			}
      }
   }
}


#if CONTROLLER_MODE
void MIDI_KeypadOutput(uint8_t kpButton)
{
   uint8_t i;
   if( kpButton < KEYPAD_INPUTS )
   {
	   i = kpButton + ANALOGUE_INPUTS + DIGITAL_INPUTS + METRONOME_INPUTS;
	   
		/* Send a NOTE ON (default) | Channel */
	   MIDI_Tx((GetChannelCommand(i)) | MIDISettings->MIDI_ChannelCode);
	   MIDI_Tx(GetChannelKey(i));
	   MIDI_Tx( GetDigitalVelocity(i - ANALOGUE_INPUTS) );
			
				/*if( SoftTimer2[SC_DigitalVUUpdate].timerEnable )
		      {
		         VUValues[i-ANALOGUE_INPUTS] = GetDigitalVelocity(i - ANALOGUE_INPUTS);
				}*/
	}
}
#endif


void MIDI_SetBaud(uint16_t newBaud)
{

   MIDISettings->MIDI_BaudRate = newBaud; 
   MIDI_SetBaudRate( newBaud >> 8, newBaud & 0xFF );
}


uint16_t MIDI_GetBaud(void)
{
   return MIDISettings->MIDI_BaudRate;
}

uint8_t MIDI_GetChannelCode(void)
{
	return (MIDISettings->MIDI_ChannelCode);
}

void MIDI_SetChannelCode(uint8_t newCode)
{
   if( newCode <= 0x0F )
   {
		MIDISettings->MIDI_ChannelCode = newCode;
   }
}

/* An invalid command code will return NOTE_OFF control code position*/
uint8_t MIDI_GetControlCode(uint8_t command, uint8_t direction)
{
   uint8_t i;
   uint8_t returnCode = 0x70;
   for(i = 0; FLASH_GET_PGM_BYTE(&MidiLookUpTable[i].MIDI_Commands); i++)
   {
      if( command == FLASH_GET_PGM_BYTE(&MidiLookUpTable[i].MIDI_Commands) )
      {
         returnCode = i;
      }
   }
   if(returnCode != 0x70 )
   {
      if( direction == MIDI_NEXT_CONTROL_CODE )
      {
         if( (++returnCode) >= MIDI_COMMAND_COUNT )
         {
            returnCode = 0;
         }
      }
      else
      {
         if( (--returnCode) >= MIDI_COMMAND_COUNT )
         {
            returnCode = MIDI_COMMAND_COUNT-1;
         }
      }
      return FLASH_GET_PGM_BYTE(&MidiLookUpTable[returnCode].MIDI_Commands);
   }

   return FLASH_GET_PGM_BYTE(&MidiLookUpTable[0].MIDI_Commands);
}

/* Obtain the control code string */
void MIDI_ControlString(uint8_t command, char* buffer)
{
   uint8_t i;
   for(i = 0; FLASH_GET_PGM_BYTE(&MidiLookUpTable[i].MIDI_Commands); i++)
   {
      if( command == FLASH_GET_PGM_BYTE(&MidiLookUpTable[i].MIDI_Commands) )
      {
         strcpy_P(buffer, FLASH_GET_PGM_WORD(&MidiLookUpTable[i].MIDI_CommandString));
         return;
      }
   }
   strcpy_P(buffer, MIDI_UNDEFINED_STRING);
}


/* Obtain the control code size */
uint8_t MIDI_CommandSize(uint8_t command)
{
   uint8_t i;
   for(i = 0; FLASH_GET_PGM_BYTE(&MidiLookUpTable[i].MIDI_Commands); i++)
   {
      if( command == FLASH_GET_PGM_BYTE(&MidiLookUpTable[i].MIDI_Commands) )
      {
         return FLASH_GET_PGM_BYTE(&MidiLookUpTable[i].MIDI_MsgSize);
      }
   }
   return 1;
}


/* Pass any number from 0->127 and obtain the corresponding note in a string format */
void MIDI_NoteString(uint8_t note, char* buffer)
{
   note = note - (NOTE_COUNT * MIDI_Octave(note));
   strncpy(buffer, &MIDI_NOTES[note*2] , 2);
   /* Append a NULL */
   buffer[2] = '\0';
}

/* Returns the octave of the passed note */
uint8_t MIDI_Octave(uint8_t note)
{
  return (note / NOTE_COUNT);
}





void SetLastMIDIValue(uint8_t channel, uint8_t value)
{
	/* Update the last sample */
   MIDI_LastMIDIValue[channel] = value;
}

uint8_t GetLastMIDIValue(uint8_t channel)
{
   return MIDI_LastMIDIValue[channel];
}




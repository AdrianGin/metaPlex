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
#include <stdlib.h>


#include "hardwareSpecific.h"
#include "EDrum.h"
#include "Menu/Menu.h"
#include "UI_KP/UI_KP.h"
#include "UI_LCD/UI_LCD.h"
#include "UserFunctions/userFunctions.h"
#include "UserFunctions/gainAdjustFunctions.h"
#include "UserFunctions/dualTriggerAdjustFunctions.h"
#include "mmculib/uint16toa.h"
#include "mmculib/uint8toa.h"
#include "MIDI/midi.h"

#include "USBMIDIThru/USBMIDIThru.h"
#include "MIDI/SysEx/SysEx.h"
#include "Sample/sample.h"
#include "Sample/analogueSample.h"
#include "Sample/digitalSample.h"
#include "Sensor/sensor.h"
#include "TimerCallbacks/TimerCallbacks.h"

#include "ADC/adc12.h"
#include "Profiles/profiles.h"
#include "VUMeter/vumeter.h"
#include "MenuSettings.h"
#include "LCDSettings.h"

#include "edrumAVRsharedfunctions.h"

#if HAS_CONTROLLER_MODE
#include "ControllerMode/ControllerMode.h"
#endif

static uint8_t SelectedProfile = PROFILE_1;
uint8_t SelectedChannel;
char outputString[21];


void delayWithUSBPoll(uint8_t hundredms, uint8_t withDots)
{
   uint8_t i;
   SoftTimerStop(SoftTimer1[SC_MIDIScan]);
   SoftTimerStart(SoftTimer1[SC_usbPoll]);
	for( i = 0; i < hundredms; i++ )
	{
      if( withDots )
      {
		   UF_MenuChar('.');      	
      }
		_delay_ms(100);
	}
   SoftTimerStop(SoftTimer1[SC_usbPoll]);
   SoftTimerStart(SoftTimer1[SC_MIDIScan]);
}



void reset(void* data)
{
   _delay_ms(1000);
	hardwareReset();
}

/* Menu wrapper functions */
void UF_MenuSetInput(uint8_t NewInput)
{
   MenuSetInput(ActiveMenu, NewInput);
}

void UF_stateMachine(void)
{
   stateMachine(ActiveMenu, ActiveMenu->currentState);
}

void UF_executeState(uint8_t state)
{
   executeState(ActiveMenu, state);
}

void UF_MenuReset(void)
{
   ActiveMenu->MenuReset();
}

void UF_MenuNewLine(void)
{
   ActiveMenu->MenuNewLine();
}

void UF_MenuChar(uint8_t data)
{
   ActiveMenu->MenuChar(data);
}

void UF_MenuPrint_P(PGM_P string)
{
   ActiveMenu->MenuPrint_P(string);  
}   

void UF_MenuPrint(char* string)
{
   ActiveMenu->MenuPrint(string);    
}

void UF_MenuUpOneLevel(Menu_t* menu)
{
   Menu_UpOneLevel(menu);
}

/* End of wrapper functions */

//void nullFunction   

void about(void* data)
{
	uint8_t* input = data;
   uint8_t nameIndex;

	if( primaryMenu.firstEnter != 1 )
	{
		switch( *input )
		{	
	      case KP_BACK:      
			UI_LCD_LoadDefaultChars();	
	   	UF_MenuSetInput(KP_BACK);
	      UF_stateMachine();
			SoftTimerStop(SoftTimer2[SC_AboutUpdate]);
			ThanksIndex(MAIN_SCREEN);       
	      return;	
		}	

      
      nameIndex = ThanksIndex(GET);
      if( ++nameIndex == SIZEOFABOUT )
      {
         nameIndex = ThanksIndex(MAIN_SCREEN);   
      }
      ThanksIndex(nameIndex);

	}
	
	primaryMenu.firstEnter = 0;
	SoftTimerStart(SoftTimer2[SC_AboutUpdate]);
	aboutScroll(ThanksIndex(GET));	
}



/* Pass GET to obtain the index, otherwise the index is set to the passed
 * parameter */
uint8_t ThanksIndex(uint8_t mode)
{
	static uint8_t nameIndex = MAIN_SCREEN;
	if( mode == GET )
	{
		return nameIndex;
	}
	else
	{
		nameIndex = mode;
		return mode;
	}
}

void aboutScroll(uint8_t nameIndex)
{
	if( nameIndex > SIZEOFABOUT )
	{
		nameIndex = MAIN_SCREEN;	
	}

	const uint8_t lightning[][8] = {{158,143,135,140,156,142,130,129},
											  {143,158,156,134,135,142,136,144}};

   uint8_t i;
   uint16_t sampleRate;
   uint16_t sampleFrequency;

	switch( nameIndex )
	{
		case MAIN_SCREEN:

			UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)lightning[0], 0);
			UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)lightning[1], 1);
			
			UF_MenuReset();
			UF_MenuPrint_P( PSTR("  Roberto & Adrian"));
			UF_MenuNewLine();
			UF_MenuPrint_P(PSTR("      present:") );
			UF_MenuNewLine();	
			UF_MenuChar(0x00);
			UF_MenuPrint_P( PSTR("   metaPLEX 1.3   "));
			UF_MenuChar(0x01);
		   UF_MenuNewLine();
			UF_MenuPrint_P(PSTR("Version:") );
			UF_MenuPrint_P(VersionId);
		break;	
      
		case TECH_SPECS:

         for(i = 0; ActiveChannels[i] != LAST_CHANNEL_INDEX; i++)
         {
         }

         sampleRate = ADC_SAMPLE_SPEEDus + SensorSettings->CrosstalkDelay;
         sampleFrequency = (1000000 / sampleRate);


			UF_MenuReset();
			UF_MenuPrint_P( PSTR("  Technical Specs:"));
			UF_MenuNewLine();
			UF_MenuPrint_P(PSTR("ATmega32 @ ") );
         utoa(F_CPU/1e6, outputString, 10);
         UF_MenuPrint(outputString);
         UF_MenuPrint_P( PSTR(" MHz"));
			UF_MenuNewLine();	
			UF_MenuPrint_P( PSTR("Sample Rate:"));
         utoa(sampleFrequency, outputString, 10);
			UF_MenuPrint(outputString);
         UF_MenuPrint_P( PSTR(" Hz"));	
		   UF_MenuNewLine();
			UF_MenuPrint_P(PSTR("Active Channels:") );
         utoa(i, outputString, 10);
         UF_MenuPrint(outputString);
		break;   
      
		case CONTACT_INFO:

			UF_MenuReset();
			UF_MenuPrint_P(PSTR("   Contact Info:"));
			UF_MenuNewLine();
			UF_MenuPrint_P(PSTR("adrian.gin@gmail.com") );
			UF_MenuNewLine();
         UF_MenuPrint_P(PSTR("  http://lad-drum"));
         UF_MenuNewLine();
         UF_MenuPrint_P(PSTR("  .googlecode.com"));
		break;          		
	}
}

void DisableEdrum(void* data)
{
   uint8_t* input = (uint8_t*)data;
		
	if( primaryMenu.firstEnter != 1 )
   {     
		UF_stateMachine( );
      switch( *input )
      {
         case KP_UP:
         case KP_A:
            if( SoftTimerIsEnabled(SoftTimer1[SC_MIDIScan]) )
            {
               SoftTimer1[SC_MIDIScan].timerEnable = 0;
            }
            else
            {
               SoftTimer1[SC_MIDIScan].timerEnable = 1;
            }
            break;
         
			case KB_BACK:
			case KP_BACK:
				primaryMenu.firstEnter = 1;
				return;
			break;
				
			case KB_ENTER:
			case KP_ENTER:
			   UF_executeState( primaryMenu.currentState);
				return;	
			break;

			default:
			break;
		}	
	}
	UF_MenuReset();
	primaryMenu.firstEnter = 0;
   UF_MenuPrint_P( PSTR("eDrum Active:") );
   if( SoftTimerIsEnabled(SoftTimer1[SC_MIDIScan]) )
   {
      UF_MenuPrint_P( PSTR("Yes") ); 
   }
   else
   {
      UF_MenuPrint_P( PSTR("No") );
   }
   UF_MenuNewLine();
}


void USBMIDIThru(void* data)
{
   uint8_t* input = (uint8_t*)data;

   uint8_t usbmidiThruState;
		
	if( primaryMenu.firstEnter != 1 )
   {     
		UF_stateMachine();
      switch( *input )
      {
         case KP_UP:
         case KP_A:
            usbmidiThruState = !USBMIDIThruIsActive(GET_USBMIDITHRU_STATE);

            if( usbmidiThruState )
            {
               ActiveProcess = USB_MIDI_THRU;
            }
            else
            {
               ActiveProcess = DEFAULT_PROCESS;
            }

            USBMIDIThruIsActive( usbmidiThruState );
            break;
         
			case KB_BACK:
			case KP_BACK:
				primaryMenu.firstEnter = 1;
				return;
			break;
				
			case KB_ENTER:
			case KP_ENTER:
			   UF_executeState( primaryMenu.currentState);
				return;	
			break;

			default:
			break;
		}	
	}
	UF_MenuReset();
	primaryMenu.firstEnter = 0;

   UF_MenuPrint_P( PSTR("USB<->MIDI Thru:") );

   if( USBMIDIThruIsActive( GET_USBMIDITHRU_STATE ) )
   {
      UF_MenuPrint_P( PSTR("On") ); 
   }
   else
   {
      UF_MenuPrint_P( PSTR("Off") );
   }
   UF_MenuNewLine();
}


void USBMIDI_Monitor(void *data)
{
   uint8_t* input;
   input = data;

	

   UF_MenuUpOneLevel(&primaryMenu);

}



void SysExDisplay(void* data)
{
	uint8_t* input = (uint8_t*)data;
		
	if( primaryMenu.firstEnter != 1 )
   {     
		UF_stateMachine();
      switch( *input )
      {
			case KB_BACK:
			case KP_BACK:
				primaryMenu.firstEnter = 1;
				return;
			break;
				
			case KB_ENTER:
			case KP_ENTER:
			   UF_executeState( primaryMenu.currentState);
			   
				return;	
			break;

			default:
			break;
		}	
	}


	UF_MenuReset();
	primaryMenu.firstEnter = 0;

	/* Since each byte is sent as 2x 7 bit SysEx 'bytes' */
	utoa( sizeof(Profile_t) * 2, outputString, 10);
	
   UF_MenuPrint_P( PSTR("SysEx Size:") );

   UF_MenuPrint(outputString);
   UF_MenuPrint_P( PSTR(" bytes") ); 
   
   UF_MenuNewLine();
}

void DumpSysEx(void* data)
{
   uint8_t* input;
	   
   input = data;

	UF_MenuPrint_P( PSTR("Sending SysEx Data") );	
   UF_MenuNewLine();		
	UF_MenuPrint_P( PSTR("Sending") );	

   delayWithUSBPoll(4, WITH_DOTS);

   UF_MenuNewLine();		

   /* Stop output timers */
   SoftTimerStop(SoftTimer1[SC_MIDIScan]);
   ActiveProcess = SENDING_SYSEX;
}



void GetSysEx(void* data)
{
   uint8_t* input = 0;
	   
   input = data;

	UF_MenuPrint_P( PSTR("Ready for SysEx Data") );	
   UF_MenuNewLine();			

   delayWithUSBPoll(4, WITH_DOTS);

   UF_MenuNewLine();	
	IsReceivingSysExData(RECEIVING_SYSEX_DATA);
	
   
	if( primaryMenu.firstEnter != 1 )
   {     
      switch( *input )
      {
			case KB_BACK:
			case KP_BACK:
				primaryMenu.firstEnter = 1;
				SysExFlush();
            IsReceivingSysExData(!RECEIVING_SYSEX_DATA);
            UF_MenuPrint_P( PSTR("User cancelled!"));

            delayWithUSBPoll(8, 0);
         	
		   	UF_MenuUpOneLevel(&primaryMenu);
         	
				return;
			break;

			default:
			break;
		}	
	}
	SysExFlush();
   ActiveProcess = RECEIVE_SYSEX;
   
   /* Stop the Auxuliary Timer */
//	TBCCTL2 &= ~(CCIE);
//	TBCCTL0 &= ~(CCIE);
	primaryMenu.firstEnter = 0;
}


void FirmwareUpgrade(void* data)
{
   FirmwareInstructions(0);
   ActiveProcess = FIRMWARE_UPGRADE;
}

void FirmwareInstructions(void* data)
{
   if( data == DOWNLOAD_FAILED )
   {
      UF_MenuPrint_P( PSTR("  DOWNLOAD FAILED!") );
   }
   else
   {
      UF_MenuPrint_P( PSTR("Waiting for Firmware") );
   }
   UF_MenuNewLine();
   UF_MenuPrint_P( PSTR(" IF DOWNLOAD FAILS:") );
   UF_MenuNewLine();
	UF_MenuPrint_P( PSTR("  **HOLD ANY KEY**") );
   UF_MenuNewLine();
	UF_MenuPrint_P( PSTR("CYCLE POWER & RETRY") );
}



#if HAS_CONTROLLER_MODE
/* Play mode disables TimerB2 */
void ControllerMode(void* data)
{
	uint8_t* input = data;

	if( primaryMenu.firstEnter != 1 )
	{
		switch( *input )
		{	
         /* The LCD BL key does not exit however */
         case KP_5:
         
         break;
         
	      default:     
	   	UF_MenuSetInput(KP_BACK);
	      UF_stateMachine(primaryMenu.currentState);
	      UF_MenuSetInput(0);
	      /* Start the Aux Timer again */
//			TBCCTL2 |= (CCIE);
	      return;	
		}	
	}
		
	primaryMenu.firstEnter = 0;	

	CM_SetMenuMode(CM_SETTINGS_MODE);
	CM_printEnteredData();		
	/* Only used when KP inputs are buttons 
	for( i = 0; i < ANALOGUE_INPUTS; i++)
	{
		SetLastMIDIValue(i, MIDI_MAX_DATA);	
		SetLastSampleValue(i, MAX_THRESHOLD);	
	}*/
	
	/* Don't stop the Auxuliary Timer */
	ActiveProcess = CONTROLLER_MODE;
}




/** Function to setup each individual keypad button */
void KeypadButtonSettings(void* data)
{
   uint8_t* input = 0;
   
   SelectedChannel = GetState(&primaryMenu) - ST_CHANNEL_1;
	uint8_t SelectedDigitalChannel = SelectedChannel - ANALOGUE_INPUTS;
	uint8_t SelectedKeypadChannel = SelectedDigitalChannel - (DIGITAL_INPUTS + METRONOME_INPUTS);
	input = data;

 	if( primaryMenu.firstEnter != 1 )
 	{
	   switch( *input )
	   {
	      /* Up and down a Key */
	      case KP_1:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+1);
	      break;
	      
	      case KP_7:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)-1);  
	      break;
	      
	      /* Up  an octave */
	      case KP_STAR:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+NOTE_COUNT);
	      break;
	      
	      case KP_D:
            SetChannelCommand(SelectedChannel, GetChannelCommand(SelectedChannel) + 0x10 );
         break;
	      
	      /* Up and down output levels */
	      case KP_3:
				SetDigitalVelocity(SelectedDigitalChannel, 
										 GetDigitalVelocity(SelectedDigitalChannel)+1);
	      break;
	      
	      case KP_9:
				SetDigitalVelocity(SelectedDigitalChannel, 
										 GetDigitalVelocity(SelectedDigitalChannel)-1);  
	      break;
	      
	      case KP_HASH:
				SetDigitalVelocity(SelectedDigitalChannel, 
										 GetDigitalVelocity(SelectedDigitalChannel)+10);  
	      break;	      
	      
         case KP_BACK:
				/* Go back up one menu */   
   			UF_MenuSetInput(KB_BACK);
  				UF_stateMachine(primaryMenu.currentState);
  				UF_MenuSetInput(0);
  			return;
				

	      default:
	         break;
	   }
	}
	
   primaryMenu.firstEnter = 0;
   /* Indicate the Keypad selected */
	
	UF_MenuPrint_P(PSTR("Keypad Button "));
	
	if( SelectedKeypadChannel > 0x0A )
	{
		outputString[0] = (SelectedKeypadChannel - 0x09) + 'A';		
		
		if( SelectedKeypadChannel == 14 )
		{
		outputString[0] = '*';	
		}
	
   	if( SelectedKeypadChannel == 15 )
		{
			outputString[0] = '#';	
		}	
		outputString[1] = '\0';	
	}
	else
	{
		uint8toa( SelectedKeypadChannel, outputString );
	}
	
	UF_MenuPrint(outputString);          
	UF_MenuNewLine();	 
	
	UF_MenuPrint_P(PSTR("Note: "));
	
	if( GetChannelCommand(SelectedChannel) == MIDI_NOTE_ON )
	{
   	MIDI_NoteString(GetChannelKey(SelectedChannel), outputString);	
      UF_MenuPrint(outputString);
   	if( MIDI_Octave(GetChannelKey(SelectedChannel)) == 0 )
   	{
         UF_MenuPrint_P( PSTR("0"));  
      }
      else
      {  
      	uint8toa( MIDI_Octave(GetChannelKey(SelectedChannel)), outputString);
    		UF_MenuPrint(outputString); 	
      } 
   }  
   else
   {
      uint8toa( GetChannelCommand(SelectedChannel) >> 4, outputString);
      UF_MenuPrint(outputString);
      UF_MenuPrint_P( PSTR(" | ") );
   	uint8toa( GetChannelKey(SelectedChannel), outputString);
 		UF_MenuPrint(outputString);         
   }
   
	UF_MenuNewLine();	   
	
	/* Display the channel velocity */
	uint8toa(GetDigitalVelocity(SelectedDigitalChannel), outputString);
	UF_MenuPrint_P(PSTR("Velocity: "));
	UF_MenuPrint(outputString);   
}


#endif



void SetMIDIRate(void* data)
{
	uint8_t* input = (uint8_t*)data;
		
	if( primaryMenu.firstEnter != 1 )
   {     
		UF_stateMachine();
      switch( *input )
      {
			case KB_BACK:
			case KP_BACK:
				primaryMenu.firstEnter = 1;
				return;
			break;
				
			case KB_ENTER:
			case KP_ENTER:
            //primaryMenu.firstEnter = 0;
			   UF_executeState( primaryMenu.currentState);
				return;	
			break;

			default:
			break;
		}	
	}
	primaryMenu.firstEnter = 0;
	
	UF_MenuReset();
   PrintMIDIRate();
   UF_MenuNewLine(); 
}


void PrintMIDIRate(void)
{
	uint8_t selectedBaud = 0;
	
	UF_MenuPrint_P( PSTR("MIDI Output Rate: ") );
   UF_MenuNewLine(); 	
   UF_MenuPrint_P( PSTR("Baud Rate:"));
   
   switch(MIDI_GetBaud())
   {
		case MIDI_BAUD_31250:
			selectedBaud = B31250;
		break;
		
		case MIDI_BAUD_38400:
			selectedBaud = B38400;
		break;
		
		case MIDI_BAUD_115200:
			selectedBaud = B115200;
		break;
		
		case MIDI_BAUD_1M:
			selectedBaud = B1M;
		break;				
	
		default:
		break;
	}
      
  	UF_MenuPrint_P( MIDI_BAUD[selectedBaud] );	
}

void EditMIDIRate(void* data)
{
	uint8_t* input = (uint8_t*)data;
	uint8_t selectedBaud = 0;
	
   if( primaryMenu.firstEnter == 0 )
   {     
      switch( *input )
      {
         /* Increment msec delay */
         case KP_UP:
         case KP_C:
				   switch(MIDI_GetBaud())
				   {
						case MIDI_BAUD_31250:
							MIDI_SetBaud(MIDI_BAUD_38400); 
						break;
						
						case MIDI_BAUD_38400:
							MIDI_SetBaud(MIDI_BAUD_115200); 
						break;
						
						case MIDI_BAUD_115200:
							MIDI_SetBaud(MIDI_BAUD_1M); 
						break;		
						
						case MIDI_BAUD_1M:
							MIDI_SetBaud(MIDI_BAUD_31250);
						break;
					
						default:
						break;
					}            
         break;
  
         /* Time Component increment function */
         case KB_BACK:
         case KP_BACK:
				/* Go back up one menu */   
   			UF_MenuUpOneLevel(&primaryMenu);
  				return;
  				
         break;
                        
         default:
         break;     
      }
   }
   
   primaryMenu.firstEnter = 0;
   
   UF_MenuPrint_P( PSTR("Baud Rate:"));
   switch(MIDI_GetBaud())
   {
		case MIDI_BAUD_31250:
			selectedBaud = B31250;
		break;
		
		case MIDI_BAUD_38400:
			selectedBaud = B38400;
		break;
		
		case MIDI_BAUD_115200:
			selectedBaud = B115200;
		break;		
		
		case MIDI_BAUD_1M:
			selectedBaud = B1M;
	
		default:
		break;
	}
      
  	UF_MenuPrint_P( MIDI_BAUD[selectedBaud] );
}


void ChannelToggleFunction(void* data)
{
   ChannelToggle(SelectedChannel);
	UF_MenuUpOneLevel(ActiveMenu);
}

/* Handles the incrementing and decrementing of notes */
void NoteEditor(void* data, uint8_t keyType)
{
   uint8_t channelCommand;
   uint8_t channelKey;
   uint8_t* input = data;

   void (*setChannelKey)(uint8_t channel, uint8_t key);
   void (*setChannelCommand)(uint8_t channel, uint8_t command);

   if( keyType == OPEN_KEY)
   {
      channelKey  = GetChannelKey(SelectedChannel);
      channelCommand = GetChannelCommand(SelectedChannel);
      setChannelKey = &SetChannelKey;
      setChannelCommand = &SetChannelCommand;
   }
   else
   {
      channelKey  = GetChannelKeyClosed(SelectedChannel);
      channelCommand = GetClosedChannelCommand(SelectedChannel);
      setChannelKey = &SetChannelKeyClosed;
      setChannelCommand = &SetClosedChannelCommand;
   }


   switch( *input )
   {
         /* Up and down a Key */
         case KP_UP:
            if( channelCommand <= MIDI_SONG_SELECT )
            {
               if( ++channelKey == (MIDI_MAX_KEY + 1))
               {
                  channelKey = 0;
                  setChannelCommand(SelectedChannel,MIDI_GetControlCode(channelCommand, MIDI_NEXT_CONTROL_CODE));
               }
            }
            else
            {
               channelKey = 0;
               setChannelCommand(SelectedChannel,MIDI_GetControlCode(channelCommand, MIDI_NEXT_CONTROL_CODE));
            }

         break;

         case KP_DOWN:
            if( channelCommand <= MIDI_SONG_SELECT )
            {
               if( --channelKey == UINT8_MAX)
               {
                  channelKey = MIDI_MAX_KEY;
                  setChannelCommand(SelectedChannel,MIDI_GetControlCode(channelCommand, MIDI_PREVIOUS_CONTROL_CODE));
               }
            }
            else
            {
               channelKey = 0;
               setChannelCommand(SelectedChannel,MIDI_GetControlCode(channelCommand, MIDI_PREVIOUS_CONTROL_CODE));
            }
         break;
      }
      setChannelKey(SelectedChannel, channelKey);
}

void KeySelectFunction(void* data)
{
	uint8_t* input = data;

   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   NoteEditor(data, OPEN_KEY);

   switch( *input )
   {
      case KP_BACK:
         UF_MenuUpOneLevel(ActiveMenu);
         return;
   }

   if( SelectedChannel >= ANALOGUE_INPUTS )
   {
      PrintDigitalChannelSetup(1);
   }
   else
   {
      PrintAnalogueChannelSetup(1);
   }
   MenuUpdate(ActiveMenu, NO_EXECUTE);
   UF_MenuSetInput(KP_INVALID);

}

void PrintNoteFormat(uint8_t keyType)
{
   uint8_t channelKey;
   uint8_t channelCommand;

   if( keyType == OPEN_KEY)
   {
      channelKey  = GetChannelKey(SelectedChannel);
      channelCommand = GetChannelCommand(SelectedChannel);
   }
   else
   {
      channelKey  = GetChannelKeyClosed(SelectedChannel);
      channelCommand = GetClosedChannelCommand(SelectedChannel);
   }

   MIDI_ControlString(channelCommand, outputString);
   UF_MenuPrint(outputString);
   UF_MenuPrint_P( PSTR(")") );

   if( channelCommand <= MIDI_AFTERTOUCH )
   {
      MIDI_NoteString(channelKey, outputString);
      UF_MenuPrint(outputString);
      uint8toa( MIDI_Octave(channelKey), outputString);
   }
   else
   {
      
      uint8toa( channelKey, outputString);
   }
   UF_MenuPrint(outputString);
}

/** Function to setup each individual analogue channel */
void PrintAnalogueChannelSetup(uint8_t UpDownPosition)
{
   static uint8_t enterCount;

   ActiveMenu->parentMenu->firstEnter = 0;
   enterCount ^= (1);

   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

	MenuLCD_SetPos(0, 0);

	/* Indicate the channel selected */
	UF_MenuPrint_P(PSTR("Channel "));
	uint8toa(SelectedChannel + 1, outputString);
	UF_MenuPrint(outputString);

 	UF_MenuPrint_P(PSTR(": "));

   if( UpDownPosition == 0 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
   	if( GetChannelStatus(SelectedChannel) == CHANNEL_ON )
   	{
   		UF_MenuPrint_P( PSTR("On") );
   	}
   	else
   	{
   		UF_MenuPrint_P( PSTR("Off") );
   	}
   }
   UF_MenuNewLine();


	UF_MenuPrint_P(PSTR("Note:"));

   if( UpDownPosition == 1 && enterCount)
   {
      UF_MenuPrint_P( PSTR("               ") );
   }
   else
   {
      PrintNoteFormat(OPEN_KEY);
   }

	UF_MenuNewLine();

	/* Display the channel 'gain' */
	if( GetGainType(SelectedChannel) == LINEAR_GAIN )
	{
		UF_MenuPrint_P( PSTR("Linear ") );
	}
	else
	{
		UF_MenuPrint_P( PSTR("Non-Linear ") );
	}
	UF_MenuPrint_P(PSTR("Gain: "));

   if( UpDownPosition == 2 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
   	itoa( (int8_t)(GetChannelGain(SelectedChannel) - GAIN_OFFSET), outputString, 10);
   	UF_MenuPrint(outputString);
   }
   UF_MenuNewLine();

//	Don't hide sub children.
   analogueMenu.updateOptions = SHOW_CHILDREN;
   SelectedSubMenu = &analogueMenu;
   //ActiveMenu = &analogueMenu;
   UpdateActiveChannels();

}




void ChannelSetup(void* data)
{
   uint8_t* input = data;


   SelectedChannel = GetState(&primaryMenu) - ST_CHANNEL_1;

 	if( primaryMenu.firstEnter != 1 )
 	{
      UF_stateMachine();
	   switch( *input )
	   {

         case KP_UP:
         case KP_DOWN:
            MenuSetInput( &analogueMenu, *input );
            break;

	      /* Up and down a Key */
	      case KP_1:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+1);
	      break;

	      case KP_7:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)-1);
	      break;

	      /* Up an octave */
	      case KP_STAR:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+NOTE_COUNT);
	      break;

	      case KP_D:
            SetChannelCommand(SelectedChannel, GetChannelCommand(SelectedChannel) + 0x10 );
         break;

	      /*
	      case KP_7:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)-NOTE_COUNT);
	      break;*/

	      /* Up and down a Gain */
	      case KP_B:
				SetChannelGain(SelectedChannel, (GetChannelGain(SelectedChannel)+1));
	      break;

	      case KP_C:
				SetChannelGain(SelectedChannel, (GetChannelGain(SelectedChannel)-1));
	      break;

	      /* Setting Modifiers */
	      case KP_A:
				ChannelToggle(SelectedChannel);
	      	break;

        case KB_BACK:
        case KP_BACK:
           SoftTimerStop(SoftTimer2[SC_AutoMenuUpdate]);
           MenuSetInput( &analogueMenu, 0 );
           primaryMenu.firstEnter = 1;
			return;

        case KB_ENTER:
        case KP_ENTER:
                UF_MenuReset();
                ActiveMenu = &analogueMenu;
                UF_MenuSetInput(*input);
                /* We need to explicitly switch the state since
                 the submenu will need to parse the input, since the
                 head function has a function (HandleSubMenu), the MenuUpdate
                 will not switch the state for us.
                 */
                UF_stateMachine();
                MenuUpdate(&analogueMenu, 0);
                primaryMenu.firstEnter = 1;
            return;



	      default:
	         break;
	   }
	}

 	SoftTimer2[SC_AutoMenuUpdate].timeCompare = SLOW_AUTO_MENU_UPDATE;
   PrintAnalogueChannelSetup(analogueMenu.selectedItem);
   MenuUpdate(&analogueMenu, 0);
   MenuSetInput( &analogueMenu, KP_INVALID );
}


void SetDualInput(void* data)
{
   uint8_t* input;
	input = data;

	Menu_t* newParentMenu;

   if( SelectedChannel < ANALOGUE_INPUTS)
   {
      newParentMenu = &analogueMenu;
   }
   else
   {
      newParentMenu = &digitalMenu;
   }
   dualTrigMenu.parentMenu = newParentMenu;


   if( ActiveMenu->firstEnter != 1 )
   {

      switch( *input )
      {

         case KP_UP:
         case KP_DOWN:
            MenuSetInput( &dualTrigMenu, *input );
            break;

         /* Up and down a Key for Open Key*/
         case KP_1:
            SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+1);
         break;

         case KP_7:
            SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)-1);
         break;

         /* Up an octave */
         case KP_STAR:
            SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+NOTE_COUNT);
         break;

         /* Up and down a Key for Closed */
         case KP_3:
            SetChannelKeyClosed(SelectedChannel, GetChannelKeyClosed(SelectedChannel)+1);
         break;

         case KP_9:
            SetChannelKeyClosed(SelectedChannel, GetChannelKeyClosed(SelectedChannel)-1);
         break;

         /* Up an octave */
         case KP_HASH:
            SetChannelKeyClosed(SelectedChannel, GetChannelKeyClosed(SelectedChannel)+NOTE_COUNT);
         break;

         /* Setting Modifiers */
         case KP_A:
            DualModeToggle(SelectedChannel);
         break;

         /* Digital Trigger Select */
         case KP_B:
            SetTrigger(SelectedChannel, GetTrigger(SelectedChannel)+1);
         break;

         case KP_C:
            SetTrigger(SelectedChannel, GetTrigger(SelectedChannel)-1);
         break;

         case KB_BACK:
         case KP_BACK:
            SoftTimerStop(SoftTimer2[SC_AutoMenuUpdate]);
            //UF_stateMachine( analogueMenu.currentState );
            UF_stateMachine();
            ActiveMenu = &primaryMenu;
            SelectedSubMenu = newParentMenu;

            MenuSetInput(newParentMenu, KP_INVALID);
            executeState(ActiveMenu, ActiveMenu->currentState);

            newParentMenu->updateOptions = RESET_MENU | HIDE_CHILDREN;
            newParentMenu->firstEnter = 1;


            return;

         case KB_ENTER:
         case KP_ENTER:

            UF_MenuReset();
            ActiveMenu = &dualTrigMenu;
            SelectedSubMenu = &dualTrigMenu;

            MenuSetInput( &dualTrigMenu, *input );
            UF_stateMachine();
            MenuUpdate(&dualTrigMenu, 0);
            newParentMenu->firstEnter = 1;
            MenuSetInput( &dualTrigMenu, 0 );


            return;
         break;

         default:
            break;
      }
   }

   ActiveMenu->firstEnter = 0;


   SoftTimer2[SC_AutoMenuUpdate].timeCompare = SLOW_AUTO_MENU_UPDATE;

   //dualTrigMenu.updateOptions = HIDE_CHILDREN;

   MenuUpdate(&dualTrigMenu, 0);

   //stateMachine(&dualTrigMenu, dualTrigMenu.currentState);
   //return;
   MenuSetInput( &dualTrigMenu, KP_INVALID );


   PrintDualTriggerInformation(dualTrigMenu.selectedItem);

   //
}



void SetThreshold(void* data)
{
	uint8_t* input;
   
   static uint16_t lastPotValue;
	static uint8_t firstEnter = 1;
	
   input = data;
   
	uint16_t PotValue = SensorPotValue() >> (THRESHOLD_ADJUST);

	SoftTimer2[SC_AutoMenuUpdate].timeCompare = SMOOTH_AUTO_MENU_UPDATE;
	SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);
	

	if( firstEnter == 1)
	{
      LCD_Load_ProgressBar();
	}
	
	
	switch( *input )
	{
         /* Up and down a Threshold Level */
         case KP_UP:
            SetChannelThresh(SelectedChannel,GetChannelThresh(SelectedChannel) + 1);
            break;
         case KP_A:
				SetChannelThresh(SelectedChannel, (((GetChannelThresh(SelectedChannel) >> THRESHOLD_LEVELS)+1) << THRESHOLD_LEVELS));
         break;
         
         case KP_DOWN:
            SetChannelThresh(SelectedChannel,GetChannelThresh(SelectedChannel) - 1);
            break;
         case KP_B:
				SetChannelThresh(SelectedChannel, (((GetChannelThresh(SelectedChannel) >> THRESHOLD_LEVELS)-1) << THRESHOLD_LEVELS));  
         break;
	
	      case KP_C:
				SoftTimer2[SC_AutoMenuUpdate].timerEnable ^= 1;  
	      break;  	  
			       
         case KP_BACK:
            SoftTimer2[SC_AutoMenuUpdate].timeCompare = DEFAULT_AUTO_MENU_UPDATE;
				SoftTimerStop(SoftTimer2[SC_AutoMenuUpdate]);
         	UF_MenuUpOneLevel(&analogueMenu); 
            firstEnter = 1;
         return;
	}
		
	firstEnter = 0;

   MenuLCD_SetPos(0, 0);

	SetChannelThresh(SelectedChannel, GetChannelThresh(SelectedChannel) - lastPotValue + PotValue);
	lastPotValue = PotValue;

	uint8toa((GetChannelThresh(SelectedChannel) >> THRESHOLD_LEVELS), outputString);
	UF_MenuPrint_P(PSTR("Threshold Level: "));
	UF_MenuPrint(outputString);    
	
	UF_MenuNewLine(); 
	UF_MenuPrint_P(PSTR("Fine Tune:")); 
	
	UI_LCD_Pos(&PrimaryDisplay, 1, 10);         
   lcdProgressBar((GetChannelThresh(SelectedChannel)%(1<<THRESHOLD_LEVELS) ),(1<<THRESHOLD_LEVELS), 10);

	UF_MenuNewLine();   
	utoa(GetChannelThresh(SelectedChannel), outputString, 10);
	UF_MenuPrint_P(PSTR("Threshold: "));
	UF_MenuPrint(outputString); 
   
	UF_MenuNewLine();   
	utoa(SensorPotValue(), outputString, 10);
	UF_MenuPrint_P(PSTR("PotVal: "));
	UF_MenuPrint(outputString);           
    
}




void SetRetrigger(void* data)
{
	uint8_t* input;
   
   static uint8_t adjustStyle = DIGITAL_ADJUST;
	static uint8_t firstEnter = 1;
	
   input = data;

	if( firstEnter == 1)
	{
      LCD_Load_ProgressBar();
	}	

	SoftTimer2[SC_AutoMenuUpdate].timeCompare = SMOOTH_AUTO_MENU_UPDATE;

	switch( *input )
	{
         /* Increase and decrease retrigger level */
	      case KP_A:
				SetChannelReTrig(SelectedChannel,  (int16_t)(GetChannelReTrig(SelectedChannel) + 10 ));  
	      break;  	           
         
         case KP_UP:
         case KP_B:
				SetChannelReTrig(SelectedChannel, (int16_t)(GetChannelReTrig(SelectedChannel) + 1));
         break;
         
         case KP_DOWN:
         case KP_C:
				SetChannelReTrig(SelectedChannel,  (int16_t)(GetChannelReTrig(SelectedChannel) -1 ));        			
         break;
	
	      case KP_D:
				SetChannelReTrig(SelectedChannel,  (int16_t)(GetChannelReTrig(SelectedChannel) - 10 ));  
	      break;  	      
			
			
			/* Toggle retrigger adjustment style */
			case KP_HASH:
				adjustStyle ^= 1;
				if( adjustStyle == ANALOGUE_ADJUST )
				{
					SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);
				}
				else
				{
					SoftTimerStop(SoftTimer2[SC_AutoMenuUpdate]);
				}
			break;
			
         case KP_BACK:
            SoftTimer2[SC_AutoMenuUpdate].timeCompare = DEFAULT_AUTO_MENU_UPDATE;
				SoftTimerStop(SoftTimer2[SC_AutoMenuUpdate]);
				adjustStyle = DIGITAL_ADJUST;
				if( SelectedChannel >= ANALOGUE_INPUTS )
				{ 
         	   UF_MenuUpOneLevel(&digitalMenu); 
            }
            else
            {
               UF_MenuUpOneLevel(&analogueMenu); 
            }
            firstEnter = 1;
         return;
	}
		
	firstEnter = 0;
   MenuLCD_SetPos(0, 0);

	if( adjustStyle == ANALOGUE_ADJUST )
	{
		uint16_t PotValue = SensorPotValue() >> (RETRIGGER_ADJUST);
		SetChannelReTrig(SelectedChannel, PotValue + MIN_RETRIGGER);	
	}

	UF_MenuPrint_P(PSTR("Use # to change adj."));
	UF_MenuNewLine();	
	
	UF_MenuPrint_P(PSTR("Retrigger Level: "));
	UF_MenuNewLine();	
	uint8toa(GetChannelReTrig(SelectedChannel), outputString);
	UF_MenuPrint(outputString);
	UF_MenuPrint_P(PSTR("0 ms "));	
   
	UI_LCD_Pos(&PrimaryDisplay, 2, 9);
	 
   lcdProgressBar(GetChannelReTrig(SelectedChannel), MAX_RETRIGGER, 11);   	
	    
	UF_MenuNewLine(); 

   UF_MenuPrint_P(PSTR("BPM: "));	
   uint16_t BPM = (RETRIGGER_RESOLUTION / GetChannelReTrig(SelectedChannel));
	utoa(BPM, outputString, 10);
	UF_MenuPrint(outputString);
   
	//UF_MenuPrint_P(PSTR("Default Retrig: 10ms"));	
	
   UpdateChannelRetriggers();
   	
}


void SetGainCurves(void* data)
{

   uint8_t* input;
   
   static int8_t presetSetting = CUSTOM;
	input = data;


   switch( *input )
   {
      /* Up and down a Key for Open Key*/
      case KP_1:
			SetChannelGain(SelectedChannel, GetChannelGain(SelectedChannel)+1);
      break;
      
      case KP_7:
			SetChannelGain(SelectedChannel, GetChannelGain(SelectedChannel)-1);  
      break;
      
      /* Up an octave 
      case KP_STAR:
			SetChannelGain(SelectedChannel, GetChannelGain(SelectedChannel)+NOTE_COUNT);
      break;*/
      
      /* Up and down a Key for Closed */
      case KP_3:
			SetSlope2Gain(SelectedChannel, GetSlope2Gain(SelectedChannel)+1);
      break;
      
      case KP_9:
			SetSlope2Gain(SelectedChannel, GetSlope2Gain(SelectedChannel)-1);  
      break;
      
      /* Up an octave
      case KP_HASH:
			SetSlope2Gain(SelectedChannel, GetSlope2Gain(SelectedChannel)+NOTE_COUNT);
      break;*/
      
      /* Setting Modifiers */
      case KP_A:
			GainTypeToggle(SelectedChannel);
      break;	     
		
		case KP_B:
			if( ++presetSetting >= NUMBER_OF_GAIN_PRESETS )
			{
				presetSetting = 0;
			}
		break;
		
		case KP_C:
			if( --presetSetting < 0 )
			{
				presetSetting = CUSTOM;
			}
		break; 
      
      
      case KP_2:
			SetCrossover(SelectedChannel, (int16_t)GetCrossover(SelectedChannel)+50);
      break;
      
      case KP_8:
			SetCrossover(SelectedChannel, (int16_t)GetCrossover(SelectedChannel)-50);  
      break;
      
      case KP_0:
			SetCrossover(SelectedChannel, (int16_t)GetCrossover(SelectedChannel)+100);			
		break;
      
      case KB_BACK:
      case KP_BACK:      
      	UF_MenuUpOneLevel(&analogueMenu);     
         return;
         
      default:
         break;
   }
    PrintGainInformation(0xFF);
			
	
}
void MonitorChannel(void* data)
{
	uint8_t* input = data;

	switch( *input )
	{
      case KP_BACK:
         /* Stop and restore timers */
         SoftTimerStart(SoftTimer1[SC_MIDIScan]);
			SoftTimerStop(SoftTimer2[SC_MonitorChannel]);
			if( SelectedChannel >= ANALOGUE_INPUTS )
			{
      	   UF_MenuUpOneLevel(&digitalMenu);
         }
         else
         {
            UF_MenuUpOneLevel(&analogueMenu);
         }
      return;
	}

   SoftTimerStop(SoftTimer1[SC_MIDIScan]);
	SoftTimerStart(SoftTimer2[SC_MonitorChannel]);

}



// progress bar defines
#define PROGRESSPIXELS_PER_CHAR	6

void lcdProgressBar(uint16_t progress, uint16_t maxprogress, uint8_t length)
{
	uint8_t i;
	uint32_t pixelprogress;
	uint8_t c;

	// draw a progress bar displaying (progress / maxprogress)
	// starting from the current cursor position
	// with a total length of "length" characters
	// ***note, LCD chars 0-5 must be programmed as the bar characters
	// char 0 = empty ... char 5 = full

	// total pixel length of bargraph equals length*PROGRESSPIXELS_PER_CHAR;
	// pixel length of bar itself is
	pixelprogress = ((progress*(length*PROGRESSPIXELS_PER_CHAR))/maxprogress);

	// print exactly "length" characters
	for(i=0; i<length; i++)
	{
		// check if this is a full block, or partial or empty
		// (uint16_t) cast is needed to avoid sign comparison warning
		if( ((i*(uint16_t)PROGRESSPIXELS_PER_CHAR)+5) > pixelprogress )
		{
			// this is a partial or empty block
			if( ((i*(uint16_t)PROGRESSPIXELS_PER_CHAR)) > pixelprogress )
			{
				// this is an empty block
				// use space character?
				c = 1;
			}
			else
			{
				// this is a partial block
				c = (pixelprogress % PROGRESSPIXELS_PER_CHAR)+1;
			}
		}
		else
		{
			// this is a full block
			c = 6;
		}

		// write character to display
		UI_LCD_Char(&PrimaryDisplay, c);
	}
}

void LCD_Load_ProgressBar(void)
{
	// load the progress bar Chars
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[0], 1);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[1], 2);
   UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[2], 3);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[3], 4);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[4], 5);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[5], 6);
   UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[6], 7);
}



		
	


void PrintDigitalChannelSetup(uint8_t UpDownPosition)
{
   uint8_t SelectedDigitalChannel = SelectedChannel - ANALOGUE_INPUTS;
   static uint8_t enterCount;
   primaryMenu.firstEnter = 0;
   enterCount ^= (1);

   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   MenuLCD_SetPos(0, 0);

   /* Indicate the channel selected */
   if( SelectedDigitalChannel >= DIGITAL_INPUTS )
   {
      UF_MenuPrint_P(PSTR("Metronome Ch "));
      uint8toa(SelectedDigitalChannel - 7, outputString);
   }
   else
   {
      UF_MenuPrint_P(PSTR("Digital Ch "));
      uint8toa(SelectedDigitalChannel + 1, outputString);
   }
   UF_MenuPrint(outputString);
   UF_MenuPrint_P(PSTR(":"));

   if( UpDownPosition == 0 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
      if( GetChannelStatus(SelectedChannel) == CHANNEL_ON )
      {
         UF_MenuPrint_P( PSTR("On") );
      }
      else
      {
         UF_MenuPrint_P( PSTR("Off") );
      }
   }
   UF_MenuNewLine();
   

   UF_MenuPrint_P(PSTR("Note:"));
   if( UpDownPosition == 1 && enterCount)
   {
      UF_MenuPrint_P( PSTR("               ") );
   }
   else
   {
      PrintNoteFormat(OPEN_KEY);
   }
   UF_MenuNewLine();

   /* Display the channel velocity */
   UF_MenuPrint_P(PSTR("Velocity: "));
   if( UpDownPosition == 2 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
      uint8toa(GetDigitalVelocity(SelectedDigitalChannel), outputString);
      UF_MenuPrint(outputString);
   }
   UF_MenuNewLine();   

// Don't hide sub children.
   digitalMenu.updateOptions = SHOW_CHILDREN;
   SelectedSubMenu = &digitalMenu;
}


void VelocityAdjustFunction(void* data)
{
   uint8_t* input = data;
   uint8_t SelectedDigitalChannel = SelectedChannel - ANALOGUE_INPUTS;

   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   switch( *input )
   {
      /* Up and down a Key */
      case KP_UP:
         SetDigitalVelocity(SelectedDigitalChannel,
                            GetDigitalVelocity(SelectedDigitalChannel)+1);
      break;

      case KP_DOWN:
         SetDigitalVelocity(SelectedDigitalChannel,
                            GetDigitalVelocity(SelectedDigitalChannel)-1);
      break;

      case KP_BACK:
         UF_MenuUpOneLevel(ActiveMenu);
      return;
   }

   if( SelectedChannel >= ANALOGUE_INPUTS )
   {
      PrintDigitalChannelSetup(2);
   }
   MenuUpdate(ActiveMenu, NO_EXECUTE);
   UF_MenuSetInput(KP_INVALID);

}

void DigitalChannelSettings(void* data)
{
   uint8_t* input;
   
   SelectedChannel = GetState(&primaryMenu) - ST_CHANNEL_1;
   uint8_t SelectedDigitalChannel = SelectedChannel - ANALOGUE_INPUTS;
	input = data;

 	if( primaryMenu.firstEnter != 1 )
 	{
		UF_stateMachine();
	   switch( *input )
	   {
         case KP_UP:
         case KP_DOWN:
            MenuSetInput( &digitalMenu, *input );
         break;
         
	      /* Up and down a Key */
	      case KP_1:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+1);
	      break;
	      
	      case KP_7:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)-1);  
	      break;
	      
	      /* Up  an octave */
	      case KP_STAR:
				SetChannelKey(SelectedChannel, GetChannelKey(SelectedChannel)+NOTE_COUNT);
	      break;
	      
	      case KP_D:
            SetChannelCommand(SelectedChannel, GetChannelCommand(SelectedChannel) + 0x10 );
         break;
	      
	      
	      /* Up and down output levels */
	      case KP_3:
				SetDigitalVelocity(SelectedDigitalChannel, 
										 GetDigitalVelocity(SelectedDigitalChannel)+1);
	      break;
	      
	      case KP_9:
				SetDigitalVelocity(SelectedDigitalChannel, 
										 GetDigitalVelocity(SelectedDigitalChannel)-1);  
	      break;
	      
	      case KP_HASH:
				SetDigitalVelocity(SelectedDigitalChannel, 
										 GetDigitalVelocity(SelectedDigitalChannel)+10);  
	      break;	
	      /* Setting Modifiers */
	      case KP_A:
				ChannelToggle(SelectedChannel);
	      break;
	               
	      
			case KB_BACK:
			case KP_BACK:
				primaryMenu.firstEnter = 1;
				return;
			break;
				
			case KB_ENTER:
			case KP_ENTER:
            UF_MenuReset();
				ActiveMenu = &digitalMenu;
            MenuSetInput( &digitalMenu, *input );
				executeState(&digitalMenu, digitalMenu.currentState);
				primaryMenu.firstEnter = 1;
				return;	
			break;
	         
	         
	      default:
	         break;
	   }
	}
   
   SoftTimer2[SC_AutoMenuUpdate].timeCompare = SLOW_AUTO_MENU_UPDATE;
   PrintDigitalChannelSetup(digitalMenu.selectedItem);
   MenuUpdate(&digitalMenu, 0);
   MenuSetInput( &digitalMenu, KP_INVALID );
}





void VUMeterSetup(void* data)
{

	uint8_t* input = data;
	static uint8_t firstEnter = 1;
   uint8_t VUMeterIndex = GetState(&primaryMenu) - ST_VUMETER;

	if( firstEnter == 1)
	{
		/* Load VU Meter */
		UI_LCD_LoadDefaultChars();
	}

	switch( *input )
	{         
			case KP_A:
         case KP_UP:
				VUSetRows(GetVURows() + 1);	
			   SoftTimer2[SC_VUDecay].timeCompare = (MAX_ROWS + 1 - GetVURows()) << 2;
						
			break;
		
		
         case KP_BACK:
				SoftTimerStop(SoftTimer2[SC_VUDecay]);	
            
            if( VUMeterIndex == 0 )
            {
				  SoftTimerStop(SoftTimer2[SC_VUMeterUpdate]);
            }
            else
            {
              SoftTimerStop(SoftTimer2[SC_DigitalVUUpdate]);   
            }
							
				UF_MenuUpOneLevel(&primaryMenu);
            firstEnter = 1;
            /* Reset the VU Height */
            VUSetRows(DEFAULT_VU_HEIGHT);
         return;
	}
	firstEnter = 0;
		
	/* Start the VU Meter */
	UF_MenuReset();
	UF_MenuPrint_P(PSTR("123456789ABCDEFG MAX"));
	
	
	VUSetPosition(1,18);
	UF_MenuPrint_P(PSTR("96"));
	VUSetPosition(2,18);
	UF_MenuPrint_P(PSTR("64"));	
	VUSetPosition(3,18);
	UF_MenuPrint_P(PSTR("32"));	
	
	if( GetVURows() == MAX_ROWS )
	{
		VUSetPosition(0,0);
	}
	else
	{
		VUSetPosition(1,0);
	}

   /* Start the correct timer */
   if( VUMeterIndex == 0 )
   {
	  SoftTimerStart(SoftTimer2[SC_VUMeterUpdate]);
   }
   else
   {
     SoftTimerStart(SoftTimer2[SC_DigitalVUUpdate]);   
   }

	SoftTimerStart(SoftTimer2[SC_VUDecay]);
}



/* Amplifer Input Select */
void AmpInputSelect(void* data)
{
	uint8_t* input = (uint8_t*)data;
	
	if( primaryMenu.firstEnter != 1 )
   {     
		UF_stateMachine();
      switch( *input )
      {
			case KB_BACK:
			case KP_BACK:
				primaryMenu.firstEnter = 1;
				return;
			break;
				
			case KB_ENTER:
			case KP_ENTER:
			   UF_executeState( primaryMenu.currentState);
			break;

			default:
			break;
		}	
	}
	
	UF_MenuReset();
	primaryMenu.firstEnter = 0;
	UF_MenuPrint_P( PSTR("Trigger Using:") );
	
	UF_MenuNewLine();
	
	if( GetSensorInput() == ADC_CHANNEL(0) )
	{
		UF_MenuPrint_P( PSTR("Fixed Gain (1x)") );		
	}
	else
	{
		UF_MenuPrint_P( PSTR("Variable Gain (POTx)") );	
	}
	
   UF_MenuNewLine(); 		
}




void SensorInputChange(void* data)
{
	/* Corresponding to either SENSOR_OUT or SENSOR_OUT2 */
	uint8_t SelectedState = GetState(&primaryMenu);
	
	SensorInputSelect( (SelectedState == ST_VARIABLE_GAIN) ? ADC_CHANNEL(DEFAULT_ADC_CHANNEL) : ADC_CHANNEL(0) );

   UF_MenuSetInput(KP_BACK);
   UF_stateMachine();
   UF_MenuSetInput(0);	
	
}




/* Profiles */
void ShowProfile(void* data)
{
	uint8_t* input = (uint8_t*)data;
	static uint8_t firstEnter = 1;
	
		
	if( firstEnter != 1 )
   {     
		UF_stateMachine();
      switch( *input )
      {
			case KB_BACK:
			case KP_BACK:
				firstEnter = 1;
				return;
			break;
				
			case KB_ENTER:
			case KP_ENTER:
				UF_MenuReset();
		   	UF_executeState(primaryMenu.currentState);
				firstEnter = 1;
				return;	
			break;

			default:
			break;
		}	
	}

	firstEnter = 0;

	UF_MenuPrint_P( PSTR("Current Profile: ") );
   UF_MenuNewLine();
   
   UF_MenuPrint_P((PROGRAM_PTR)MenuDescriptor(&primaryMenu, ST_LOAD_PROFILE_DEF+SelectedProfile));

   UF_MenuNewLine();	
}

/* Save the CurrentProfile into the passed profileIndex */
void SaveProfile(void* data)
{
   uint8_t* input;
   uint8_t   ProfileSlot = GetState(&primaryMenu) - ST_SAVE_PROFILE_1 + 1;
	  
   input = data;

   if( ProfileSlot < NUMBER_OF_PROFILES )
   {

   	UF_MenuPrint_P( PSTR("Saving Data") );	
      UF_MenuNewLine();		
   	UF_MenuPrint_P( PSTR("Please Wait") );	

      delayWithUSBPoll(4, WITH_DOTS);
	
      UF_MenuNewLine();
      /* Because we go into the Bootarea, we need to disable all uneccesary interrupts */
   	Profile_Write(&CurrentProfile, ProfileSlot);
      

   	UF_MenuPrint_P( PSTR("Profile successfully") );
      UF_MenuNewLine();		
   	UF_MenuPrint_P( PSTR("saved!") );
   }
   else
   {
      Profile_Error();
   }

   delayWithUSBPoll(10, 0);

	UF_MenuUpOneLevel(&primaryMenu);
	UF_MenuReset();
}

void LoadProfile(void* data)
{
   uint8_t* input;
   uint8_t   ProfileSlot = GetState(&primaryMenu) - ST_LOAD_PROFILE_DEF;
	   
   input = data;

	UF_MenuPrint_P( PSTR("Retrieving Data") );	
   UF_MenuNewLine();		
	UF_MenuPrint_P( PSTR("Please Wait") );	

   delayWithUSBPoll(4, WITH_DOTS);

   UF_MenuNewLine();		

   if( ProfileSlot < NUMBER_OF_PROFILES )
   {

   	Profile_Read(ProfileSlot);
      /* Implement the changes */
   	MIDI_SetBaud(MIDI_GetBaud());
   	MIDI_SetChannelCode( MIDI_GetChannelCode() );

   	/* Update the sensor select */
   	SensorInputSelect(GetSensorInput());
	

      /* Update the Retrigger periods */
      UpdateChannelRetriggers();
      UpdateActiveChannels();

      UF_MenuPrint_P((PROGRAM_PTR)MenuDescriptor(&primaryMenu, ST_LOAD_PROFILE_DEF+ProfileSlot));
      UF_MenuNewLine();		
   	UF_MenuPrint_P( PSTR("successfully loaded!") );

   	SelectedProfile = ProfileSlot;
   }
   else
   {
      Profile_Error();
   }

   delayWithUSBPoll(20, 0);
	
	UF_MenuUpOneLevel(&primaryMenu);
	UF_MenuReset();
}


void Profile_Error(void)
{
   UF_MenuReset();
	UF_MenuPrint_P( PSTR("Only ("));
	uint8toa( NUMBER_OF_PROFILES - 1, outputString );
	UF_MenuPrint(outputString);				

	UF_MenuPrint_P( PSTR(") profiles") );  
   UF_MenuNewLine();		
	UF_MenuPrint_P( PSTR("in this version!") );  
}

void AdjustCrosstalk(void* data)
{
	uint8_t* input = (uint8_t*)data;
	int16_t crosstalk;

	
	crosstalk = GetCrossTalkDelay();
	
	
   if( primaryMenu.firstEnter == 0 )
   {     
      switch( *input )
      {
         /* Increment crosstalk delay by 10 */
         case 'q':
         case KP_A:
				crosstalk += 10;
            if( crosstalk > MAX_CROSSTALK)
            {
               crosstalk = MIN_CROSSTALK;   
            }   
         break;
            
         /* Increment crosstalk delay by 1*/  
         case 'a': 
         case KP_UP:                  
         case KP_B:
				crosstalk += 1;
            if( crosstalk > MAX_CROSSTALK)
            {
               crosstalk = MIN_CROSSTALK;   
            }   
         break;

         /* decrement crosstalk delay by 1*/
         case KP_DOWN:  
         case KP_C:
				crosstalk -= 1;
            if(crosstalk < MIN_CROSSTALK)
            {
               crosstalk = MAX_CROSSTALK;   
            }
         break;     
         
         /* Decrement crosstalk delay by 10 */  
        case KP_D:
				crosstalk -= 10;
            if(crosstalk < MIN_CROSSTALK)
            {
               crosstalk = MAX_CROSSTALK;   
            }
         break;       
           
         /* Time Component increment function */
         case KB_BACK:
         case KP_BACK:
				/* Go back up one menu */   
   			UF_MenuSetInput(KB_BACK);
  				UF_stateMachine();
  				UF_MenuSetInput(0);
  				return;
  				
         break;
                        
         default:
         break;     
      }
   }
   
   primaryMenu.firstEnter = 0;
   
   SetCrossTalkDelay(crosstalk);
   
   UF_MenuPrint_P( PSTR("Crosstalk Delay:"));
   UF_MenuNewLine();
	utoa(crosstalk, outputString, 10);
	UF_MenuPrint(outputString);		
   UF_MenuPrint_P( PSTR(" us"));
	
}






void ChangeChannelCode(void* data)
{
	uint8_t* input = (uint8_t*)data;
	int8_t code;
	
	if( primaryMenu.firstEnter == 1)
	{
		
	}
	
	code = MIDI_GetChannelCode();

   if( primaryMenu.firstEnter == 0 )
   {     
      switch( *input )
      {
         /* Increment channel # */
         case 'q':
         case KP_UP:
         case KP_A:
            if( ++code > MAX_MIDI_CHANNEL)
            {
               code = 0;   
            }   
         break;
            
         /* Deccrement channel #*/  
         case 'a':
         case KP_DOWN:                   
         case KP_B:
            if(--code < 0)
            {
               code = MAX_MIDI_CHANNEL;   
            }
         break;
           
         /* Time Component increment function */
         case KB_BACK:
         case KP_BACK:
				/* Go back up one menu */   
   			UF_MenuSetInput(KB_BACK);
  				UF_stateMachine();
  				UF_MenuSetInput(0);
  				return;
  				
         break;
                        
         default:
         break;     
      }
   }
   
   primaryMenu.firstEnter = 0;
   
   MIDI_SetChannelCode(code);
   
   UF_MenuPrint_P( PSTR("eDrum MIDI Code: "));
	uint8toa( code + 1, outputString);
	UF_MenuPrint(outputString);		
}


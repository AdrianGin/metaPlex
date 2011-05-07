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

/* SysEx Data Routines */



#include <stdint.h>
#include <stdlib.h>
#include "EDrum.h"
#include "hardwareSpecific.h"
#include "MIDI/midi.h"
#include "Profiles/profiles.h"
#include "Menu/Menu.h"
#include "MenuSettings.h"
#include "SoftTimer/softtimer.h"
#include "SysEx.h"

#include "hardUart/hardUart.h"
#include "edrumAVRsharedfunctions.h"

static uint16_t DataCount;
#define putMIDI(x) MIDI_Tx(x)

void SysExSendNextByte(void* data, uint16_t count)
{
   uint8_t* buffer = (uint8_t*)data;


   switch(count)
   {
      case 0:
         MIDI_Tx(MIDI_SYSEX_START);
         break;
      case 1:
         MIDI_Tx(MIDI_MANUFACTURER);
         break;
      case 2:
         MIDI_Tx(MIDI_DEVICE_CODE);
         break;
      case SEND_SYSEX_STOP:
         MIDI_Tx(MIDI_SYSEX_STOP);
         break;
      default:
         if( ((buffer[count-3]) & 0x80) )
         {
            MIDI_Tx(1);
         }
         else
         {
            MIDI_Tx(0);
         }
         MIDI_Tx( (buffer[count-3]) & 0x7F );
         break;
   }

}


uint8_t IsReceivingSysExData(uint8_t state)
{  
   static uint8_t ReceivingSysExData;     
   if( state == GET_SYSEX_DATA )
   {
       return ReceivingSysExData;
   }
   else
   {
      ReceivingSysExData = state;
      return ReceivingSysExData;   
   }   
}


void ParseSysExData(uint8_t nextByte)
{
   //UART_Tx(nextByte);
   uint8_t* profilePtr;
   
   /* data is sent as 2x 7 bit bytes, so the first byte needs to be stored */
   static uint8_t data;
   
   switch (DataCount)
   {
      case 0:
         if( nextByte != MIDI_SYSEX_START )
         {
            return;
         }

      break;   
      
      case 1:
         if( nextByte != MIDI_MANUFACTURER )
         {
            SysEx_ReceiveError(BAD_MANUFACTURER_ID);
            return;
         }   
      break;
         
      case 2:
         if( nextByte != MIDI_DEVICE_CODE )
         {
            SysEx_ReceiveError(BAD_DEVICE_ID);
            return;
         }
      break;

      default:
         
      break;

   }
   
   profilePtr = (uint8_t*)&CurrentProfile;
   
   if( (DataCount > (sizeof(Profile_t)*2) + 3) )
   {
      SysEx_ReceiveError(SYSEX_TOO_LARGE);
      return;
   }

   if( DataCount >= 3 )
   {
      
      /* On successful download */
      if( (nextByte == MIDI_SYSEX_STOP) )
      {
         
         if(  DataCount == ((sizeof(Profile_t)*2) + 3) )
         {
            uint8_t i;
            char outputString[4];
            
              
            primaryMenu.MenuPrint_P( PSTR("Download Successful!"));
      
            utoa( DataCount, outputString, 10);
      
            primaryMenu.MenuNewLine();
            
            primaryMenu.MenuPrint(outputString);
      
            /* Implement the changes */
         	MIDI_SetBaud(MIDI_GetBaud());
         	MIDI_SetChannelCode( MIDI_GetChannelCode() );
         
         	/* Update the sensor select */
         	SensorInputSelect(GetSensorInput());
         	
         	/* Update Activated Analogue Channels */
            UpdateActiveChannels();
            /* Update the Retrigger periods */
            UpdateChannelRetriggers();
            
      
            for( i = 0; i < 5; i++ )
         	{	
         		_delay_ms(200);
         	}
         	
         	/* To exit the Receive SysEx Sub-menu */
   	      SysExFlush(); 
         	Menu_UpOneLevel(&primaryMenu);
       		MenuUpdate(&primaryMenu, RESET_MENU);
       		return;
         }
         else
         {
            SysEx_ReceiveError(SYSEX_TOO_SMALL);
            return;
         }
      }

      
      /* Check if Datacount is odd */    
      if( !((DataCount - 3) & 0x01) )
      {
         if( nextByte )
         {
            data |= 0x80;
         }
      }
      else
      {
         data |= nextByte;
         profilePtr[(DataCount - 3) >> 1] = data;
         data = 0;
      }
   }   
   DataCount++;
}

void SysEx_ReceiveError(uint8_t errorCode)
{
   uint8_t i;
   uint8_t outputString[3];
   

 
   primaryMenu.MenuPrint_P( PSTR("Read Error: ("));
   utoa(errorCode, outputString, 10);
   primaryMenu.MenuPrint(outputString);
   primaryMenu.MenuPrint(")");
   primaryMenu.MenuNewLine();
   primaryMenu.MenuPrint_P( PSTR("Count:"));
   utoa(DataCount, outputString, 10);
   primaryMenu.MenuPrint(outputString);

   SysExFlush();
   IsReceivingSysExData(SYSEX_DATA_ERROR);
   
   for( i = 0; i < 5; i++ )
	{	
		_delay_ms(200);
	}
	
	/* To exit the Receive SysEx Sub-menu */
	
	Menu_UpOneLevel(&primaryMenu);
	MenuUpdate(&primaryMenu, RESET_MENU);
     
}

void SysExFlush(void)
{
   DataCount = 0;
   IsReceivingSysExData(!RECEIVING_SYSEX_DATA);
   ActiveProcess = DEFAULT_PROCESS;
   SoftTimer_TimerInit();
}















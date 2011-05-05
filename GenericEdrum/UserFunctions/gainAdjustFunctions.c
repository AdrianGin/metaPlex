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
#include "Menu/Menu.h"
#include "UI_KP/UI_KP.h"

#include "Sample/analogueSample.h"

#include "TimerCallbacks/TimerCallbacks.h"
#include "userFunctions/userFunctions.h"
#include "MenuSettings.h"
#include "LCDSettings.h"

#include "gainAdjustFunctions.h"

void GainAdjustFunction(void* data)
{
   uint8_t* input;
   input = data;

   if( ActiveMenu->firstEnter == 0 )
   {
      UF_stateMachine();
      switch( *input )
      {
        case KP_UP:
        case KP_DOWN:
           UF_MenuSetInput(*input );
        break;

         case KB_BACK:
         case KP_BACK:
           SoftTimerStop(SoftTimer2[SC_AutoMenuUpdate]);
           UF_MenuUpOneLevel(&analogueMenu);
           return;

         case KP_ENTER:
           ActiveMenu->firstEnter = 1;
           UF_MenuReset();
           UF_MenuSetInput(*input);
           /* We need to explicitly switch the state since
            the submenu will need to parse the input, since the
            head function has a function (HandleSubMenu), the MenuUpdate
            will not switch the state for us.
            */
           UF_stateMachine();
           MenuUpdate(&analogueMenu, 0);
           return;

         default:
            break;
      }
   }


   ActiveMenu->firstEnter = 0;
   SoftTimer2[SC_AutoMenuUpdate].timeCompare = SLOW_AUTO_MENU_UPDATE;
   PrintGainInformation(ActiveMenu->selectedItem);
}

void GainSlopeAdjustFunction(void* data)
{
   uint8_t* input = data;

   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);


   switch( *input )
   {
      /* Up and down a Key */
      case KP_UP:
         if( ActiveMenu->currentState == ST_GAIN_SLOPE1)
         {
            SetChannelGain(SelectedChannel, GetChannelGain(SelectedChannel)+1);
         }
         else
         {
            SetSlope2Gain(SelectedChannel, GetSlope2Gain(SelectedChannel)+1);
         }
      break;

      case KP_DOWN:
         if( ActiveMenu->currentState == ST_GAIN_SLOPE1)
         {
            SetChannelGain(SelectedChannel, GetChannelGain(SelectedChannel)-1);
         }
         else
         {
            SetSlope2Gain(SelectedChannel, GetSlope2Gain(SelectedChannel)-1);
         }
      break;

      case KP_BACK:
         UF_MenuSetInput(*input);
         UF_stateMachine();
         MenuUpdate(&analogueMenu, 0);
         return;
   }

   PrintGainInformation(ActiveMenu->currentState - ST_GAIN_TYPE);
}


void GainTypeAdjustFunction(void* data)
{

   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   GainTypeToggle(SelectedChannel);

   UF_MenuSetInput(KP_BACK);
   UF_stateMachine();
   MenuUpdate(&analogueMenu, 0);

}




void GainCrossOverAdjustFunction(void* data)
{
   uint8_t* input = data;

   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   switch( *input )
   {
      case KP_UP:
         SetCrossover(SelectedChannel, (int16_t)GetCrossover(SelectedChannel)+10);
      break;

      case KP_DOWN:
         SetCrossover(SelectedChannel, (int16_t)GetCrossover(SelectedChannel)-10);
      break;

      case KP_BACK:
         UF_MenuSetInput(*input);
         UF_stateMachine();
         MenuUpdate(&analogueMenu, 0);
         return;
   }

   PrintGainInformation(ActiveMenu->currentState - ST_GAIN_TYPE);
}

void GainPresetAdjustFunction(void* data)
{
   uint8_t* input = data;
   uint8_t presetSetting;

   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   switch( *input )
   {
      case KP_UP:
         presetSetting = setPresetSetting(GET) + 1;
         if( presetSetting >= NUMBER_OF_GAIN_PRESETS )
         {
            presetSetting = 0;
         }
      break;

      case KP_DOWN:
         presetSetting = setPresetSetting(GET) - 1;
         if( presetSetting == 0xFF )
         {
            presetSetting = NUMBER_OF_GAIN_PRESETS - 1;
         }
      break;

      case KP_BACK:
         UF_MenuSetInput(*input);
         UF_stateMachine();
         MenuUpdate(&analogueMenu, 0);
         return;
   }

   setPresetSetting(presetSetting);
   PrintGainInformation(ActiveMenu->currentState - ST_GAIN_TYPE);
}

uint8_t setPresetSetting(uint8_t newPreset)
{
   static uint8_t presetSetting;

   if( newPreset < NUMBER_OF_GAIN_PRESETS)
   {
      presetSetting = newPreset;
   }
   return presetSetting;
}


void PrintGainInformation(uint8_t UpDownPosition)
{
   static uint8_t enterCount;

   enterCount ^= (1);

   uint8_t presetSetting = setPresetSetting(GET);

   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   if( GetGainType(SelectedChannel) == LINEAR_GAIN )
	{
		presetSetting = CUSTOM;
	}

   if(presetSetting != CUSTOM)
   {
      SetChannelGain(SelectedChannel, PresetGain1[presetSetting]);
      SetSlope2Gain(SelectedChannel, PresetGain2[presetSetting]);
      SetCrossover(SelectedChannel, PresetGainCrossover[presetSetting]);
   }
   UF_MenuReset();
   UF_MenuPrint_P(PSTR("Gain Type:"));
   if( UpDownPosition == 0 && enterCount)
   {
      UF_MenuPrint_P( PSTR("          ") );
   }
   else
   {
      if( GetGainType(SelectedChannel) == LINEAR_GAIN )
      {
         UF_MenuPrint_P( PSTR("Linear") );
      }
      else
      {
         UF_MenuPrint_P( PSTR("Non-Linear") );
      }
   }
	UF_MenuNewLine();


   /* Display the first slope channel 'gain' */
   UF_MenuPrint_P(PSTR("Gain1:"));
   if( UpDownPosition == 1 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
      itoa( (int8_t)(GetChannelGain(SelectedChannel) - GAIN_OFFSET) , outputString, 10);
      UF_MenuPrint(outputString);
   }

   MenuLCD_SetPos(1, 9);
   /* Display the channel slope 2 'gain' */
   UF_MenuPrint_P(PSTR(" Gain2:"));
   if( UpDownPosition == 2 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
      itoa( (int8_t)(GetSlope2Gain(SelectedChannel) - GAIN_OFFSET), outputString, 10);
      UF_MenuPrint(outputString);
   }
   UF_MenuNewLine();

   UF_MenuPrint_P(PSTR("Gain Crossover:"));
   if( UpDownPosition == 3 && enterCount)
   {
		/* Display the gain crossover point */
      UF_MenuPrint_P( PSTR("    ") );
   }
   else
   {
      itoa(GetCrossover(SelectedChannel), outputString, 10);
      UF_MenuPrint(outputString);
   }
	UF_MenuNewLine();

	UF_MenuPrint_P(PSTR("Preset:"));
   if( UpDownPosition == 4 && enterCount)
   {
      UF_MenuPrint_P( PSTR("             ") );
   }
   else
   {
      UF_MenuPrint_P(PresetGainStrings[presetSetting]);
   }
   UF_MenuNewLine();
}




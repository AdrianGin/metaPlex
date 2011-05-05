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

#include "TimerCallbacks/TimerCallbacks.h"
#include "userFunctions/userFunctions.h"
#include "Sample/digitalSample.h"
#include "MenuSettings.h"
#include "LCDSettings.h"
#include "switchTypeAdjustFunctions.h"


void SwitchTypeAdjustFunction(void* data)
{
   uint8_t* input;
   uint8_t SelectedDigitalChannel = SelectedChannel - ANALOGUE_INPUTS;

   input = data;

   if( SelectedDigitalChannel >= DIGITAL_INPUTS )
   {
      UF_MenuPrint_P(PSTR("Metronome inputs are"));
      UF_MenuNewLine();
      UF_MenuPrint_P(PSTR("fixed at:"));
      UF_MenuNewLine();
      UF_MenuPrint_P(PSTR("Negative Polarity"));
      UF_MenuNewLine();
      UF_MenuPrint_P(PSTR("Continuous"));

      delayWithUSBPoll(15, 0);

      UF_MenuUpOneLevel(ActiveMenu);
      return;
   }

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
           UF_MenuUpOneLevel(ActiveMenu);
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
           MenuUpdate(ActiveMenu, 0);
           return;

         default:
            break;
      }
   }


   ActiveMenu->firstEnter = 0;
   SoftTimer2[SC_AutoMenuUpdate].timeCompare = SLOW_AUTO_MENU_UPDATE;
   PrintSwitchTypeInformation(ActiveMenu->selectedItem);
}


void SwitchPolarityAdjustFunction(void* data)
{
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);
   ActivePolarityToggle(SelectedChannel-ANALOGUE_INPUTS);
   UF_MenuSetInput(KP_BACK);
   UF_stateMachine();
   MenuUpdate(ActiveMenu, 0);
}

void SwitchModeAdjustFunction(void* data)
{
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);
   TriggerModeToggle(SelectedChannel-ANALOGUE_INPUTS);
   UF_MenuSetInput(KP_BACK);
   UF_stateMachine();
   MenuUpdate(ActiveMenu, 0);
}
void SwitchReleaseAdjustFunction(void* data)
{
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);
   ActiveReleaseToggle(SelectedChannel-ANALOGUE_INPUTS);
   UF_MenuSetInput(KP_BACK);
   UF_stateMachine();
   MenuUpdate(ActiveMenu, 0);
}

void PrintSwitchTypeInformation(uint8_t UpDownPosition)
{
   static uint8_t enterCount;
   uint8_t SelectedDigitalChannel = SelectedChannel-ANALOGUE_INPUTS;

   enterCount ^= (1);
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   UF_MenuReset();
   UF_MenuPrint_P(PSTR("Polarity: "));
   if( UpDownPosition == 0 && enterCount)
   {
      UF_MenuPrint_P( PSTR("        ") );
   }
   else
   {
      if( GetActivePolarity(SelectedDigitalChannel) == ACTIVE_HIGH )
      {
         UF_MenuPrint_P( PSTR("Positive") );
      }
      else
      {
         UF_MenuPrint_P( PSTR("Negative") );
      }
   }
	UF_MenuNewLine();


   /* Trigger Mode */
   UF_MenuPrint_P(PSTR("Mode: "));
   if( UpDownPosition == 1 && enterCount)
   {
      UF_MenuPrint_P( PSTR("           ") );
   }
   else
   {
      if( GetTriggerMode(SelectedDigitalChannel) == SINGLE_SHOT )
      {
         UF_MenuPrint_P(PSTR("Single Shot"));
      }
      else
      {
         UF_MenuPrint_P(PSTR("Continuous"));
      }
   }
   UF_MenuNewLine();

   UF_MenuPrint_P(PSTR("Active Release: "));
   if( UpDownPosition == 2 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
      if( GetActiveRelease(SelectedDigitalChannel) == ACTIVE_RELEASE_ENABLED )
      {
         UF_MenuPrint_P( PSTR("On") );
      }
      else
      {
         UF_MenuPrint_P( PSTR("Off") );
      }
   }
   UF_MenuNewLine();
}




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
#include "MenuSettings.h"
#include "LCDSettings.h"

#include "dualTriggerAdjustFunctions.h"





void DualTriggerToggle(void* data)
{
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);
   DualModeToggle(SelectedChannel);
   UF_MenuUpOneLevel(&dualTrigMenu);
   return;
}

void DualTriggerNoteAdjust(void* data)
{
   uint8_t* input = data;
   uint8_t keyType = GetState(ActiveMenu) - ST_SET_OPEN_NOTE;

   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   NoteEditor(data, keyType);

   switch( *input )
   {
      case KP_BACK:
         UF_MenuUpOneLevel(ActiveMenu);
      return;
   }

   MenuUpdate(ActiveMenu, RESET_MENU | NO_EXECUTE);
   UF_MenuSetInput(KP_INVALID);

   /* print the open or closed key */
   PrintDualTriggerInformation(keyType + 1);
}

void DualTriggerControlAdjust(void* data)
{
   uint8_t* input = data;
   SoftTimer2[SC_AutoMenuUpdate].timeCompare = FAST_AUTO_MENU_UPDATE;
   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);

   switch( *input )
   {
      case KP_UP:
         SetTrigger(SelectedChannel, GetTrigger(SelectedChannel)+1);
      break;

      case KP_DOWN:
         SetTrigger(SelectedChannel, GetTrigger(SelectedChannel)-1);
      break;

      case KP_BACK:
         UF_MenuUpOneLevel(ActiveMenu);
      return;
   }

   MenuUpdate(ActiveMenu, RESET_MENU | NO_EXECUTE);
   UF_MenuSetInput(KP_INVALID);
   PrintDualTriggerInformation(3);
}


void PrintDualTriggerInformation(uint8_t UpDownPosition)
{
   static uint8_t enterCount;
   enterCount ^= (1);

   UF_MenuReset();

   /* Indicate the channel selected */
   UF_MenuPrint_P(PSTR("CH:"));
   utoa(SelectedChannel + 1, outputString, 10);
   UF_MenuPrint(outputString);

   UF_MenuPrint_P(PSTR(" Dual Trig:"));
   if( UpDownPosition == 0 && enterCount)
   {
      UF_MenuPrint_P( PSTR("   ") );
   }
   else
   {
      if( GetDualMode(SelectedChannel) == HAS_DUAL_INPUT )
      {
         UF_MenuPrint_P( PSTR("On") );
      }
      else
      {
         UF_MenuPrint_P( PSTR("Off") );
      }
   }
	UF_MenuNewLine();


   /* Display the open note */
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

   /* Display the closed note */
   UF_MenuPrint_P(PSTR("C.Note:"));
   if( UpDownPosition == 2 && enterCount)
   {
      UF_MenuPrint_P( PSTR("             ") );
   }
   else
   {
      PrintNoteFormat(CLOSED_KEY);
   }

   UF_MenuNewLine();

   UF_MenuPrint_P(PSTR("Activated by: CH"));
   if( UpDownPosition == 3 && enterCount)
   {
      UF_MenuPrint_P( PSTR("    ") );
   }
   else
   {
      utoa(GetTrigger(SelectedChannel)+1, outputString, 10);
      UF_MenuPrint(outputString);
   }

   SoftTimerStart(SoftTimer2[SC_AutoMenuUpdate]);
   SelectedSubMenu = &dualTrigMenu;


}




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

#include "hardwareSpecific.h"
#include "sample/sample.h"
#include "VUMeter/vumeter.h"
#include "MIDI/midi.h"
#include "Sample/sample.h"
#include "Sample/digitalSample.h"
#include "Sample/analogueSample.h"
#include "Menu/Menu.h"
#include "EDrum.h"
#include "MenuSettings.h"
#include "UserFunctions/userFunctions.h"
#include "UI_KP/UI_KP.h"

#include "LCDSettings.h"
#include "edrumAVRsharedfunctions.h"
#include "TimerCallbacks/TimerCallbacks.h"


/* These are the critical timers, 10ms resolution */
volatile SoftTimer_8  SoftTimer1[TIMER1_COUNT] = { {1, 0, 0, Callback_MIDIScan}, // MIDI Output
                                                   {1, 0, 0, usbPoll} };   



/* These are non-critical timers, and dont really run during 'play time',
   Functions which start these need to ensure they stop them for optimisation
   Use a 10ms timer base*/
volatile SoftTimer_16  SoftTimer2[TIMER2_COUNT] = { {100, 0, 0, Callback_OneSecond},
                                           {DEFAULT_AUTO_MENU_UPDATE, 0, 0, Callback_AutoMenuUpdate},  // Threshold Bar
                                           {7, 0, 0, Callback_VUMeterUpdate},  // VU Meter Update
                                           {7, 0, 0, Callback_DigitalVUUpdate},  // Digital VU Meter Update
                                           {3, 0, 0, Callback_VUDecay},  // VU Decay
                                           {400,400,0, Callback_AboutUpdate}, // AboutUpdate
                                           {1000, 1000, 0, Callback_LCDBacklight}, //LCD Backlight
                                           {15, 0, 0, Callback_MonitorChannel},
                                           {KP_WAIT, 0, 0, Callback_Keypress}};



void Callback_MIDIScan(void)
{
   /* Retriggers need to be reset before outputting MIDI Data */
   uint8_t i;      
   /* Each increment of Retrigger increases the time by 1ms */
   for( i = 0; i < NUMBER_OF_INPUTS ; i++ )
   {
      if(SoftTimerInterrupt(RetriggerPeriod[i]))
      {  
         /* Reset the value here */
         SignalPeak[i] = 0;
         SoftTimerStop(RetriggerPeriod[i]);
         SoftTimerReset(RetriggerPeriod[i]);      
      }
   }

   /* Benchmark reporting */
#if SET_BENCHMARK
   UART_Tx( (uint8_t)(BenchMarkCount>>8) );
   UART_Tx( (uint8_t)(BenchMarkCount) );
   BenchMarkCount = 0;
#else
   /* Update the Digital States */
   ScanDigitalInputs();
   MIDI_DigitalOutput();
#endif
}


void Callback_OneSecond(void)
{
   SoftTimerStop(SoftTimer2[SC_OneSecond]);
}


void Callback_AutoMenuUpdate(void)
{
   /* Update the Threshold and Retrigger bar */
   MenuSetInput(ActiveMenu, MENU_UPDATE);
   MenuUpdate(ActiveMenu, 0);
}

void Callback_VUDecay(void)
{
   /* Decay the VU Meters here */
   if( ActiveProcess == PLAY_MODE )
   {
      VULevelDecay(ALL_METERS);
   }
}


void Callback_LCDBacklight(void)
{

   SoftTimerStop( SoftTimer2[SC_LCD_BL_Period] );
   //UI_LCD_BL_Off();
   
   /* If no SoftTimer2's are enabled, then turn off the Timer2 module, Timer2 to be enabled
    * only after a keypress. */
   if( !SoftTimer_IsTimer2Active() )
   {
      /* Stop the Auxuliary Timer */
      DISABLE_AUXILIARY_TIMER();
   }
}




void Callback_VUMeterUpdate(void)
{
/* Do the VU Meter*/
   uint16_t i;
   uint8_t  VURows = GetVURows();
                          
   for( i = 0 ; i < ANALOGUE_INPUTS; i++ )
   {
      if( GetChannelStatus(i) )
      {
         if( VUValues[i] > GetChannelThresh(i) )
         {
            uint16_t conditionedSignal = (VUValues[i] - GetChannelThresh(i));
            
            conditionedSignal = GainFunction(i, conditionedSignal);
            
            /* Normalise with x rows */
            VUSetLevel(i, VUNormalise(conditionedSignal, MIDI_MAX_DATA, VURows), VURows);            
         }
      }
   }
   VUMeterPrint(SEQUENTIAL_METERS | 0x0F, VURows);
   ResetVUValues();
}



/* Digital & Metronome VU Meter  */
void Callback_DigitalVUUpdate(void)
{
   /* Do the VU Meter*/
   uint16_t i;
   uint8_t  VURows = GetVURows();
                          
   for( i = 0 ; i < DIGITAL_INPUTS+METRONOME_INPUTS; i++ )
   {
      uint8_t ActualChannel = i + ANALOGUE_INPUTS;
      
      if( GetChannelStatus(ActualChannel) )
      {   
         if( VUValues[i] )
         {
            VUSetLevel(i, VUNormalise(GetDigitalVelocity(i), MIDI_MAX_DATA, VURows), VURows);
         }
      }
   }
   VUMeterPrint(SEQUENTIAL_METERS | 0x0F, VURows);
   ResetVUValues();
}

   
   

void Callback_AboutUpdate(void)
{
   uint8_t nameIndex = 0;
   nameIndex = ThanksIndex(GET);
   if( ++nameIndex == SIZEOFABOUT )
   {
      nameIndex = ThanksIndex(MAIN_SCREEN);   
   }
   ThanksIndex(nameIndex);
   aboutScroll(nameIndex);
}



void Callback_MonitorChannel(void)
{
   uint16_t adcValue = SignalPeak[SelectedChannel];
   
   /* Let this function control the retriggers and signal peaks */
   Callback_MIDIScan();

   UF_MenuReset();
   char outputString[10];
   itoa(GetLastMIDIValue(SelectedChannel), outputString, 10);
   UF_MenuPrint_P(PSTR("Last MIDI Code:"));
   UF_MenuPrint(outputString);
   UF_MenuNewLine();

   itoa(ADC_RESOLUTION, outputString, 10);
   UF_MenuPrint(outputString);
   UF_MenuPrint_P(PSTR("bit "));

   itoa(adcValue, outputString, 10);
   UF_MenuPrint_P(PSTR("ADC Code:"));
   UF_MenuPrint(outputString);

   UF_MenuNewLine();
   /* Display the first slope channel 'gain' */
   itoa( (int8_t)(GetChannelGain(SelectedChannel) - GAIN_OFFSET) , outputString, 10);
   UF_MenuPrint_P(PSTR("Gain1:"));
   UF_MenuPrint(outputString);

   /* Display the channel slope 2 'gain' */
   itoa( (int8_t)(GetSlope2Gain(SelectedChannel) - GAIN_OFFSET), outputString, 10);
   UF_MenuPrint_P(PSTR(" Gain2:"));
   UF_MenuPrint(outputString);
   UF_MenuNewLine();   

   /* Display the gain crossover point */
   itoa(GetCrossover(SelectedChannel), outputString, 10);
   UF_MenuPrint_P(PSTR("Gain Crossover:"));
   UF_MenuPrint(outputString);
   UF_MenuNewLine();
}



void Callback_Keypress(void)
{
   static uint8_t result = KP_INVALID;
   static uint8_t pressCount;
   uint8_t i;

   DISABLE_KEYPAD();

   if( result == KP_INVALID )
   {
      result = UI_KP_GetPress();
      /* This will generate an interrupt flag, so we need to clear the flag */
      CLEAR_KEYPAD();
   }

   SoftTimerStop(SoftTimer2[SC_Keypress]);
   if( (result != KP_INVALID) && SoftTimer_IsTimer2Active() )
   {
      /* We have (50ms to stop everything we're doing..) */
      
      for( i = 0; i < TIMER2_COUNT; i++)
      {
         if( SoftTimerIsEnabled(SoftTimer2[i]) )
         {
            SoftTimerStop(SoftTimer2[i]);
         }
      }
      SoftTimer2[SC_Keypress].timeCompare = KP_WAIT;
      SoftTimerStart(SoftTimer2[SC_Keypress]);
      return;
   }

   if( result != KP_INVALID )
   {
      MenuSetInput(ActiveMenu, result);
      MenuUpdate(ActiveMenu, RESET_MENU);
      /* Active the back light */
      //UI_LCD_BL_On();
      //SoftTimerStart(SoftTimer2[SC_LCD_BL_Period]);
      /* Generate our own timer interrupt, for the repeat */
      if( UI_KP_GetState() != KP_NO_REPEAT )
      {
         SoftTimer2[SC_Keypress].timeCompare = KP_REPEAT_RATE;
         pressCount++;
         if( pressCount > 25 )
         {
            SoftTimer2[SC_Keypress].timeCompare = KP_REPEAT_RATE_FAST ;
         }
         if( pressCount > 200 )
         {
            SoftTimer2[SC_Keypress].timeCompare = KP_REPEAT_RATE_FASTEST;
            pressCount--;
         }
      }
      else
      {
         SoftTimer2[SC_Keypress].timeCompare = KP_REPEAT_DELAY;
         UI_KP_SetState(KP_REPEATING);
         ENABLE_KEYPAD();
      }
      /* Take another sample after a little delay */
      result = KP_INVALID;
      SoftTimerStart(SoftTimer2[SC_Keypress]);
   }
   else
   {
      /* Only enable the keypad once the user has released the key */
      pressCount = 0;
      ENABLE_KEYPAD();
      
   }
}


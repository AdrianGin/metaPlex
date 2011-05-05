
#include <stdint.h>
#include "UI_KP/UI_KP.h"
#include "hardwareSpecific.h"

static const uint8_t KP_ButtonIndex[] = {RAW_KP_0, RAW_KP_1, RAW_KP_2, RAW_KP_3, 
                                  RAW_KP_4, RAW_KP_5, RAW_KP_6, RAW_KP_7, 
                                  RAW_KP_8, RAW_KP_9, RAW_KP_A, RAW_KP_B,
                                  RAW_KP_C, RAW_KP_D, RAW_KP_STAR, RAW_KP_HASH};



void UI_KP_Init(void)
{  

   /* Set the rows are inputs FIRST! */
   UI_ROW_DIR &= ~(UI_ROWS);
   UI_ROW_OUT |=  (UI_ROWS);

   /* Columns as outputs */  
   UI_COL_DIR |= (UI_COLS);
   UI_COL_OUT &= ~(UI_COLS);


}

uint8_t UI_KP_GetPress(void)
{
   uint8_t ColResult;
   uint8_t KPResult;

   /* Set Columns to inputs FIRST! */
   UI_COL_DIR &= ~(UI_COLS);
   UI_COL_OUT |= (UI_COLS);
   
   /* Set the Rows to Outputs */
   UI_ROW_DIR |= (UI_ROWS);
   UI_ROW_OUT &= ~(UI_ROWS);  

   _delay_us(50);

   ColResult = (UI_COL_IN & UI_COLS);

   KPResult = (ColResult);
   /* Reset the buttons to original state */
   UI_KP_Init();
   
   return UI_KP_ReturnID(KPResult);
   
}



uint8_t UI_KP_ReturnID(uint8_t KP_Button)
{
	uint8_t i;
	
	for( i = 0; i < KP_ButtonCount; i++)
	{
		if( KP_Button == KP_ButtonIndex[i] )
		{
			return i;	
		}
	}
	
	return KP_INVALID;
	
}

uint8_t UI_KP_SetState(uint8_t newState)
{
   static uint8_t keypadState;

   if( newState == KP_GET_STATE)
   {
      return keypadState;
   }
   
   keypadState = newState;
   return keypadState;
}







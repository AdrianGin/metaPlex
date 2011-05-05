/* Project specific file for LCD Settings */
#include <stdint.h>
#include "UI_LCD/UI_LCD.h"
#include "hardwareSpecific.h"
#include "MSB2LSB/MSB2LSB.h"
#include "LCDSettings.h"




HD44780lcd_t   PrimaryDisplay = 
{
   3, 19, 0, 0, 0, 
   UI_LCD_HWInit, 
   UI_LCD_SetRegister, 
   UI_LCD_Strobe, 
   UI_LCD_BL_On, 
   UI_LCD_BL_Off
};


static uint8_t BL_State = !LCD_BL_ON;
static uint8_t Min_BL_State = 0;

const uint8_t LcdCustomChar[][8] =
{
	{0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00}, // 0. 0/5 full progress block
	{0x00, 0x1F, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x00}, // 1. 1/5 full progress block
	{0x00, 0x1F, 0x18, 0x18, 0x18, 0x18, 0x1F, 0x00}, // 2. 2/5 full progress block
	{0x00, 0x1F, 0x1C, 0x1C, 0x1C, 0x1C, 0x1F, 0x00}, // 3. 3/5 full progress block
	{0x00, 0x1F, 0x1E, 0x1E, 0x1E, 0x1E, 0x1F, 0x00}, // 4. 4/5 full progress block
	{0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00}, // 5. 5/5 full progress block
	{0x0A, 0x15, 0x11, 0x0A, 0x04, 0x00, 0x00, 0x00}, // 6. Heart
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}, // 7. Vertical 1/8 progress  
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F}, // 8. 2/8
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F}, // 9. 3/8
	{0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F}, // 10. Vertical 4/8 progress  
	{0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, // 11. 5/8
	{0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, // 12. 6/8 Vertical progress
	{0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, // 13. 7/8 Vertical progress	
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x0A, 0x04}, // 14. Down Arrow
};






void UI_LCD_LoadDefaultChars(void)
{
	/* Load VU Meter and down arrow for the menu*/
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[14], 0);	
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[7], 1);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[8], 2);
   UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[9], 3);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[10], 4);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[11], 5);
	UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[12], 6);
   UI_LCD_LoadCustomChar(&PrimaryDisplay, (uint8_t*)LcdCustomChar[13], 7);		
}

void UI_LCD_Strobe(void)
{  
#if VERSION_CODE == VERSION_WITH_PE	
	UI_LCD_SetRegister(UI_LCD_E, 0x01);   
	UI_LCD_SetRegister(UI_LCD_E, 0x00);    
#endif

#if VERSION_CODE == VERSION_WITHOUT_PE	
	UI_LCD_CONTROL_PORT |= UI_LCD_E;
	UI_LCD_CONTROL_PORT &= ~UI_LCD_E; 
#endif
}



/* Use a wrapper for the UI_MAX7300 interface to ensure LCD_Power is enabled
 * if any write commands are used */
void UI_LCD_SetRegister(uint8_t data)
{

#if VERSION_CODE == VERSION_WITH_PE
   if( reg == UI_LCD_PORT )
   {   
		data = MSB2LSB(data) >> 4;		
      data &= ~(1 << UI_LCD_PRS);
      data |= (1 << UI_LCD_PPOWER) | (UI_LCD_RSStatus << UI_LCD_PRS); 
   }
   UI_SetRegister(reg, data);   
#endif

#if VERSION_CODE == VERSION_WITHOUT_PE	
	data = MSB2LSB(data) >> 4;
	UI_LCD_CONTROL_PORT &= ~(UI_LCD_RS);
	UI_LCD_CONTROL_PORT |= (PrimaryDisplay.RSStatus << UI_LCD_PRS);
	 
   UI_LCD_DATA_PORT &= ~(UI_LCD_DATA);
	UI_LCD_DATA_PORT |= data;  
#endif   
}


/* Setup all of the UI_MAX7300 LCD Pins as an Output */
void UI_LCD_HWInit(void)
{
#if VERSION_CODE == VERSION_WITH_PE
   uint8_t LCD_INIT[] = {MAX7300_DDRA1, 0x55, 0xA5};
   /* All ports are by default inputs with no pullups */      

	/* Set all UI_LCD pins to outputs */
	/* UI_LCD Pins P12 - P15 as Outputs */	
	/* UI_LCD pins P16-P17 to Outputs leave P18&19 as Input*/
   UI_SetBlock(LCD_INIT, 3);

   /* Set UI_LCD pins to to output LOW */
	UI_LCD_SetRegister(UI_LCD_PORT, 0x00);   
#endif

#if VERSION_CODE == VERSION_WITHOUT_PE
	/* Make all pins outputs */
   UI_LCD_DATA_DIR |= (UI_LCD_DATA);
   UI_LCD_CONTROL_DIR |= (UI_LCD_CONTROL);   

	/* Bring all lines low */
	UI_LCD_DATA_PORT &= ~(UI_LCD_DATA);
	UI_LCD_CONTROL_PORT &= ~(UI_LCD_CONTROL);

	/* LCD BL as an output */
	LCD_BL_SEL |= (1 << LCD_BL_PIN);
	LCD_BL_DDR |= (1 << LCD_BL_PIN);
#endif
	
}




/* Back light control don't need to use PWM, use the 16-bit timer for
 * something more useful
 */
void UI_LCD_BL_On(void)
{
   uint8_t i;
   
   
   if( BL_State == LCD_BL_ON)
   {
      return;  
   }
   
   for( i = Min_BL_State ; i < LCD_BL_MAX_BRIGHTNESS >> 2; i ++ )
   {
      _delay_ms(10);
//      TACCR1 = i << 2;
   }
   
//   TACCR1 = LCD_BL_MAX_BRIGHTNESS;
   BL_State = LCD_BL_ON;
   
   //LCD_BL_PORT |= (1 << LCD_BL_PIN);   
}

void UI_LCD_BL_Off(void)
{
   
   uint8_t i;
   
   for( i = LCD_BL_MAX_BRIGHTNESS >> 2 ; i >= Min_BL_State; i -- )
   {
      _delay_ms(10);
//      TACCR1 = i << 2;
   }
   
//   TACCR1 = Min_BL_State;
   BL_State = !LCD_BL_ON;   
   //LCD_BL_PORT &= ~(1 << LCD_BL_PIN);  
}

void UI_LCD_BL_Toggle(void)
{
   LCD_BL_PORT ^= (1 << LCD_BL_PIN);   
}


void UI_LCD_BLInit(uint16_t MinBrightness)
{
   
   Min_BL_State = MinBrightness;
   
   /* Sets up the PWM frequency =  (fclk / LCD_BL_MAX_BRIGHTNESS) */
//   TACCR0 = LCD_BL_MAX_BRIGHTNESS;
   
//   TACTL |= (TASSEL_SMCLK | MC_UPTO_CCR0);
//  TACCTL1 |= (OUTMOD_RESET_SET);
   
}










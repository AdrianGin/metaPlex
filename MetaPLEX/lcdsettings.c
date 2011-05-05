/* Project specific file for LCD Settings */
#include <stdint.h>
#include "UI_LCD/UI_LCD.h"
#include "hardwareSpecific.h"
#include "LCDSettings.h"


#ifdef UI_LCD_8BITMODE
#include "SPI/spi.h"

#else

#include "MSB2LSB/MSB2LSB.h"


#endif

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
   /* Need to disable the SPI for a moment as we clock in the LCD data */
	UI_LCD_CONTROL_PORT &= ~(1 << UI_LCD_RS_PIN);
	UI_LCD_CONTROL_PORT |= (PrimaryDisplay.RSStatus << UI_LCD_RS_PIN);  
	_delay_us(25);
   UI_LCD_CONTROL_PORT |= UI_LCD_E;
   _delay_us(15);
	UI_LCD_CONTROL_PORT &= ~UI_LCD_E; 
}

/* Use a wrapper for the UI_MAX7300 interface to ensure LCD_Power is enabled
 * if any write commands are used */
void UI_LCD_SetRegister(uint8_t data)
{
// 74HC164 Serial -> Parallel Shift Register is to be used.
#ifdef UI_LCD_8BITMODE
   SPI_TxByte(data);
#else
   uint8_t lcdData;
	//data = MSB2LSB(data) >> 4;
   UI_LCD_DATA_PORT &= ~(UI_LCD_DATA);
	

   lcdData =      (((data & (0x08)) ? UI_LCD_D7 : 0)) |
                  (((data & (0x04)) ? UI_LCD_D6 : 0)) |
                  (((data & (0x02)) ? UI_LCD_D5 : 0)) |
                  (((data & (0x01)) ? UI_LCD_D4 : 0));
   
   UI_LCD_DATA_PORT |= lcdData; 
#endif 
}


/* Setup all of the UI_MAX7300 LCD Pins as an Output */
void UI_LCD_HWInit(void)
{
   	/* Make all pins outputs */
   UI_LCD_CONTROL_DIR |= (UI_LCD_CONTROL);   
   UI_LCD_DATA_DIR |= (UI_LCD_DATA);

	/* LCD BL as an output */
	LCD_BL_SEL |= (1 << LCD_BL_PIN);
	LCD_BL_DDR |= (1 << LCD_BL_PIN);

   /* SPI needs to be initialised when using 74HC164*/
   //SPI_Init();
}




/* Back light control don't need to use PWM, use the 16-bit timer for
 * something more useful
 */
void UI_LCD_BL_On(void)
{
   if( BL_State == LCD_BL_ON)
   {
      return;  
   }
   
   BL_State = LCD_BL_ON; 
   LCD_BL_PORT |= (1 << LCD_BL_PIN);  
}

void UI_LCD_BL_Off(void)
{
   BL_State = !LCD_BL_ON;   
   LCD_BL_PORT &= ~(1 << LCD_BL_PIN);  
}

void UI_LCD_BL_Toggle(void)
{
   LCD_BL_PORT ^= (1 << LCD_BL_PIN);   
}


void UI_LCD_BLInit(uint16_t MinBrightness)
{
   Min_BL_State = MinBrightness;
}










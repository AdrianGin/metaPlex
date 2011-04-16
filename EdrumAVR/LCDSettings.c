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

/* Project specific file for LCD Settings */
#include <stdint.h>
#include "UI_LCD/UI_LCD.h"
#include "hardwareSpecific.h"
#include "LCDSettings.h"


#ifdef UI_LCD_8BITMODE
#include "SPI/spi.h"
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
   SPI_PORT &= ~(1 << MOSI);
   SPI_PORT &= ~(1 << SCK);
   SPCR &= ~(1<<SPE);
	SPI_PORT &= ~(1 << UI_LCD_RS_PIN);
	SPI_PORT |= (PrimaryDisplay.RSStatus << UI_LCD_RS_PIN);   
	UI_LCD_CONTROL_PORT |= UI_LCD_E;
	UI_LCD_CONTROL_PORT &= ~UI_LCD_E; 

   SPCR |= (1<<SPE);
}

/* Use a wrapper for the UI_MAX7300 interface to ensure LCD_Power is enabled
 * if any write commands are used */
void UI_LCD_SetRegister(uint8_t data)
{
// 74HC164 Serial -> Parallel Shift Register is to be used.
#ifdef UI_LCD_8BITMODE
   SPI_TxByte(data);
#else
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
	/* Make all pins outputs */
   UI_LCD_CONTROL_DIR |= (UI_LCD_CONTROL);   

	/* Bring all lines low */
	UI_LCD_CONTROL_PORT &= ~(UI_LCD_CONTROL);

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










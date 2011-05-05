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

#include "UI/UI.h"
#include "MAX7300/max7300.h"
#include "UI_LCD.h"
#include "hardwareSpecific.h"


/* Internal Functions */


void UI_LCD_SetInstruction(HD44780lcd_t* lcd)
{
   lcd->RSStatus = UI_LCD_RS_INSTRUCTION;
}
   

void UI_LCD_SetData(HD44780lcd_t* lcd)
{
   lcd->RSStatus = UI_LCD_RS_DATA;  
}


#if VERSION_CODE == VERSION_WITH_PE	
/* Shutdown and Activate only used in special cases (ie. not edrum) */
void UI_LCD_Activate(void)
{
   UI_LCD_PowerStatus = LCD_ON;
   /* Set UI_LCD pins to to output LOW except for LCD Power which is high*/
	UI_SetRegister(UI_LCD_POWER, 0x01);    
   /* Need to wait 40ms after applying power */
   _delay_ms(40);
}

void UI_LCD_Shutdown(void)
{
   /* Set UI_LCD pins to to output LOW */
	UI_SetRegister(UI_LCD_PORT, 0x00);   
   UI_SetRegister(UI_LCD_POWER, 0x00); 
   UI_LCD_PowerStatus = LCD_OFF;
}
#endif



/* Assumes 4-bit Mode, 4 MSB are sent first */
void UI_LCD_Write(HD44780lcd_t* lcd, char code)
{
#ifdef UI_LCD_8BITMODE
	lcd->SetRegister(code);
   lcd->Strobe();
#else
	lcd->SetRegister((code >> 4) & (0x0F) );
   lcd->Strobe();
	lcd->SetRegister((code) & (0x0F) );   
   lcd->Strobe();
#endif
}

void  UI_LCD_Char(HD44780lcd_t* lcd, char data)
{
   UI_LCD_SetData(lcd);
   UI_LCD_Write(lcd, data);
}




/* Clears the screen, sets up the LCD to accept commands */
void UI_LCD_Init(HD44780lcd_t* lcd)
{

#ifdef UI_LCD_8BITMODE
   UI_LCD_SetInstruction(lcd);
   UI_LCD_Write( lcd, LCD_FUNCTION_DEFAULT );
   UI_LCD_Write( lcd, LCD_DISPLAY_DEFAULT );
   UI_LCD_Write( lcd, LCD_MODE_DEFAULT );  
   UI_LCD_Clear(lcd);   
   UI_LCD_Home(lcd);
#else

   UI_LCD_SetInstruction(lcd);
	/* Set to 8 - bit mode */
	lcd->SetRegister(((LCD_FUNCTION_DEFAULT) >> 4) & (0x0F) );
   lcd->Strobe();
   UI_LCD_Write( lcd, LCD_FUNCTION_DEFAULT );  
   UI_LCD_Write( lcd, LCD_DISPLAY_DEFAULT );
   UI_LCD_Write( lcd, LCD_MODE_DEFAULT );  
   UI_LCD_Clear(lcd);
   _delay_ms(2);    
   UI_LCD_Home(lcd);
   _delay_ms(2); 
#endif

}




/* Public Functions */
void UI_LCD_Home(HD44780lcd_t* lcd)
{
   UI_LCD_SetInstruction(lcd);   
   UI_LCD_Write(lcd, (1 << LCD_HOME ) );   
   _delay_ms(20);   
   
   lcd->RowPos = 0;
   lcd->ColPos = 0;
}


void UI_LCD_Clear(HD44780lcd_t* lcd)
{
   UI_LCD_SetInstruction(lcd);   
   UI_LCD_Write(lcd, (1 << LCD_CLR ) );
   _delay_ms(2);
}

/* Prints a string to the LCD at the current position 
 * If the line overflows, the data continues to the next address */
void UI_LCD_String(HD44780lcd_t* lcd, char* string)
{
   UI_LCD_SetData(lcd);
   
   while( *string )
   {
      UI_LCD_Write(lcd, *string++);      
   }
}

/* New Lines are done by using UI_LCD_Pos( row+1, 0); function */

/* Prints a string to the LCD at the current position 
 * If the line overflows, the data continues to the next address */
void UI_LCD_String_P(HD44780lcd_t* lcd, const char* string_P)
{
   uint8_t c;
   UI_LCD_SetData(lcd);
   
   while( (c = UI_LCD_GET_FLASHBYTE(string_P++)) )
   {
      UI_LCD_Write(lcd, c);
   }   
}


/* Max Row is 3 and Max Col is 0 -> 19 (Dec) */
void UI_LCD_Pos(HD44780lcd_t* lcd, uint8_t row, uint8_t col)
{
   uint8_t DDRAMAddr = 0;
   
   UI_LCD_SetInstruction(lcd);   
   lcd->RowPos = row;
	lcd->ColPos = col;
   
   // remap lines into proper order
	switch( row )
	{
	   case 0: 
         DDRAMAddr = LCD_LINE0_DDRAMADDR + col; 
      break;
      
	   case 1: 
         DDRAMAddr = LCD_LINE1_DDRAMADDR + col; 
      break;
         
	   case 2: 
         DDRAMAddr = LCD_LINE2_DDRAMADDR + col; 
         break;
         
	   case 3: 
         DDRAMAddr = LCD_LINE3_DDRAMADDR + col; 
         break;
	   default: 
         //DDRAMAddr = LCD_LINE0_DDRAMADDR + col; 
         //lcd->RowPos = 0;
      break;
	}

	// set data address
	UI_LCD_Write(lcd, 1<<LCD_DDRAM | DDRAMAddr);


}


void UI_LCD_SetCursor(HD44780lcd_t* lcd)
{
	UI_LCD_SetInstruction(lcd);
	
	/** Activates the display, cursor and cursor blink */
	UI_LCD_Write(lcd,  (1 << LCD_DISPLAY) 
					| (1 << LCD_ON_DISPLAY) 
					| (1 << LCD_ON_CURSOR) ); 
}

void UI_LCD_ClearCursor(HD44780lcd_t* lcd)
{
	UI_LCD_SetInstruction(lcd);
	
	/** Activates the display, cursor and cursor blink */
	UI_LCD_Write(lcd, (1 << LCD_DISPLAY) | (1 << LCD_ON_DISPLAY)); 
}
   


/* Accepts a 8 Byte custom char array */
void UI_LCD_LoadCustomChar(HD44780lcd_t* lcd, uint8_t* lcdCustomCharArray, uint8_t lcdCharNum)
{
	uint8_t i;

	// multiply the character index by 8
	lcdCharNum = (lcdCharNum<<3);	// each character occupies 8 bytes

	// copy the 8 bytes into CG (character generator) RAM
   UI_LCD_SetInstruction(lcd);
	// set CG RAM address
	UI_LCD_Write(lcd, (1<<LCD_CGRAM) | (lcdCharNum));	
	
	/* Assumes auto increment of cursor position */
	for(i=0; i<8; i++)
	{	      

		_delay_us(200);
		UI_LCD_SetData(lcd);
		// write character data
		UI_LCD_Write(lcd, *(lcdCustomCharArray++) );
	}

   /* Reset the DDRAM LCD Position */
   /* Basically a faster return home */
   UI_LCD_Pos(lcd, 0,0);

}


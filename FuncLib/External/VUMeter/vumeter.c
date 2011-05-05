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
#include "vumeter.h"
#include "LCDSettings.h"

/* VULevel has 8 bytes, the first 5 are the number of pixels for a 4 row 
 * VUMeter, the upper 3 are used to denote how many rows are used */

static uint8_t VULevel[MAX_METERS];
static uint8_t VURowStart;
static uint8_t VUColStart;

static uint8_t VURow;
static uint8_t VUCol;

static uint8_t VURows = DEFAULT_VU_HEIGHT;

volatile uint16_t	VUValues[MAX_METERS];

void UpdateVUValues(void* src)
{
	memcpy((void*)VUValues, src, sizeof(VUValues));
}

void ResetVUValues(void)
{
	uint8_t i = MAX_METERS;
	while( i-- )
	{
		VUValues[i] = 0;	
	}	
}

/* Accepts the normalised value as well as how
 * many chars it occupies */
void VUSetLevel(uint8_t meterIndex, uint8_t normalValue, uint8_t rows)
{
	/* Ensure the VU Level is within limits */
   if( normalValue <= (rows << 3) )
   {
   	VULevel[meterIndex] = normalValue;
	}
	else
	{
		VULevel[meterIndex] = (rows << 3);	
	}
   
}

/* Normalises the value to rows*8
 * The return value is effectively the number of 
 * pixels to draw if each row has 8 pixels */
 
/* Max value and maxVal values are 2047 */ 
uint16_t VUNormalise(uint16_t value, uint16_t maxVal, uint16_t rows)
{
	
   value = value * (rows << 3);
   value = value / maxVal;
   
   return value;
}


/* Row ranges from 1 -> 7 */
/* T1 prints the VULevels from 0 -> 8 */
/* T2 prints the VULevels from 0 -> 16 */
/* T3 prints the VULevels from 0 -> 24 */
/* T4 prints the VULevels from 0 -> 32 */
/* T5 prints the VULevels from 8 -> 32 */
/* T6 prints the VULevels from 16 -> 32 */
/* T7 prints the VULevels from 24 -> 32 */

/* each VU meter occupies a column, T(x)/8 size in height */

/* Assumes char 0 is 0 pixels, 1 = 1 pixel high ... 8 = 8 high */
/* meterIndex can be ALL_METERS to print all meters */
void VUMeterPrint(uint8_t meterIndex, uint8_t rows )
{

   int8_t stop = 0;
   int8_t row;
   int8_t col;
   
   if( rows > FULL_RANGE )
   {
      stop = rows - FULL_RANGE;
      rows = FULL_RANGE;  
   }
   
   if( meterIndex & SEQUENTIAL_METERS )
   {
		uint8_t printLimit = (meterIndex & 0x0F);
	   VUPosition(VURowStart, VUColStart);
	   VURow = VURowStart;
   	VUCol = VUColStart;
   
      /* For each VUMeter, print out the pixels */
      for( row = rows - 1; row >= stop; row-- )
      {
         int8_t temp;
         
         
		   /* Print out all the required VUMeters */
		   for( col = 0; col <= printLimit; col++ )
		   {
	         temp = (VULevel[col]) - (row << 3);
	         /* Limit the pixels to each row to 0 -> 8 */
	         if( temp <= 0 )
	         {
	            temp = ' ';  
	         }
	         else if( temp >= PIXELS_PER_ROW )
	         {
	            temp = FULL_BLOCK;  
	         }
	         VUPrint(temp);
			}
         VUNewLine();
      }		
		return;
	}
   
   

	VURow = VURowStart;
	/* Move across to the VUMeter and reset position */
	VUPosition(VURowStart, meterIndex);
	
   /* For each VUMeter, print out the pixels */
   for( row = rows - 1; row >= stop; row-- )
   {
      int8_t temp;
      temp = (VULevel[meterIndex]) - (row << 3);
      
      /* Limit the pixels to each row to 0 -> 8 */
      if( temp <= 0 )
      {
         temp = ' ';  
      }
      else if( temp >= PIXELS_PER_ROW )
      {
         temp = FULL_BLOCK;  
      }
      VUPrint(temp);
      VUNewLine();
   }

}


/* Setup the position of the top left char */
void VUSetPosition(uint8_t row, uint8_t column)
{
   VURowStart = row;
   VUColStart = column;
   
   /* Setup the start positions */
   VURow = VURowStart;
   VUCol = VUColStart;   
   
   VUPosition(VURow, VUCol);
}

void VUNewLine(void)
{     
   VUPosition(++VURow, VUCol);
}



/* Decays at a fixed rate of 1 pixel per call */
/* meterIndex can be passed ALL METERS to decay all meters */
void VULevelDecay(uint8_t meterIndex)
{
   
   if( meterIndex == ALL_METERS )
   {
      uint8_t i;
      for( i = 0; i < MAX_METERS; i++ )
      {
         if(VULevel[i])
         {
            VULevel[i] --;  
         }
           
      }
   }
   else
   {
      if(VULevel[meterIndex])
      {
         VULevel[meterIndex] --;
      }
   }
}



void VUSetRows(uint8_t rows)
{
	if( rows > MAX_ROWS )
	{
		rows = 1;	
	}
	VURows = rows;
}

uint8_t GetVURows(void)
{
	return VURows;	
}

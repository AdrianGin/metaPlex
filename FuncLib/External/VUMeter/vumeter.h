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



#ifndef VUMETER_H
#define VUMETER_H


#include "UI_LCD/UI_LCD.h"

#define VUPrint(char) UI_LCD_Char(&PrimaryDisplay, char)
#define VUPosition(row, col) UI_LCD_Pos(&PrimaryDisplay, row, col)


/* To print all columns */
#define SEQUENTIAL_METERS   (0xE0)
#define ALL_METERS (0xFF)
#define FULL_RANGE   (4)
#define PIXELS_PER_ROW  (8)
/* For a 20x4 LCD Module */
#define MAX_METERS  (20)

#define MAX_ROWS	(4)
#define FULL_BLOCK	(0xFF)
#define DEFAULT_VU_HEIGHT (MAX_ROWS - 1)
extern volatile uint16_t	VUValues[];


void VUSetLevel(uint8_t meterIndex, uint8_t normalValue, uint8_t rows);

void UpdateVUValues(void* src);
void ResetVUValues(void);

uint16_t VUNormalise(uint16_t value, uint16_t maxVal, uint16_t rows);

void VUMeterPrint(uint8_t column, uint8_t rows );
void VUSetPosition(uint8_t row, uint8_t column);
void VUNewLine(void);

void VULevelDecay(uint8_t meterIndex);


void VUSetRows(uint8_t rows);
uint8_t GetVURows(void);


#endif

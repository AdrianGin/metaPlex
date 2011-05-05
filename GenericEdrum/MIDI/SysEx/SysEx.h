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



#ifndef	_SYSEX_ROUTINES
#define	_SYSEX_ROUTINES

#include "Profiles\profiles.h"

#define GET_SYSEX_DATA (0xFF)
#define SEND_SYSEX_STOP (0xFFFF)
#define RECEIVING_SYSEX_DATA (1)
#define SYSEX_DATA_ERROR (3)

void SysExSendNextByte(void* data, uint16_t count);
uint8_t IsReceivingSysExData(uint8_t state);
void ParseSysExData(uint8_t nextByte);
void SysExFlush(void);
void SysEx_ReceiveError(void);


#endif

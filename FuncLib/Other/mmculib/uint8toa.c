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

/* File:   uint8toa.c
   Author: M. P. Hayes, UCECE
   Date:   15 May 2007
   Descr: Edited by Adrian Gin
*/

#include <stdint.h>
#include <stdlib.h>
#include "uint8toa.h"

/*

#define uint8toa(num,str)	utoa(num, str, 10)


void uint8toa (uint8_t num, uint8_t *str)
{
   uint8_t d;
   uint8_t i;
   const uint8_t powers[] = {100, 10, 1, 0};

   for (i = 0; (d = powers[i]); i++)
   {
      uint8_t q;
      q = num / d;
      if (q || d == 1)
      {
         *str++ = '0' + q;
         num -= q * d;
      }
   }

   *str = '\0';
}

*/





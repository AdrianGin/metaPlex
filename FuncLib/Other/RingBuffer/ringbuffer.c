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

/* Ring buffer implementation */

#include "ringbuffer.h"


/* Critical means disable interrupts on entry and restore interrupt 
 * state on exit */

int ringbuffer_get(RINGBUFFER_T *buffer) {
    //buffer not empty?
    if (buffer->read_pos != buffer->write_pos) {
        //wrap around read position
        buffer->read_pos &= (buffer->size - 1);
        //get byte from buffer, update read position and return
        return buffer->memory[buffer->read_pos++];
    } else {
        return BUFFER_OVERFLOW; /* This is really trying to get a nonexistant byte */
    }


}


int ringbuffer_put(RINGBUFFER_T *buffer, char character) {
    //is there space in the buffer?
    int bufferMask = (buffer->size - 1);

    if ( ((buffer->write_pos + 1) & bufferMask) != (buffer->read_pos & bufferMask)) {
        //wrap around write position
        buffer->write_pos &= bufferMask;

        //write the character
        buffer->memory[buffer->write_pos++] = character;
        //update size info

        return ringbuffer_len(buffer);
    } else {
        return BUFFER_OVERFLOW;
    }
}

unsigned int ringbuffer_len(RINGBUFFER_T *buffer) {
   
    if( buffer->read_pos > buffer->write_pos )
    {
        return buffer->write_pos + buffer->size - buffer->read_pos;
    }
    return buffer->write_pos - buffer->read_pos;
}

unsigned int ringbuffer_isEmpty(RINGBUFFER_T *buffer) {
   return buffer->write_pos == buffer->read_pos;
}


void ringbuffer_clear(RINGBUFFER_T *buffer) {
    buffer->read_pos = 0;
    buffer->write_pos = 0;
}

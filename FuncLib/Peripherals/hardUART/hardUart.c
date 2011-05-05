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

/* Serial Port routines for the AVR mpu's.
*  These functions utilise the hardware UART registers
*
* Filename: hardUart.c
* By Adrian Gin (amg94@student.canterbury.ac.nz)
* Created: 16-06-06
* 
* For AVR Core
* Since the UART has true ouputs, a MAX232 is required to interface
* with a computer's serial port.
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include	<util/delay.h>
#include	"hardUart.h"
#include "RingBuffer/ringbuffer.h"

#include "mmculib/uint8toa.h"

static char txbuffer[TXBUFFER_SIZE];
RINGBUFFER_T TransmitBuffer = {txbuffer, sizeof(txbuffer)};
static char rxbuffer[RXBUFFER_SIZE];
RINGBUFFER_T ReceiveBuffer = {rxbuffer, sizeof(rxbuffer)};

volatile uint8_t transmitState;


/* uartInit:
 * Initialises the baudrate, parity, stop bit generation and 8bit mode
 * It must be called before any hardUart function is used 
 *
 * Interrupts are not set by default.
 *
 */
void uartInit(AVR_USART_t* port, uint8_t U2Xvalue)
{
	/*Setup the U2X Bit*/
	*port->UCSRxA	=	(*port->UCSRxA & (~(1<<U2X))) | (U2Xvalue << U2X);
	
	*port->UCSRxB |= ((1<<RXEN) | (1<<TXEN) | (1<<RXCIE) | (1<<TXCIE));	/*Enable Rx and Tx modules*/
	*port->UCSRxB &= ~(1<<UCSZ2);				/*Set to 8bit mode*/
	

	/*Select port->UCSRxC to be written to*/	
	/* Set to Asynchronous Mode
	 *			 1 Stop-bit
	 *			 No Parity
	 *			 8-bit char mode
	 */
	*port->UCSRxC = (NOPARITY<<UPM0)
      				|	(BIT8 << UCSZ0) 
      				|  (1<<URSEL);

   /*Enable the pull up on RXD */
   PORTD |= (1 << PD0);

}

/* uartSetBaud:
 * Changes the baudrate to the specified value.
 * See the datasheet for more details on what the
 * Baudrate generation registers should be.
 */
void uartSetBaud(AVR_USART_t* port, uint8_t baudrateH, uint8_t baudrateL)
{
	*port->UBRRxH = 	baudrateH;
	/* The lower 8bits must be written last as the baudrate generator
	 * is updated as soon as UBRRL is written to*/
	*port->UBRRxL	=	baudrateL; 

}


/* Disables the Receiver and Transmitter modules*/
void uartDisable(AVR_USART_t* port)
{
	*port->UCSRxB &= ~((1<<RXEN) | (1<<TXEN) | (1<<TXCIE) | (1<<RXCIE) | (1<<UDRIE));	/*Disable Rx and Tx modules*/
	
}

/* uartTx:
 *
 * Transmits the passed byte to the Uart.Tx pin.
 *
 */
//void uartTx(AVR_USART_t* port, uint8_t outbyte)
//{
	/*Wait until output shift register is empty*/	
//	while( (*port->UCSRxA & (1<<UDRE)) == 0 );
		
	/*Send byte to output buffer*/
//	*UDRx	= outbyte;
//}



void uartTx(AVR_USART_t* port, uint8_t byte)
{
   /* If the buffer is full, then we have to wait until we have to send the data
    * to prevent data loss */
   while(ringbuffer_put((RINGBUFFER_T*)port->TransmitBuffer, byte) == BUFFER_OVERFLOW)
   {
   }

   if( (*port->UCSRxA & (1 << UDRE)) && !ringbuffer_isEmpty((RINGBUFFER_T*)port->TransmitBuffer))
   {
      *port->UDRx = ringbuffer_get((RINGBUFFER_T*)port->TransmitBuffer);
   }
}







/** Writes nbytes of buffer to the UART */
void uartTxDump(AVR_USART_t* port, uint8_t* buffer, uint16_t nbytes )
{
	uint16_t i = 0;
	while( i++ < nbytes )
	{
		uartTx(port, *buffer++);
	}
}

/** Writes nbytes of buffer to the UART */
void uartTxDump_P(AVR_USART_t* port, PGM_P buffer, uint16_t nbytes )
{
	uint16_t i = 0;
	while( i++ < nbytes )
	{
      uint8_t c;
      c = pgm_read_byte(buffer++);
		uartTx(port, c);
	}
}



/* uartTxString:
 * Outputs the passed string to the UART.Tx pin
 * The output is true ouput, not inverted, so a MAX232 or some sort of
 * TTL -> +/- 15V converter is required.
 */
void uartTxString(AVR_USART_t* port, uint8_t* outString)
{
	while( *outString )
	{
		uartTx(port, *outString++);
   }
}

/* Usage: uartTxString_P( PSTR("hello!") ); */
/* Send a string which resides in the program memory */
void uartTxString_P(AVR_USART_t* port, PGM_P outString_P)
{
   char c;
   while( (c = pgm_read_byte(outString_P++)) )
   {
      uartTx(port, c);    
   }
}

void uartNewLine(AVR_USART_t* port)
{
   uartTx(port, '\r');
   uartTx(port, '\n'); 
}

/* To echo the receiver buffer, write this code in the main.c file */
/* Just remember that the Interrupt is enabled 
ISR(SIG_UART_RECV)
{
	uartTx(UDR);
}
*/



/* Once a tx has completed, this is called */
/*
ISR(USART_UDRE_vect)
{
   //return;
   //
   // Tx the next byte if there are still bytes in the buffer
   if( !ringbuffer_isEmpty((RINGBUFFER_T*)PrimaryUART.TransmitBuffer) )
   {
      UDR = ringbuffer_get((RINGBUFFER_T*)PrimaryUART.TransmitBuffer);
   }
   else
   {     
      UCSRB &= ~(1<<UDRIE);
   }
}

ISR(USART_TXC_vect, ISR_NOBLOCK)
{
   // Tx the next byte if there are still bytes in the buffer
   if( !ringbuffer_isEmpty((RINGBUFFER_T*)PrimaryUART.TransmitBuffer) )
   {
      UDR = ringbuffer_get((RINGBUFFER_T*)PrimaryUART.TransmitBuffer);
   }
}

*/

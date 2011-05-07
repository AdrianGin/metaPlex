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
* Filename: harUart.h
* By Adrian Gin (amg94@student.canterbury.ac.nz)
* Created: 16-06-07
* 
* For AVR Core
* Since the UART has true ouputs, a MAX232 is required to interface
* with a computer's serial port.
*
* Addition of support for USART0 and USART1.
*
*/


/** \file harUart.h
	 \brief UART Drivers for the AVR Core.
*/

/**	 

	\ingroup avr_peripheral
 	\defgroup hardUart Hardware UART Routines
 	\code #include "hardUart.h" \endcode

 	 
	 \par Description
	 Functions which provide an interface to the AVR UART
	 component.
 	 
*/
//@{

#ifndef 	_HARDUART_ROUTINES
#define	_HARDUART_ROUTINES


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include "RingBuffer/ringbuffer.h"

#define	SET	1
#define	CLEAR	0

#define	DEFAULT_SPD		8
#define	U2X_BIT_STATUS	SET

#define BAUD(rate) ((((F_CPU) / (rate))/16) - 1)

/** For a processor at 8MHz and U2X = 1
 */
#if  U2X_BIT_STATUS == SET
#define 	BAUD2400		(416)
#define 	BAUD4800		(207)
#define 	BAUD9600		(103)
#define 	BAUD14400	(68)
#define  BAUD19200	(51)
#define 	BAUD28800	(34)
#define	BAUD38400	(25)
#define	BAUD57600	(16)
#define	BAUD76800	(12)
#define	BAUD115200	(8)
#define	BAUD230400	(3) /** High Error */
#define	BAUD250000	(3)
#define	BAUD500000	(1)
#define	BAUD1000000	(0)

/** For a processor at 8MHz and U2X = 0
 */
#else 
 
#define 	BAUD2400		(207)
#define 	BAUD4800		(103)
#define 	BAUD9600		(51)
#define 	BAUD14400	(34)
#define	BAUD19200	(25)
#define 	BAUD28800	(16)
#define	BAUD38400	(12)
#define	BAUD57600	(8)
#define	BAUD76800	(6)
#define	BAUD115200	(3)
#define	BAUD230400	(1) /*High Error */
#define	BAUD250000	(1)
#define	BAUD500000	(0)

#endif 

#define	BAUD_DEFAULT	BAUD9600


#define	PARITY_MASK		(0x30)
#define	NOPARITY			(0x00)
#define	EVEN				(0x02)
#define	ODD				(0x03)

#define	CHARSIZE_MASK	(0x06)
#define 	BIT8				(0x03)
#define	BIT7				(0x02)
#define	BIT6				(0x01)
#define	BIT5				(0x00)

#define	UCSRCMASK		(0x7F)


#ifndef TXEN
#define RXC    7
#define TXC    6
#define UDRE   5
#define FE    4
#define DOR    3
#define UPE    2
#define U2X    1
#define MPCM   0

#define RXCIE  RXCIE1
#define TXCIE  TXCIE1
#define UDRIE  UDRIE1
#define RXEN   RXEN1
#define TXEN   TXEN1
#define UCSZ2  UCSZ12
#define RXB8   RXB81
#define TXB8   TXB81

#define URSEL  UMSEL11
#define UMSEL0 UMSEL10
#define UPM1   UPM11
#define UPM0   UPM10
#define USBS   USBS1
#define UCSZ1  UCSZ11
#define UCSZ0  UCSZ10
#define UCPOL  UCPOL1
#endif


#define IS_TRANSMITTING (1)

/** uartInit:
 * Initialises the baudrate, parity, stop bit generation and 8bit mode
 * It must be called before any hardUart function is used 
 *
 * Interrupts are not set by default.
 *
 */
#define FAST 1
#define SLOW 0

#define  TXBUFFER_SIZE   (16)
#define  RXBUFFER_SIZE   (16)


extern volatile RINGBUFFER_T ReceiveBuffer;
extern volatile RINGBUFFER_T TransmitBuffer;
extern volatile uint8_t transmitState;

typedef struct
{
   volatile uint8_t* UCSRxA;
   volatile uint8_t* UCSRxB;
   volatile uint8_t* UCSRxC;
   volatile uint8_t* UBRRxH;
   volatile uint8_t* UBRRxL;
   volatile uint8_t* UDRx;

   RINGBUFFER_T* ReceiveBuffer;
   RINGBUFFER_T* TransmitBuffer;

} AVR_USART_t;

void uartInit(AVR_USART_t* port, uint8_t U2Xvalue);



/** uartDisable:
 * Disables the Receiver and Transmitter modules*/
void uartDisable(AVR_USART_t* port);



/** uartSetBaud:
 * Changes the baudrate to the specified value.
 * See the datasheet for more details on what the
 * Baudrate generation registers should be.
 */
void uartSetBaud(AVR_USART_t* port, uint8_t baudrateH, uint8_t baudrateL);


/** uartTxString:
 * Outputs the passed string to the UART.Tx pin
 * The output is true ouput, not inverted, so a MAX232 or some sort of
 * TTL -> +/- 15V converter is required.
 */
void uartTxString(AVR_USART_t* port, uint8_t* outString);

void uartTxString_P(AVR_USART_t* port, PGM_P outString_P);

/** uartTx:
 *
 * Transmits the passed byte to the Uart.Tx pin.
 *
 */
//void uartTx(uint8_t outbyte);
void uartTx(AVR_USART_t* port, uint8_t byte); // Use this for interrupt based transmission


/** uartTxDump:
 *  Prints out nbytes of buffer to the UART
 */
void uartTxDump(AVR_USART_t* port, uint8_t* buffer, uint16_t nbytes );

void uartTxDump_P(AVR_USART_t* port, PGM_P buffer, uint16_t nbytes );

/* ISR(SIG_UART_RECV)
 *
 * The interrupt routine for when a receive is complete
 */
//ISR(SIG_UART_RECV);

void uartNewLine(AVR_USART_t* port);

#endif

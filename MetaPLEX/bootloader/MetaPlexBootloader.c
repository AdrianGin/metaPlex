#include <avr/io.h>
#include <string.h>
#include "hardwareSpecific.h"
#include "MetaPlexBootloader.h"
#include "flashmem/lowlevel_flashmem.h"
#include "flashmem/flashmem.h"
#include "firmwareUpdate/firmwareUpdate.h"
#include "USBMIDI/USBMIDI.h"



#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/USB/Class/MIDI.h>


const PROGRAM_CHAR VersionID[] = "USB-MIDI Bootloader V1.0";

//Must be a power of two
#define RX_BUFFER_SIZE (16)
#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1)
/* After 200 times to retry sending a message, we assume the USB is
   disconnected */
#define USB_CONNECT_TIMEOUT (1500)
#define CABLE_NO_COUNT (2)

volatile uint8_t rxReadPtr[CABLE_NO_COUNT];
uint8_t USB_Connected;
volatile uint8_t RxBuffer[CABLE_NO_COUNT][RX_BUFFER_SIZE];
volatile uint8_t rxWritePtr[CABLE_NO_COUNT];


USB_ClassInfo_MIDI_Device_t MIDI_Interface =
	{
		.Config =
			{
				.StreamingInterfaceNumber = 1,

				.DataINEndpointNumber      = MIDI_STREAM_IN_EPNUM,
				.DataINEndpointSize        = MIDI_STREAM_EPSIZE,
				.DataINEndpointDoubleBank  = false,

				.DataOUTEndpointNumber     = MIDI_STREAM_OUT_EPNUM,
				.DataOUTEndpointSize       = MIDI_STREAM_EPSIZE,
				.DataOUTEndpointDoubleBank = false,
			},
	};

usbMIDIcable_t MIDICable[2] = {
      {.lastDataByte = NO_DATA_BYTE},
      {.lastDataByte = NO_DATA_BYTE}
};



/* Firmware Update also via UART */
ISR(USART1_RX_vect)
{
   uint8_t buffer = UDR1;
   //USBMIDI_PutByte(buffer, 1);

   /* Echo this back out */
   RxBuffer[0][rxWritePtr[0]] = buffer;
   rxWritePtr[0] = ((rxWritePtr[0] + 1) & RX_BUFFER_MASK);
}

ISR(BADISR_vect, ISR_NOBLOCK)
{
    // user code here
}


void bootuartTxString_P(PGM_P outString_P)
{
   char c;
   while( (c = pgm_read_byte(outString_P++)) )
   {
      bootuartTx(c);    
   }
}

void bootuartTx(uint8_t outbyte)
{
	/*Wait until output shift register is empty*/	
	while( (UCSR1A & (1<<UDRE)) == 0 );
	/*Send byte to output buffer*/
	UDR1	= outbyte;
}

 
/*---------------------------------------------------------------------------*/
/* usbFunctionWriteOut                                                       */
/*                                                                           */
/* this Function is called if a MIDI Out message (from PC) arrives.          */
/*                                                                           */
/*---------------------------------------------------------------------------*/
 
 
void usbFunctionWriteOut(uint8_t * data, uint8_t len)
{

}
 

/* This reads the MIDI data received from USB */
uint8_t USBMIDI_GetByte(uint8_t* inByte, uint8_t cableNo)
{
   /* Process messages in the UART Rx buffer is there are any */
   if( rxReadPtr[cableNo] != rxWritePtr[cableNo] )
   {
      *inByte = RxBuffer[cableNo][rxReadPtr[cableNo]];
      rxReadPtr[cableNo] = ((rxReadPtr[cableNo] + 1) & RX_BUFFER_MASK);
      return 1;
   }

   return NO_DATA_BYTE;
}


/* This here makes the process Buffer redundant */
void USBMIDI_PutByte(uint8_t byte, uint8_t cableNo)
{
   uint8_t midiReady;
   uint16_t retry;


   /* Only process this stuff if we have enough room in the MIDI out buffer, we must block otherwise */
   if( !usbMIDI_bufferIsReady() )
   {
      /* Data will be lost here, so we send out the data, and BLOCK, but what if the USB is not connected?
         We must exit after a timeout */
       USBMIDI_OutputData();
   }

   midiReady = MIDIDataReady(byte, &MIDICable[cableNo]);
   /* Copy it out, so the tempbuffer is ready again */
   if( midiReady == MIDI_DATA_READY)
   {
      memcpy(&MIDImsgComplete[wMIDImsgCount], &MIDICable[cableNo].msg, sizeof(usbMIDIMessage_t));
	  MIDImsgComplete[wMIDImsgCount].header = MIDImsgComplete[wMIDImsgCount].header | (cableNo << 4);
      wMIDImsgCount = (wMIDImsgCount + 1) & MIDI_OUT_MASK;
   }
}



void USBMIDI_EnableRequests(void)
{
}



void USBMIDI_ProcessBuffer(void)
{
}


void usbPoll(void)
{

	MIDI_EventPacket_t ReceivedMIDIEvent;
	uint8_t* data;
	uint8_t i;

    if(MIDI_Device_ReceiveEventPacket(&MIDI_Interface, &ReceivedMIDIEvent))
	{
		/* Route it all to the UART port at 31250 baud */
	   uint8_t codeIndexNumber;
	   uint8_t messageSize = 0;
	   uint8_t cableNo = 0;
	   uint8_t j = 0;

	   PORTD ^= (1 << MIDI_IN_LED_RED);
	   data = (uint8_t*)&ReceivedMIDIEvent;

		codeIndexNumber = data[0] & (0x0F);
		messageSize = FLASH_GET_PGM_BYTE(&MIDIResponseMap[codeIndexNumber]);
		cableNo = data[0] >> 4;

		for(j = 0; j < messageSize; j++)
		{
			uint8_t buffer = data[j+1];
			RxBuffer[cableNo][rxWritePtr[cableNo]] = buffer;
			rxWritePtr[cableNo] = ((rxWritePtr[cableNo] + 1) & RX_BUFFER_MASK);
		}

	}
	MIDI_Device_USBTask(&MIDI_Interface);
	USB_USBTask();
}

void usbInit(void)
{
	USB_Init();
}


void USBMIDI_OutputData(void)
{
    /* If there is device => USB waiting to be sent */
    while( usbMIDI_bufferLen() )
    {
        uint8_t* data = (uint8_t*)&MIDImsgComplete[rMIDImsgCount];

		MIDI_Device_SendEventPacket(&MIDI_Interface, (MIDI_EventPacket_t*)&MIDImsgComplete[rMIDImsgCount]);
		rMIDImsgCount = (rMIDImsgCount + 1) & MIDI_OUT_MASK;

		PORTD ^= (1 << MIDI_OUT_LED_RED);
        MIDI_Device_USBTask(&MIDI_Interface);
	    USB_USBTask();
    }    
}


void hardwareReset(void)
{
   wdt_enable(WDTO_15MS);
}


void bootuartInit(void)
{
	UCSR1B |= (1<<RXEN) | (1<<TXEN) | (1<<TXCIE) | (1<<RXCIE);	/*Enable Rx and Tx modules*/
	UCSR1B &= ~(1<<UCSZ2);				/*Set to 8bit mode*/
	

	/*Select UCSRC to be written to*/	
	/* Set to Asynchronous Mode
	 *			 1 Stop-bit
	 *			 No Parity
	 *			 8-bit char mode
	 */
	UCSR1C = (UCSR1C & ~( UCSRCMASK ))
				|  (NOPARITY<<UPM0)
				|	(BIT8 << UCSZ0) 
				|  (1<<URSEL);

   /*Enable the pull up on RXD */
   PORTD |= (1 << PD0);
}



void bootloader_init(void)
{
   /* Set the rows are inputs FIRST! */
   UI_ROW_DIR &= ~(UI_ROWS);
   UI_ROW_OUT |=  (UI_ROWS);

   /* Columns as outputs */  
   UI_COL_DIR |= (UI_COLS);
   UI_COL_OUT &= ~(UI_COLS); 
}


void bootloader_leave(void)
{
   cli();
   _flashmem_release();
   MCUCR = (1 << IVCE);     /* enable change of interrupt vectors */
   MCUCR = (0 << IVSEL);    /* move interrupts to application flash section */
   asm volatile("jmp 0"::);
}


void bootloader_enter(void)
{
   MCUCR = (1 << IVCE);
   MCUCR = (1 << IVSEL);  

   DDRD |= ((1 << MIDI_IN_LED_GREEN) | (1 << MIDI_IN_LED_RED) | (1 << MIDI_OUT_LED_RED) | (1 << MIDI_OUT_LED_GREEN));

   //DDRD |= (1 << 7);
   boot_lock_bits_set ((1<<BLB11));
   
   bootuartInit();
   UBRR1L = BAUD(31250);
   sei();

   bootuartTxString_P(PSTR("Welcome to "));
   bootuartTxString_P(&VersionID);
   bootuartTx('\n');
   bootuartTx('\r');
   bootuartTxString_P(PSTR("by FuzzyJohn Inc. 2011"));

   uint8_t nextByte;
   firmwareDataCount = 0;
   firmwareByteCount = 0;
   firmwareAddress = 0;

   while(1)
   {
      usbPoll();
      while( USBMIDI_GetByte(&nextByte, 0) != NO_DATA_BYTE)
      {
         ParseFirmwareData(nextByte);
      }
      USBMIDI_OutputData();
   }
}

int main(void)
{

   MCUSR = 0;
   wdt_disable();

   MCUCR = (1 << JTD);
   MCUCR = (1 << JTD);
   bootloader_init();

   /* If bootloader condition */
   if( BOOTLOADER_CONDITION )
   {  
      usbInit();
      bootloader_enter();
   }
   else
   {
      bootloader_leave();
   }
   return 0;
}




/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&MIDI_Interface);
	//ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&External_MIDI_Thru_Interface); 
	
}

/** Event handler for the library USB Control Request event. */
void EVENT_USB_Device_ControlRequest(void)
{
	MIDI_Device_ProcessControlRequest(&MIDI_Interface);
	//MIDI_Device_ProcessControlRequest(&External_MIDI_Thru_Interface);
}



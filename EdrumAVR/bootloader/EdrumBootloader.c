#include <avr/io.h>
#include <string.h>
#include "hardwareSpecific.h"
#include "EdrumBootloader.h"
#include "flashmem/flashmem.h"
#include "firmwareUpdate/firmwareUpdate.h"
#include "USBMIDI/USBMIDI.h"
#include "usbdrv/usbdrv.h"
#include "usbconfig.h"



const PROGRAM_CHAR VersionID[] = "USB-MIDI Bootloader V1.0";

//Must be a power of two
#define RX_BUFFER_SIZE (32)
#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1)
/* After 200 times to retry sending a message, we assume the USB is
   disconnected */
#define USB_CONNECT_TIMEOUT (1500)

volatile uint8_t rxReadPtr;
uint8_t USB_Connected;
volatile uint8_t RxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxWritePtr;

// This descriptor is based on http://www.usb.org/developers/devclass_docs/midi10.pdf
// 
// Appendix B. Example: Simple MIDI Adapter (Informative)
// B.1 Device Descriptor
//
static PROGMEM char deviceDescrMIDI[] = {	/* USB device descriptor */
	18,			/* sizeof(usbDescriptorDevice): length of descriptor in bytes */
	USBDESCR_DEVICE,	/* descriptor type */
	0x10, 0x01,		/* USB version supported */
	0,			/* device class: defined at interface level */
	0,			/* subclass */
	0,			/* protocol */
	8,			/* max packet size */
	USB_CFG_VENDOR_ID,	/* 2 bytes */
	USB_CFG_DEVICE_ID,	/* 2 bytes */
	USB_CFG_DEVICE_VERSION,	/* 2 bytes */
	1,			/* manufacturer string index */
	2,			/* product string index */
	0,			/* serial number string index */
	1,			/* number of configurations */
};

// B.2 Configuration Descriptor
static PROGMEM char configDescrMIDI[] = {	/* USB configuration descriptor */
	9,			/* sizeof(usbDescrConfig): length of descriptor in bytes */
	USBDESCR_CONFIG,	/* descriptor type */
	101, 0,			/* total length of data returned (including inlined descriptors) */
	2,			/* number of interfaces in this configuration */
	1,			/* index of this configuration */
	0,			/* configuration name string index */
	USBATTR_SELFPOWER,	/* attributes */
	USB_CFG_MAX_BUS_POWER / 2,	/* max USB current in 2mA units */

// B.3 AudioControl Interface Descriptors
// The AudioControl interface describes the device structure (audio function topology) 
// and is used to manipulate the Audio Controls. This device has no audio function 
// incorporated. However, the AudioControl interface is mandatory and therefore both 
// the standard AC interface descriptor and the classspecific AC interface descriptor 
// must be present. The class-specific AC interface descriptor only contains the header 
// descriptor.

// B.3.1 Standard AC Interface Descriptor
// The AudioControl interface has no dedicated endpoints associated with it. It uses the 
// default pipe (endpoint 0) for all communication purposes. Class-specific AudioControl 
// Requests are sent using the default pipe. There is no Status Interrupt endpoint provided.
	/* AC interface descriptor follows inline: */
	9,			/* sizeof(usbDescrInterface): length of descriptor in bytes */
	USBDESCR_INTERFACE,	/* descriptor type */
	0,			/* index of this interface */
	0,			/* alternate setting for this interface */
	0,			/* endpoints excl 0: number of endpoint descriptors to follow */
	1,			/* */
	1,			/* */
	0,			/* */
	0,			/* string index for interface */

// B.3.2 Class-specific AC Interface Descriptor
// The Class-specific AC interface descriptor is always headed by a Header descriptor 
// that contains general information about the AudioControl interface. It contains all 
// the pointers needed to describe the Audio Interface Collection, associated with the 
// described audio function. Only the Header descriptor is present in this device 
// because it does not contain any audio functionality as such.
	/* AC Class-Specific descriptor */
	9,			/* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
	36,			/* descriptor type */
	1,			/* header functional descriptor */
	0x0, 0x01,		/* bcdADC */
	9, 0,			/* wTotalLength */
	1,			/* */
	1,			/* */

// B.4 MIDIStreaming Interface Descriptors

// B.4.1 Standard MS Interface Descriptor
	/* interface descriptor follows inline: */
	9,			/* length of descriptor in bytes */
	USBDESCR_INTERFACE,	/* descriptor type */
	1,			/* index of this interface */
	0,			/* alternate setting for this interface */
	2,			/* endpoints excl 0: number of endpoint descriptors to follow */
	1,			/* AUDIO */
	3,			/* MS */
	0,			/* unused */
	0,			/* string index for interface */

// B.4.2 Class-specific MS Interface Descriptor
	/* MS Class-Specific descriptor */
	7,			/* length of descriptor in bytes */
	36,			/* descriptor type */
	1,			/* header functional descriptor */
	0x0, 0x01,		/* bcdADC */
	65, 0,			/* wTotalLength */

// B.4.3 MIDI IN Jack Descriptor
	6,			/* bLength */
	36,			/* descriptor type */
	2,			/* MIDI_IN_JACK desc subtype */
	1,			/* EMBEDDED bJackType */
	1,			/* bJackID */
	0,			/* iJack */

	6,			/* bLength */
	36,			/* descriptor type */
	2,			/* MIDI_IN_JACK desc subtype */
	2,			/* EXTERNAL bJackType */
	2,			/* bJackID */
	0,			/* iJack */

//B.4.4 MIDI OUT Jack Descriptor
	9,			/* length of descriptor in bytes */
	36,			/* descriptor type */
	3,			/* MIDI_OUT_JACK descriptor */
	1,			/* EMBEDDED bJackType */
	3,			/* bJackID */
	1,			/* No of input pins */
	2,			/* BaSourceID */
	1,			/* BaSourcePin */
	0,			/* iJack */

	9,			/* bLength of descriptor in bytes */
	36,			/* bDescriptorType */
	3,			/* MIDI_OUT_JACK bDescriptorSubtype */
	2,			/* EXTERNAL bJackType */
	4,			/* bJackID */
	1,			/* bNrInputPins */
	1,			/* baSourceID (0) */
	1,			/* baSourcePin (0) */
	0,			/* iJack */


// B.5 Bulk OUT Endpoint Descriptors

//B.5.1 Standard Bulk OUT Endpoint Descriptor
	9,			/* bLenght */
	USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
	0x1,			/* bEndpointAddress OUT endpoint number 1 */
	3,			/* bmAttributes: 2:Bulk, 3:Interrupt endpoint */ //This should be bulk for max speed
	8, 0,			/* wMaxPacketSize */
	2,			/* bIntervall in ms */
	0,			/* bRefresh */
	0,			/* bSyncAddress */

// B.5.2 Class-specific MS Bulk OUT Endpoint Descriptor
	5,			/* bLength of descriptor in bytes */
	37,			/* bDescriptorType */
	1,			/* bDescriptorSubtype */
	1,			/* bNumEmbMIDIJack  */
	1,			/* baAssocJackID (0) */


//B.6 Bulk IN Endpoint Descriptors

//B.6.1 Standard Bulk IN Endpoint Descriptor
	9,			/* bLenght */
	USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
	0x81,			/* bEndpointAddress IN endpoint number 1 */
	3,			/* bmAttributes: 2: Bulk, 3: Interrupt endpoint */ // This should be bulk so that it is faster than the UART
	8, 0,			/* wMaxPacketSize */
	2,			/* bIntervall in ms */
	0,			/* bRefresh */
	0,			/* bSyncAddress */

// B.6.2 Class-specific MS Bulk IN Endpoint Descriptor
	5,			/* bLength of descriptor in bytes */
	37,			/* bDescriptorType */
	1,			/* bDescriptorSubtype */
	1,			/* bNumEmbMIDIJack (0) */
	3,			/* baAssocJackID (0) */
};




/* Firmware Update also via UART */
ISR(SIG_UART_RECV)
{
   uint8_t buffer = UDR;
   //USBMIDI_PutByte(buffer);

   /* Echo this back out */
   RxBuffer[rxWritePtr] = buffer;
   rxWritePtr = ((rxWritePtr + 1) & RX_BUFFER_MASK);
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
	while( (UCSRA & (1<<UDRE)) == 0 );
	/*Send byte to output buffer*/
	UDR	= outbyte;
}


uchar usbFunctionDescriptor(usbRequest_t * rq)
{
   USB_Connected = 1;

   if (rq->wValue.bytes[1] == USBDESCR_DEVICE) 
   {
      usbMsgPtr = (uchar *) deviceDescrMIDI;
      return sizeof(deviceDescrMIDI);
   } 
   else //(rq->wValue.bytes[1] == USBDESCR_CONFIG)  
   { /* must be config descriptor */
      
      usbMsgPtr = (uchar *) configDescrMIDI;
      return sizeof(configDescrMIDI);
   }

}
 
 
 
 
 
 
 
/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */
 
 
uchar usbFunctionSetup(uchar data[8])
{
   usbRequest_t *rq = (void *) data;
  
   if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) 
   { /* class request type */
 
      /*  Prepare bulk-in endpoint to respond to early termination   */
      if ((rq->bmRequestType & USBRQ_DIR_MASK) == USBRQ_DIR_HOST_TO_DEVICE)
      {
      }
   }
   return 0xff;
}
 
 
 
 
/*---------------------------------------------------------------------------*/
/* usbFunctionRead                                                           */
/*---------------------------------------------------------------------------*/
/* We're never asked to send lots of data, so lets not send anything */
uchar usbFunctionRead(uchar * data, uchar len)
{
	return 0;   
}
 
 
 
 
/*---------------------------------------------------------------------------*/
/* usbFunctionWrite                                                          */
/*---------------------------------------------------------------------------*/
/* We don't expect any data so return 1 */
uchar usbFunctionWrite(uchar * data, uchar len)
{
   return 1;
}
 
 
/*---------------------------------------------------------------------------*/
/* usbFunctionWriteOut                                                       */
/*                                                                           */
/* this Function is called if a MIDI Out message (from PC) arrives.          */
/*                                                                           */
/*---------------------------------------------------------------------------*/
 
 
void usbFunctionWriteOut(uchar * data, uchar len)
{
   uint8_t byteCount = 0;

   /* Route it all to the UART port at 31250 baud */
   byteCount = usbMIDI_ParseData(data, len);
   while(byteCount--)
   {
      uint8_t buffer = *data++;
      RxBuffer[rxWritePtr] = buffer;
      rxWritePtr = ((rxWritePtr + 1) & RX_BUFFER_MASK);
   }
//   uartTxDump(data, byteCount);

#if USB_CFG_HAVE_FLOWCONTROL
   /* Here if the TxBuffer cannot hold the new data, we should disable all 
    * USB requests and wait for the MIDI TxBuffer to catch up.
    * Since we need to process the next request, we have to disable BEFORE
    * the buffer will be full (hence the 16).
    *
    * We shall re-enable the request in the main loop, not the interrupt.  */
   uint8_t length;

   if( rxReadPtr > rxWritePtr  )
   {
      length = rxWritePtr + RX_BUFFER_SIZE - rxReadPtr;
   }
   else
   {
      length = rxWritePtr - rxReadPtr;
   }

   if( (length + 8) > RX_BUFFER_MASK)
   {
      UDR = 0xAA;
      usbDisableAllRequests();
   }

   /* for internal loopback */
   /*for(uint8_t i = 0 ; i < byteCount ; i++ )
   {
      ringbuffer_put( (RINGBUFFER_T*)&ReceiveBuffer, data[i]);
   }*/

   /* This shouldn't happen since the USB output is quite fast (bulk) */
   /* This is required if our endpoints are interrupt based, note that you can't
    * DisableAllRequests on a bulk transfer. */
   if( !usbMIDI_bufferIsReady() )
   {
      UDR = 0xCC;
      usbDisableAllRequests();
   }
#endif
}
 

/* This reads the MIDI data received from USB */
uint8_t USBMIDI_GetByte(void)
{
   /* Process messages in the UART Rx buffer is there are any */
   if( rxReadPtr != rxWritePtr )
   {
      uint8_t nextByte = RxBuffer[rxReadPtr];
      rxReadPtr = ((rxReadPtr + 1) & RX_BUFFER_MASK);
      return nextByte;
   }

   return NO_DATA_BYTE;
}

/* This here makes the process Buffer redundant */
void USBMIDI_PutByte(uint8_t byte)
{
   uint8_t midiReady;
   uint16_t retry;
   /* Only process this stuff if we have enough room in the MIDI out buffer, we must block otherwise */
   if( !usbMIDI_bufferIsReady() )
   {
      /* Data will be lost here, so we send out the data, and BLOCK, but what if the USB is not connected?
         We must exit after a timeout */
      while(!usbMIDI_bufferIsReady() && USB_Connected)
      {
         if( retry++ > USB_CONNECT_TIMEOUT )
         {
            USB_Connected = 0;
         }
         usbPoll();
         USBMIDI_OutputData();
      }
   }

   midiReady = MIDIDataReady(byte, &MIDImsgComplete[wMIDImsgCount]);
   /* Copy it out, so the tempbuffer is ready again */
   if( midiReady == MIDI_DATA_READY)
   {
      wMIDImsgCount = (wMIDImsgCount + 1) & MIDI_OUT_MASK;
   }
}



void USBMIDI_EnableRequests(void)
{
#if USB_CFG_HAVE_FLOWCONTROL
   uint8_t length;

   if( rxReadPtr > rxWritePtr  )
   {
      length = rxWritePtr + RX_BUFFER_SIZE - rxReadPtr;
   }
   else
   {
      length = rxWritePtr - rxReadPtr;
   }

   if( (length + 8) < RX_BUFFER_MASK)
   {
      length = 1;
   }
   else
   {
      length = 0;
   }
   /* To save the UART output buffer from overflowing, due to a
    * UART Speed < USB Inspeed (likely under bulk mode, but unlikely under interrupt mode */
   /* To save the USB output buffer from overflowing, due to a
    * UART Speed > USB Outspeed (not likely under bulk mode, but likely under interrupt mode */

   if(  usbAllRequestsAreDisabled() 
      && usbMIDI_bufferIsReady()
      && (length))
   {
      usbEnableAllRequests();
   }
#endif
}



void USBMIDI_ProcessBuffer(void)
{
}





void USBMIDI_OutputData(void)
{
    uint8_t outputBuffer[8];
    /* If there is device => USB waiting to be sent */

    while( usbMIDI_bufferLen() && usbInterruptIsReady() )
    {
       usbPoll();
    /* If the USB Interrupt isn't ready, then we have an issue
     * of data coming from the UART quicker than it is going out, to keep up
     * we need to send 2 data events sometimes */
       if( usbMIDI_bufferLen() == 1 )
       {
          /* Send 1 byte */
          usbSetInterrupt( (uint8_t*)&MIDImsgComplete[rMIDImsgCount], 4);
          rMIDImsgCount = (rMIDImsgCount + 1) & MIDI_OUT_MASK;
       }
       else
       {
          /* Send 2 bytes */
          memcpy(&outputBuffer, &MIDImsgComplete[rMIDImsgCount], sizeof(usbMIDIMessage_t));
          memcpy(&outputBuffer[4], &MIDImsgComplete[(rMIDImsgCount+1) & MIDI_OUT_MASK], sizeof(usbMIDIMessage_t));
          usbSetInterrupt( (uint8_t*)&outputBuffer, 8);
          rMIDImsgCount = (rMIDImsgCount + 2) & MIDI_OUT_MASK;

       }
    }    

}


void hardwareReset(void)
{
   wdt_enable(WDTO_15MS);
}


void bootuartInit(void)
{
	UCSRB |= (1<<RXEN) | (1<<TXEN) | (1<<TXCIE) | (1<<RXCIE);	/*Enable Rx and Tx modules*/
	UCSRB &= ~(1<<UCSZ2);				/*Set to 8bit mode*/
	

	/*Select UCSRC to be written to*/	
	/* Set to Asynchronous Mode
	 *			 1 Stop-bit
	 *			 No Parity
	 *			 8-bit char mode
	 */
	UCSRC = (UCSRC & ~( UCSRCMASK ))
				|  (NOPARITY<<UPM0)
				|	(BIT8 << UCSZ0) 
				|  (1<<URSEL);

   /*Enable the pull up on RXD */
   PORTD |= (1 << PD0);
}



void bootloader_init(void)
{
   /* Monitor the interrupt pin */
   DDRD &= ~(1 << 3);
   PORTD &= ~(1<<3);

   /* Columns as outputs */  
   UI_COL_DIR |= (UI_COLS);
   UI_COL_OUT |= (UI_COLS);

   UI_ROW_DIR &= ~(UI_ROWS);
   UI_ROW_OUT &= ~(UI_ROWS); 
}


void bootloader_leave(void)
{
   cli();
   _flashmem_release();
   GICR = (1 << IVCE);     /* enable change of interrupt vectors */
   GICR = (0 << IVSEL);    /* move interrupts to application flash section */
   asm volatile("jmp 0"::);
}


void bootloader_enter(void)
{
   GICR = (1 << IVCE);
   GICR = (1 << IVSEL);  
   DDRD |= (1 << 7);

   boot_lock_bits_set ((1<<BLB11));
   
   usbInit();
   bootuartInit();
   UBRRL = 39;
   sei();

   bootuartTxString_P(PSTR("Welcome to "));
   bootuartTxString_P(&VersionID);
   bootuartTx('\n');
   bootuartTx('\r');
   bootuartTxString_P(PSTR("by FuzzyJohn Inc. 2011"));

   uint8_t nextByte;

   while(1)
   {
      usbPoll();
      nextByte = USBMIDI_GetByte();
      if( nextByte != NO_DATA_BYTE )
      {
         ParseFirmwareData(nextByte);
      }
   }
}

int main(void)
{
   MCUCSR = (1 << JTD);
   MCUCSR = (1 << JTD);
   bootloader_init();
   wdt_disable();

   /* If bootloader condition */
   if( BOOTLOADER_CONDITION )
   {  
      bootloader_enter();
   }
   else
   {
      bootloader_leave();
   }
   return 0;
}


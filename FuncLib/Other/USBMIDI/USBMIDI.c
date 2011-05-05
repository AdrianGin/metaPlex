
#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "USBMIDI/USBMIDI.h"


uint8_t MIDIResponseMap[] PROGRAM_SPACE = {
   3, //CIN=0x0, MIDI Size = 1,2 or 3 (Misc)
   3, //CIN=0x1, MIDI Size = 1,2 or 3 (Cable Events)
   2, //CIN=0x2, MIDI Size = 2 (2 byte System Common Message)
   3, //CIN=0x3, MIDI Size = 3 (3 byte System Common Message)
   3, //CIN=0x4, MIDI Size = 3 (SysEx Starts / Continues)
   1, //CIN=0x5, MIDI Size = 1 (SysEx End Byte (7F) )
   2, //CIN=0x6, MIDI Size = 2 (SysEx 1 Byte + End )
   3, //CIN=0x7, MIDI Size = 3 (SysEx 2 Byte + End )
   3, //CIN=0x8, MIDI Size = 3 (Note Off)
   3, //CIN=0x9, MIDI Size = 3 (Note On)
   3, //CIN=0xA, MIDI Size = 3 (PolyKey Press)
   3, //CIN=0xB, MIDI Size = 3 (Control Change)
   2, //CIN=0xC, MIDI Size = 2 (Program Change)
   2, //CIN=0xD, MIDI Size = 2 (Channel Pressure)
   3, //CIN=0xE, MIDI Size = 3 (Pitchbend)
   1}; //CIN=0xF, MIDI Size = 1 (Single Byte Press)




/* Consider all the rest to be 1-Byte System Common Messages, CN defaults to 0 */
uint8_t MIDILookupTable[] PROGRAM_SPACE =
{
   // Start of Voice Messages
   0x80, 3, 0x08,
   0x90, 3, 0x09,
   0xA0, 3, 0x0A,
   0xB0, 3, 0x0B,
   0xC0, 2, 0x0C,
   0xD0, 2, 0x0D,
   0xE0, 3, 0x0E,
   0x00,       
   // Start of System Common Messages
   0xF0, 3, 0x04,
   0xF1, 2, 0x02,
   0xF2, 3, 0x03,
   0xF3, 2, 0x02,
   0x00
};




/* Hold enough data for a few messages,
 */
usbMIDIMessage_t MIDImsgComplete[MIDI_OUT_BUFFER];

uint8_t wMIDImsgCount = 0;
uint8_t rMIDImsgCount = 0; 



uint8_t usbMIDI_bufferLen(void)
{  
    if( rMIDImsgCount > wMIDImsgCount )
    {
        return wMIDImsgCount + MIDI_OUT_BUFFER - rMIDImsgCount;
    }
    return wMIDImsgCount - rMIDImsgCount;
}

uint8_t usbMIDI_bufferIsReady(void)
{
   return ((wMIDImsgCount+ 1) & MIDI_OUT_MASK) != ((rMIDImsgCount) & MIDI_OUT_MASK);
}

/*  */
/*uint8_t usbMIDI_ParseData(uint8_t* data, uint8_t len)
{
   uint8_t codeIndexNumber;
   uint8_t messageSize = 0;
   uint8_t byteCount = 0;
   uint8_t i = 0;

   for( i = 0 ; i < len ; i = i + sizeof(usbMIDIMessage_t) )
   {
      if( data[i] )
      {
         codeIndexNumber = data[i] & (0x0F);
         messageSize = FLASH_GET_PGM_BYTE(&MIDIResponseMap[codeIndexNumber]);
         memcpy(data + byteCount, data+i+1 , messageSize);
         byteCount = byteCount + messageSize;
      }
   }

   return byteCount;
}*/





uint8_t LookupMIDIMessage(uint8_t MIDIStatusCode)
{
   uint8_t i;
   /* System Common Message */
   if( MIDIStatusCode >= 0xF0 )
   {
      for( i = SYS_COMMON_MSG; FLASH_GET_PGM_BYTE(&MIDILookupTable[i]) ; i = i + 3 )
      {
         if( MIDIStatusCode == FLASH_GET_PGM_BYTE(&MIDILookupTable[i]) )
         {
            return i;
         }
      }
      return UNKNOWN_SYS_MSG;
   }
   else
   {
      MIDIStatusCode = (MIDIStatusCode & 0xF0);
      for( i = 0; FLASH_GET_PGM_BYTE(&MIDILookupTable[i]) ; i = i + 3 )
      {
         if( MIDIStatusCode == FLASH_GET_PGM_BYTE(&MIDILookupTable[i]) )
         {
            return i;
         }
      }
      return UNKNOWN_VOICE_MSG;
   }
}
 

uint8_t GetCINNumber(uint8_t messageIndex)
{
   if( messageIndex == UNKNOWN_SYS_MSG)
   {
        return 0x05;
   }
   uint8_t CIN = FLASH_GET_PGM_BYTE(&MIDILookupTable[messageIndex+2]);
   /* Most system common messages are 1 byte long and
    * therefore have a CIN of 0x05 */
   return CIN;
}
 

uint8_t GetNumberOfBytesToRead(uint8_t messageIndex)
{
   /* If the message is undefined, return 1 byte to get (most likely RT message) */
   /* Most system common messages are 1 byte long */
   if( messageIndex == UNKNOWN_SYS_MSG)
   {
        return 1;
   }
   uint8_t bytesToReturn = FLASH_GET_PGM_BYTE(&MIDILookupTable[messageIndex+1]);
   
   return bytesToReturn;
}
 



 
uint8_t MIDIDataReady(uint8_t inByte, usbMIDIcable_t* usbMIDIcable)
{
   /* You can technically have two midi events per midi message, but lets just
    * keep it to one.
    */
   uint8_t *byteCount = &usbMIDIcable->byteCount;
   uint8_t *bytesToReceive = &usbMIDIcable->bytesToReceive;
   uint8_t *receivingSysEx = &usbMIDIcable->receivingSysEx;
   /* This stores the last received status byte */
   uint8_t *runningStatus = &usbMIDIcable->runningStatus;
   /* This stores the data byte 1 which may need to be restored if a RealTime message
    * is received */
   uint8_t *lastDataByte = &usbMIDIcable->lastDataByte;


   usbMIDIMessage_t*  MIDIMessage = &usbMIDIcable->msg;
   uint8_t CIN = 0;
   uint8_t messageIndex = 0;


   /* Implement running status here */

   if( (*byteCount == 0) && (inByte <= MIDI_MAX_DATA) && (!(*receivingSysEx)))
   {
      messageIndex = LookupMIDIMessage(*runningStatus);
      *bytesToReceive = GetNumberOfBytesToRead(messageIndex);
      CIN = GetCINNumber(messageIndex);
      MIDIMessage->header = CIN;
      MIDIMessage->MIDIData[0] = *runningStatus;

      if( *lastDataByte != NO_DATA_BYTE )
      {
         MIDIMessage->MIDIData[1] = *lastDataByte;
         *byteCount = 2;
      }
      else
      {
         *byteCount = 1;
      }
   }

   /* A new status byte is received, discard the old one */
   if( (*byteCount > 0) && (inByte > MIDI_MAX_DATA) && (inByte != MIDI_SYSEX_STOP))
   {
      *byteCount = 0;
      *receivingSysEx = 0;
   }

   if( inByte == MIDI_SYSEX_START )
   {
       *receivingSysEx = 1;
   }


   /* Determine how many bytes to receive based on the 'first'
    * byte (MIDI_Status Code) */
   if( *byteCount == 0 )
   {
      /* Save the new voice status */
      if( inByte < MIDI_SYSEX_START && inByte >= MIDI_NOTE_OFF )
      {
         *runningStatus = inByte;
         *lastDataByte  = NO_DATA_BYTE;
      }

      /* Clear */
      MIDIMessage->header = 0;
      MIDIMessage->MIDIData[0] = 0;
      MIDIMessage->MIDIData[1] = 0;
      MIDIMessage->MIDIData[2] = 0;

      if( !(*receivingSysEx) )
      {
          messageIndex = LookupMIDIMessage(inByte);
          *bytesToReceive = GetNumberOfBytesToRead(messageIndex);
          CIN = GetCINNumber(messageIndex);
          MIDIMessage->header = CIN;
      }
   }

   if( inByte <= MIDI_MAX_DATA )
   {
      *lastDataByte = inByte;
   }



   MIDIMessage->MIDIData[*byteCount] = inByte;
   *byteCount = *byteCount + 1;

   if( *receivingSysEx )
   {      
   /* By default we are continuing a SysEx Message */
      *bytesToReceive = 3;
      MIDIMessage->header = 0x04;
 
      /* If we recieve a SYSEX STOP then send the message */
      if( inByte == MIDI_SYSEX_STOP )
      {
         *bytesToReceive = *byteCount;
         MIDIMessage->header = *byteCount + 0x04;
         *receivingSysEx = 0;
      }
   }

   if( *byteCount == *bytesToReceive)
   {
      *byteCount = 0;
      if( inByte < MIDI_TIMING_CLOCK )
      {
         *lastDataByte  = NO_DATA_BYTE;
      }

      return MIDI_DATA_READY;
   }
   return MIDI_DATA_NOT_READY;
}
 

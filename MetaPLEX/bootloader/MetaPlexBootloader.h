#ifndef METAPLEX_BOOTLOADER_H
#define METAPLEX_BOOTLOADER_H

#include "Descriptors.h"

#define BOOTLOADER_CONDITION (!(PINB & (1 << 6)))

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


void bootuartInit(void);
void bootloader_enit(void);
void bootloader_enter(void);
void bootloader_leave(void);

void bootuartTxString_P(PGM_P outString_P);
void bootuartTx(uint8_t outbyte);

void usbPoll(void);

uint8_t USBMIDI_GetByte(uint8_t* inByte, uint8_t cableNo);
void USBMIDI_PutByte(uint8_t byte, uint8_t cableNo);
void USBMIDI_EnableRequests(void);
void USBMIDI_ProcessBuffer(void);
void USBMIDI_OutputData(void);

#endif

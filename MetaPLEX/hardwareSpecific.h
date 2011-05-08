#ifndef __HARDWARE_SPECIFIC
#define __HARDWARE_SPECIFIC

/* This file is specific to the AVR build of Edrum */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <avr/wdt.h>  
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>

#include "version.h"

#include "hardUART/hardUart.h"
#include "avrADC/adc.h"


/* Interrupts */
//#define ENABLE_INTERRUPT() (sei())
//#define DISABLE_INTERRUPT() (cli())
#define PROGRAM_SPACE PROGMEM
#define PROGRAM_CHAR  prog_char
#define PROGRAM_PTR   PGM_P

/* MIDI Defines */
#define MIDI_Tx(x)   midiTx(x)
#define MIDI_SetBaudRate(high,low) //uartSetBaud(high,low)
#define MIDI_BAUD(x)   (((F_CPU / (x)) / 16) - 1)
#define MIDI_BAUD_31250      MIDI_BAUD(31250)
#define MIDI_BAUD_38400      MIDI_BAUD(38400)
#define MIDI_BAUD_115200		MIDI_BAUD(115200)
#define MIDI_BAUD_1M			MIDI_BAUD(1000000)

#define MIDI_IN_LED_GREEN	(4)
#define MIDI_IN_LED_RED  (5)
#define MIDI_OUT_LED_GREEN (6)
#define MIDI_OUT_LED_RED (7)

/* SPI Defines */
#define SPI_DDR   (DDRB)
#define SPI_PORT  (PORTB)
#define SCK       (PB1)
#define MISO      (PB3)
#define MOSI      (PB2)
#define nSS       (PB0)

/* Timer Related */
#define ENABLE_PRIMARY_TIMER()      TIMSK1 |= (1 << OCIE1A)
#define ENABLE_AUXILIARY_TIMER()    TIMSK1 |= (1 << OCIE1B)
#define DISABLE_PRIMARY_TIMER()     TIMSK1 &= ~(1 << OCIE1A)
#define DISABLE_AUXILIARY_TIMER()   TIMSK1 &= ~(1 << OCIE1B)
#define DISABLE_KEYPAD()            PCICR &= ~(1 << PCIE0); PCMSK0 &= ~(UI_ROW0)
#define ENABLE_KEYPAD()             PCICR |= (1 << PCIE0); PCMSK0 |= (UI_ROW0)
#define CLEAR_KEYPAD()              PCIFR = (1 << PCIF0)

/* Uart Defines */
extern volatile AVR_USART_t PrimaryUART;
#define UART_Init(x)            uartInit(&PrimaryUART, x)
#define UART_SetBaud(high, low) uartSetBaud(&PrimaryUART, high, low)
#define UART_Tx(x)              uartTx(&PrimaryUART, x)
#define UART_TxDump(x)          uartTxDump(&PrimaryUART, x)
#define UART_Shutdown(x)        uartDisable(&PrimaryUART, x)
#define UART_TxString_P(x)      uartTxString_P(&PrimaryUART, x)
#define UART_TxString(x)        uartTxString(&PrimaryUART, x)

#define ANALOGUE_INPUTS	(40)
#define METRONOME_INPUTS (16)
#define DIGITAL_INPUTS	(0)
#define KEYPAD_INPUTS (0)

/* Channel Select Defines */
#define CHSELOUT  PORTA
#define CHSELDIR  DDRA
#define CHSELSEL  P4SEL
#define CHSELIN   PINA

#define CHSELA     (0)
#define CHSELB     (1)
#define CHSELC     (2)
#define CHSELD     (3)

#define CHSELPINS  ( (1 << CHSELA) | (1 << CHSELB) | (1 << CHSELC) | (1 << CHSELD) )
/* This is effectively the number of channels per ADC channel */
#define CHANNEL_COUNT (8)

/* Sensor Defines */
/* Variable Output */
#define ADC_CHANNEL(x)  (x)
#define DEFAULT_ADC_CHANNEL (0)
#define POT_INPUT			(6)


/* Digital Input Section */
#define DIGITAL0_DDR DDRA
#define DIGITAL1_DDR DDRA
#define DIGITAL2_DDR DDRA
#define DIGITAL3_DDR DDRC
#define DIGITAL4_DDR DDRC
#define DIGITAL5_DDR DDRD
#define DIGITAL6_DDR DDRD
#define DIGITAL7_DDR DDRC

#define DIGITAL0_PORT PORTA
#define DIGITAL1_PORT PORTA
#define DIGITAL2_PORT PORTA
#define DIGITAL3_PORT PORTC
#define DIGITAL4_PORT PORTC
#define DIGITAL5_PORT PORTD
#define DIGITAL6_PORT PORTD
#define DIGITAL7_PORT PORTC

#define DIGITAL0_PIN PINA
#define DIGITAL1_PIN PINA
#define DIGITAL2_PIN PINA
#define DIGITAL3_PIN PINC
#define DIGITAL4_PIN PINC
#define DIGITAL5_PIN PIND
#define DIGITAL6_PIN PIND
#define DIGITAL7_PIN PINC

#define DIGITAL0 (1 << 5)
#define DIGITAL1 (1 << 4)
#define DIGITAL2 (1 << 3)
#define DIGITAL3 (1 << 2)
#define DIGITAL4 (1 << 1)
#define DIGITAL5 (1 << 5)
#define DIGITAL6 (1 << 4)
#define DIGITAL7 (1 << 0)


/* ADC Defines */
#define ADC_Init()       adcInitialise()
#define ADC_Sample()     adc10()
#define ADC_SetupAddress(channel) adcSetPin(channel)

#define ADC_REF_DEFAULT ADC_REF_AVCC
/* Lowest division before it doesn't work */
#define ADC_PRESCALE_DEFAULT ADC_PRESCALE_DIV128
#define ADC_SAMPLE_SPEEDus      (1000000 / (F_CPU / (4*13)))

#define ADC_RESOLUTION (10)

/* Keypad Defines */
/* Key Pad Inputs */
#define UI_COL0   (1<<3)
#define UI_COL1   (1<<4)
#define UI_COL2   (1<<5)
#define UI_COL3   (1<<6)
//Not used
//#define UI_ROW3   (1<<6)
//#define UI_ROW2   (1<<5)
//#define UI_ROW1   (1<<4)
#define UI_ROW0   (1<<6)

#define UI_ROWS	(UI_ROW0)
#define UI_COLS	(UI_COL0 | UI_COL1 | UI_COL2 | UI_COL3)

#define UI_COL_OUT (PORTA)
#define UI_ROW_OUT (PORTB)

#define UI_COL_IN (PINA)
#define UI_ROW_IN (PINB)

#define UI_COL_DIR (DDRA)
#define UI_ROW_DIR (DDRB)

/* KeyPad Defines */
/* 1st Row */
#define RAW_KP_1   (0x18)
#define RAW_KP_2   (0x58)

#define RAW_KP_3   (0x12)


/* 2nd Row */
#define RAW_KP_4   (0x38)

#define RAW_KP_5   (0x24)
#define RAW_KP_6   (0x70)

/* 3rd Row */
#define RAW_KP_7   (0x48)
#define RAW_KP_8   (0x68)

#define RAW_KP_9   (0x42)


/* 4th Row */
#define RAW_KP_STAR   (0x88)
#define RAW_KP_0      (0x84)
#define RAW_KP_HASH   (0x82)

#define RAW_KP_A   (RAW_KP_2 & RAW_KP_6)
#define RAW_KP_B   (RAW_KP_8 & RAW_KP_6)
#define RAW_KP_C   (RAW_KP_2 & RAW_KP_4)
#define RAW_KP_D   (RAW_KP_8 & RAW_KP_4)


/* LCD Defines */
#define LCD_BL_PORT  (PORTD)
#define LCD_BL_PIN   (7)
#define LCD_BL_DDR   (DDRD)
#define LCD_BL_SEL   (DDRD)
#define UI_LCD_GET_FLASHBYTE(x) pgm_read_byte(x)

#define MENU_SPACE PROGRAM_SPACE
#define MENU_GET_PTR(x)  pgm_read_word(&x)
#define MENU_GET_BYTE(x)  pgm_read_byte(&x)
#define MENU_TEXT    prog_char //for the AVR use prog_char if you want strings in ROM
#define MENU_SPACE_PTR	PGM_P
//#define PSTR(x)   (x) //for AVR include the pgmspace.h for this macro


/* Not used in Direct Mode */
#define UI_LCD_PORT     (0x00)

/* LCD Inputs */
#define UI_LCD_RS_PIN      (2)
#define UI_LCD_RS       (1<<2)
#define UI_LCD_E        (1<<3)
#define UI_LCD_D4       (1<<4)
#define UI_LCD_D5       (1<<5)
#define UI_LCD_D6       (1<<6)
#define UI_LCD_D7       (1<<7)    

/* Use 8bit mode via 74HC164 */
//#define UI_LCD_8BITMODE
#define UI_LCD_SERIAL_DATA (UI_LCD_RS)

#define UI_LCD_CONTROL_DIR   (DDRC)
#define UI_LCD_DATA_DIR      (DDRC)

#define UI_LCD_DATA	(UI_LCD_D4 | UI_LCD_D5 | UI_LCD_D6 | UI_LCD_D7)
#define UI_LCD_CONTROL	(UI_LCD_RS | UI_LCD_E)

#define UI_LCD_CONTROL_PORT	(PORTC)
#define UI_LCD_DATA_PORT		(PORTC)


#define LCD_FUNCTION_DEFAULT	((1<<LCD_FUNCTION) | (1<<LCD_FUNCTION_2LINES))
#define LCD_MODE_DEFAULT		((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC))
#define LCD_DISPLAY_DEFAULT   ((1<<LCD_DISPLAY) | (1<<LCD_ON_DISPLAY))

/* END OF VERSION WITHOUT PE Defines */

/* Bootloader */
#define BOOTLOADER_SIZE (8192)

/* Profile FLASH saving defines */
#define SET_SECTION(x) __attribute__ ((section ((x))))
/* Total Number of Profiles */
#define NUMBER_OF_PROFILES   (3)
/* FLASH Page size in bytes */
#define FLASH_BLOCK_SIZE      (SPM_PAGESIZE)
/* The last page in the Flash is reserved for the temp buffer */
#define FLASH_TEMP_BUFFER  (BOOTLOADER_START - FLASH_BLOCK_SIZE)
/* Effective End of UserSpace Flash. Total Flash - Bootloader Flash Size*/
#define BOOTLOADER_START             (FLASHEND - BOOTLOADER_SIZE + 1)

#define PROFILE_EEPROM (0)
#define PROFILE_FLASH  (1)
/* define either EEMEM or PROGRAM_SPACE */
#define PROFILE_MEMORY  PROFILE_FLASH

#if PROFILE_MEMORY == PROFILE_EEPROM
   #define PROFILE_SPACE   EEMEM
   #define PROFILE_COPY(dest, src, len) eeprom_read_block(dest, src, len)
#else
   #define PROFILE_SPACE   PROGRAM_SPACE
   #define PROFILE_COPY(dest, src, len) memcpy_P(dest, src, len)
#endif




#define FLASH_PAGE_ERASE(address)         boot_page_erase_safe(address)
#define FLASH_WORD_WRITE(address, data)   boot_page_fill_safe(address, data)
#define FLASH_FINALISE_WRITE(address)     boot_page_write_safe(address)
#define FLASH_RELEASE()                   boot_rww_enable_safe()
#define FLASH_WAIT_READY()                boot_spm_busy_wait()
#define FLASH_GET_PGM_WORD(address) pgm_read_word(address)
#define FLASH_GET_PGM_BYTE(address) pgm_read_byte(address)

#define JUMP_TABLE_LOCATION (0xFFC0)

#define FLASH_PROGRAMMING_ENTER  (1)
#define FLASH_PROGRAMMING_EXIT  (0)

/* hardware Specific defines */
void DigitalInputInit(void);
uint8_t getDigitalState(uint8_t DigitalChannel);
void SoftTimer_TimerInit(void);
void enableFlashProgramming(uint8_t mode);
uint16_t Sample_Channel(uint8_t channel);
void SensorChannel(uint8_t channel);


#endif

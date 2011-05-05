#ifndef CONTROLLERMODE_H
#define CONTROLLERMODE_H

#define CM_CHANNEL_CHANGE	(10)
#define CM_CODE_CHANGE		(11)
#define CM_KEY_CHANGE		(12)
#define CM_MIDI_MESSAGE		(13)
#define CM_MENU_ENTER		(14)
#define CM_ACCEPT_DATA		(15)
#define CM_MAX_INPUT			(CM_ACCEPT_DATA)

#define CM_PRINT_STATUS		(0xFE)
#define CM_VOID_INPUT		(0xFF)
#define CM_INPUT_BUFFER_SIZE	(4)

#define CM_MIN_CHANNEL		(1)
#define CM_MIN_MIDI_CODE	(MIDI_NOTE_OFF)

#define CM_SETTINGS_MODE (0)
#define CM_MENU_MODE	(1)

#define INPUT_TEXT    prog_char

enum {
	CM_CHANNEL_SELECT = 0,
	CM_MIDI_CODE_CHANGE,
	CM_MIDI_KEY_CHANGE,
	CM_SEND_MIDI_MESSAGE,
	CM_INPUT_TYPE_COUNT,
} cm_InputCodes;


typedef struct
{
   uint8_t  InputCode;
   const char* descriptor;
   void (*function)(void* data);

} CM_InputType_t;


void CM_InputSelect(uint8_t inputCode);

void CM_ReceiveInput(uint8_t inputKey);

void CM_ChannelParse(void* data);

void CM_SetChannel(uint8_t channel);

void CM_SetMIDICode(void* data);

void CM_SetMIDIKey(void* data);

void CM_printEnteredData(void);

void printChannelInfo(void);

uint8_t CM_GetMenuMode(void);

void CM_SetMenuMode(uint8_t mode);


#endif

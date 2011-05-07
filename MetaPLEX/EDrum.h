#ifndef EDRUM_H
#define EDRUM_H

#include <stdint.h>

#include "hardwareSpecific.h"
#include "profiles/profiles.h"




extern volatile uint8_t ActiveProcess;
extern uint16_t BenchMarkCount;

extern const char VersionId[];

#define SET_BENCHMARK (0)
#define HAS_CONTROLLER_MODE (0)

enum {
   PLAY_MODE = 0,
   SENDING_SYSEX,
   RECEIVE_SYSEX,
   CONTROLLER_MODE,
   FIRMWARE_UPGRADE,
   USB_MIDI_THRU,
} processes;


#define EDRUM_MIDI_CABLE    (0)
#define EXTERNAL_MIDI_CABLE   (1)

#if PROFILE_MEMORY == PROFILE_EEPROM
   #define APP_END (FLASH_TEMP_BUFFER - 1)
#else
   #define APP_END (PROFILE_START - 1)
#endif

#define DEFAULT_PROCESS (PLAY_MODE)

void StartUp(void);
void Shutdown(void);
void Play(void);
void Benchmark(void);
void MidiControllerMode(void);

uint8_t VerifyDownload(void);

/* Interrupt Macros */
#define UI_INT_PORT  P1IN
#define UI_INT_PIN   (1<<3)


#endif

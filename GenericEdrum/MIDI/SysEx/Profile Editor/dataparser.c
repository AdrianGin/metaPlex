
/* Define this so it is no included */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stdint.h"

#include "hardwareSpecific.h"

#include "MIDI/midi.h"
#include "Sample/sample.h"
#include "Sensor/sensor.h"
#include "Profiles/profiles.h"


#define SYSEX_HEADER_SIZE  (4)
#define CSV_FILE (0)
#define SYSEX_FILE (1)

const char outputFile[] = "Output";


const char* MidiSettingsStrings[] = 
{
   "MIDI Output Rate",
   "MIDI Baud Rate",
   "MIDI Channel Code"
};



const char* ChanSettingsStrings[] = 
{
   "Channel Status",
   "Command Ch#",
   "Key/Note Ch#",
   "Threshold Ch#",
   "Retrigger Ch#",
   "Dual Input Map (Status)",
   "Analogue Dual Trigger Map",
   "Alternate Key/Note Ch#"
};

const char* GainSettingsStrings[] =
{
   "Gain Ch#",
   "Gain2 Ch#",
   "Crossover Ch#",
   "Gain Type Map"
};

 

const char* DigitalSettingsStrings[] = {

   "Digital Velocity Ch#",
   "Digital Status Map",
   "Digital Switch Type Map" 
};

		

const char* SensorSettingsStrings[] = {

   "Input Port",
   "Crosstalk Delay"
};


const char* SysExDescriptors[] = {

   "Header Byte",
   "Manufacturer ID",
   "Device ID" 
};



Profile_t SysExData;


typedef struct _SysExHeader
{
	/* Monday to Sunday */
	uint8_t StartByte;
	uint8_t Manufacturer_ID;
	uint8_t Device_ID;
	uint8_t EndByte;
	
} sysex_header_t;


uint8_t ReturnSysExByte(uint8_t Bit, uint8_t data)
{
   return (data | (Bit << 7));  
}


uint32_t GetCSVValue(char* string)
{
   uint32_t value = 0;
   char* Tokenptr;
   
   Tokenptr = strrchr(string, ',') + 1;
   sscanf(Tokenptr, "%lX", &value);

   return value;
}


/* Puts value into 'data' */
uint32_t GetData(FILE* dataFile)
{
  	uint8_t inputBuffer[51];
   fgets(inputBuffer, 50, dataFile );
   return GetCSVValue(inputBuffer);
}



void SysexWrite(Profile_t* data, FILE* output)
{
   uint16_t i = 0;
   uint8_t* buffer = (uint8_t*)data;
   uint8_t  outbyte = 0;
   fputc(MIDI_SYSEX_START, output);
   fputc(MIDI_MANUFACTURER, output);
   fputc(MIDI_DEVICE_CODE, output);
   

	while( i++ < sizeof(Profile_t) )
	{
      /* Ensure all bytes sent are less than 128 or 0x7F */
      if( (*buffer & 0x80) )
      {
         fputc(1, output);
      }
      else
      {
         fputc(0, output);   
      }
      
      
      fputc( (*buffer++) & 0x7F, output );
      //printf("%d\n", *buffer);
	}

   fputc(MIDI_SYSEX_STOP, output);
}

int main(int argc, char *argv[])
{

	sysex_header_t header;
	
	FILE* dataFile;
	FILE* outputFile;
	
	char	inputFileName[20];
	char  inputFileType[5];
	char  outputFileName[20] = "Output";
	unsigned char  inputBuffer[5000];
	int	i;
	int   j;
	int fileType;
	char* Tokenptr;
	uint8_t* ptr;
   uint32_t value;	

	if( argc != 2 )
	{
      printf("Usage: dataparser.exe [input file]\n"
             "input file: A .syx or .csv file\n\n"
             "A syx file will produce an output.csv and a csv input file will produce an output.syx file");
             
      return;
   }
	
	strcpy(inputFileType, &argv[1][ strlen(argv[1]) -3 ]);
	
	if( strcmp(inputFileType, "csv") != 0 && strcmp(inputFileType, "syx") != 0 )
	{
      printf("Unknown file type!\n");   
      return;
   }
	
	if( strcmp(inputFileType, "csv") == 0 )
	{
      printf("You entered a Comma Separated File!\n");
      fileType = CSV_FILE;
      strcat(outputFileName,".syx");
   }
   
   if( strcmp(inputFileType, "syx") == 0 )
	{
      printf("You entered a System Exculsive file!\n");
      fileType = SYSEX_FILE;
      strcat(outputFileName,".csv");
   }
	
	printf("Attempting to open: %s\n", argv[1]);
	dataFile = fopen(argv[1], "r");
	if( dataFile == 0 )
	{
		printf("File not found!\n");
		return 1;
	} 
	
	printf("Parsing data..........\n"
			 "Creating Output file: %s\n", outputFileName);
	
	outputFile = fopen(outputFileName, "w+b");
	if( outputFile == 0 )
	{
		printf("Could not create output file!\n");
		return 1;
	}
	
	printf("Reading Data....\n");	

	//rewind(dataFile);
	/* Read the first 4 bytes into the date struct */
	
	switch( fileType )
	{
      /* Writes a SYSEX File given the CSV File */
      case CSV_FILE:
         
         ptr = (uint8_t*)&SysExData;
         
         //while( fgets(inputBuffer, 50, dataFile )  )
         //{
         header.StartByte = (uint8_t)GetData(dataFile);
         header.Manufacturer_ID = (uint8_t)GetData(dataFile);
         header.Device_ID  = (uint8_t)GetData(dataFile);

         SysExData.midiSettings.MIDI_OutputRate = (uint16_t)GetData(dataFile);
         SysExData.midiSettings.MIDI_BaudRate = (uint16_t)GetData(dataFile);
         SysExData.midiSettings.MIDI_ChannelCode = (uint16_t)GetData(dataFile);

         SysExData.channelSettings.ChannelStatus = (uint32_t)GetData(dataFile);
      	
      	for( i = 0 ; i < NUMBER_OF_INPUTS; i++ )
      	{  
         	SysExData.channelSettings.ChannelCommand[i] = (uint8_t)GetData(dataFile);
         }
      	
      	for( i = 0 ; i < NUMBER_OF_INPUTS; i++ )
      	{   
         	SysExData.channelSettings.ChannelKey[i] = (uint8_t)GetData(dataFile);
         }
         
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	SysExData.channelSettings.ChannelThreshold[i] = (uint16_t)GetData(dataFile);  
         }
         
         for( i = 0 ; i < NUMBER_OF_INPUTS; i++)
         {
           	SysExData.channelSettings.ChannelRetrigger[i] = (uint8_t)GetData(dataFile);  
         }   
      
      	SysExData.channelSettings.HasDualInput  = (uint16_t)GetData(dataFile);  
      
         for( i = 0 ; i < (ANALOGUE_INPUTS/8); i++)
         {
           	SysExData.channelSettings.AnalogueTrigger[i] = (uint32_t)GetData(dataFile);   
         }      
      
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	SysExData.channelSettings.ClosedChannelKey[i] = (uint8_t)GetData(dataFile);   
         }
         
         
         /* Gain Settings */
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	SysExData.gainSettings.ChannelGain[i] = (uint8_t)GetData(dataFile);   
         }   
      
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	SysExData.gainSettings.ChannelGain2[i] = (uint8_t)GetData(dataFile);   
         }   
         
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	SysExData.gainSettings.Crossover[i] = (uint16_t)GetData(dataFile);  
         }      
      
      	SysExData.gainSettings.GainType = (uint16_t)GetData(dataFile);      
      
         /* Digital and Metronome Settings */
         for( i = 0 ; i < (DIGITAL_INPUTS + METRONOME_INPUTS); i++)
         {
           	SysExData.digitalSettings.DigitalVelocity[i] = (uint8_t)GetData(dataFile);  
         }   
      
      	SysExData.digitalSettings.DigitalActiveState = (uint8_t)GetData(dataFile);
      
      	SysExData.digitalSettings.DigitalTriggerMode = (uint8_t)GetData(dataFile);  
      		
      	/* Sensor Settings */
      	SysExData.sensorSettings.SensorInputPort = (uint16_t)GetData(dataFile);  
      
      	SysExData.sensorSettings.CrosstalkDelay = (int16_t)GetData(dataFile);     

         SysexWrite(&SysExData, outputFile);
         //}
         
         //fputc(value, outputFile);
         
      break;
      
      /* Makes a CSV given a SysEx File */
      case SYSEX_FILE:
      	fread((&header), sizeof(char), sizeof(sysex_header_t) - 1, dataFile);
         /* Parse Settings */
         /* Multiply by two since each byte is sent as 2 8 bit bytes */
         fread( &inputBuffer,  sizeof(char), sizeof(Profile_t) * 2, dataFile);
         ptr = (uint8_t*)(&SysExData);
         for( i = 0; i < sizeof(Profile_t) * 2; i = i + 2)
         {
            (*ptr++) =  ReturnSysExByte(inputBuffer[i], inputBuffer[i+1]);
         }
      	
         	
      	fprintf(outputFile, "%s,%X\n",
      		SysExDescriptors[0], header.StartByte);
      	
      	fprintf(outputFile, "%s,%X\n",
      		SysExDescriptors[1], header.Manufacturer_ID);
            
      	fprintf(outputFile, "%s,%X\n",
      		SysExDescriptors[2], header.Device_ID);      	
         
         
      	fprintf(outputFile, "%s,%X\n",
      		MidiSettingsStrings[0], SysExData.midiSettings.MIDI_OutputRate);
      	
      	fprintf(outputFile, "%s,%X\n",
      		MidiSettingsStrings[1], SysExData.midiSettings.MIDI_BaudRate);
            
      	fprintf(outputFile, "%s,%X\n",
         	MidiSettingsStrings[2], SysExData.midiSettings.MIDI_ChannelCode);      
         
         /* Channel Settings */
      	fprintf(outputFile, "%s,%X\n",
      		ChanSettingsStrings[0], SysExData.channelSettings.ChannelStatus);
      	
      	for( i = 0 ; i < NUMBER_OF_INPUTS; i++ )
      	{   
         	fprintf(outputFile, "%s%d,%X\n",
         		ChanSettingsStrings[1], i+1 , SysExData.channelSettings.ChannelCommand[i]);
         }
      	
      	for( i = 0 ; i < NUMBER_OF_INPUTS; i++ )
      	{   
         	fprintf(outputFile, "%s%d,%X\n",
         		ChanSettingsStrings[2], i+1 , SysExData.channelSettings.ChannelKey[i]);
         }
         
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	fprintf(outputFile, "%s%d,%X\n",
         	  ChanSettingsStrings[3], i+1, SysExData.channelSettings.ChannelThreshold[i]);  
         }
         
         for( i = 0 ; i < NUMBER_OF_INPUTS; i++)
         {
           	fprintf(outputFile, "%s%d,%X\n",
         	  ChanSettingsStrings[4], i+1, SysExData.channelSettings.ChannelRetrigger[i]);  
         }   
      
      	fprintf(outputFile, "%s,%X\n",
      		ChanSettingsStrings[5], SysExData.channelSettings.HasDualInput);   
      
         for( i = 0 ; i < (ANALOGUE_INPUTS/8); i++)
         {
           	fprintf(outputFile, "%s,%X\n",
         	  ChanSettingsStrings[6], (SysExData.channelSettings.AnalogueTrigger[i]) );  
         }      
      
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	fprintf(outputFile, "%s%d,%X\n",
         	  ChanSettingsStrings[7], i+1, SysExData.channelSettings.ClosedChannelKey[i]);  
         }
         
         
         /* Gain Settings */
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	fprintf(outputFile, "%s%d,%X\n",
         	  GainSettingsStrings[0], i+1, SysExData.gainSettings.ChannelGain[i]);  
         }   
      
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	fprintf(outputFile, "%s%d,%X\n",
         	  GainSettingsStrings[1], i+1, SysExData.gainSettings.ChannelGain2[i]);  
         }   
         
         for( i = 0 ; i < ANALOGUE_INPUTS; i++)
         {
           	fprintf(outputFile, "%s%d,%X\n",
         	  GainSettingsStrings[2], i+1, SysExData.gainSettings.Crossover[i]);  
         }      
      
      	fprintf(outputFile, "%s,%X\n",
      		GainSettingsStrings[3], SysExData.gainSettings.GainType);      
      
         /* Digital and Metronome Settings */
         for( i = 0 ; i < (DIGITAL_INPUTS + METRONOME_INPUTS); i++)
         {
           	fprintf(outputFile, "%s%d,%X\n",
         	  DigitalSettingsStrings[0], i+1, SysExData.digitalSettings.DigitalVelocity[i]);  
         }   
      
      	fprintf(outputFile, "%s,%X\n",
      		DigitalSettingsStrings[1], SysExData.digitalSettings.DigitalActiveState);
      
      	fprintf(outputFile, "%s,%X\n",
      		DigitalSettingsStrings[2], SysExData.digitalSettings.DigitalTriggerMode);  
      		
      	/* Sensor Settings */
      	fprintf(outputFile, "%s,%X\n",
      		SensorSettingsStrings[0], SysExData.sensorSettings.SensorInputPort);
      
      	fprintf(outputFile, "%s,%X\n",
      		SensorSettingsStrings[1], SysExData.sensorSettings.CrosstalkDelay);    
      
      	fprintf(outputFile, "%s,%X\n",
      		"End Byte", 0xF7);    
      break;  
      
   }

   

	printf("Closing files...\n");
	
	fclose(outputFile);
	fclose(dataFile);
	
	printf("Done!\n"
			 "Parsing Completed! (%d bytes)\n", sizeof(Profile_t)*2);	

	return 0;	
}


























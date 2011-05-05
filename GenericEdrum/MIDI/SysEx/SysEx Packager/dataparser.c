#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


#define SYSEX_HEADER_SIZE  (4)
#define CSV_FILE (0)
#define SYSEX_FILE (1)

const char outputFile[] = "Output";


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



void SysexWrite(void* data, uint32_t len, FILE* output)
{
   uint32_t i = 0;
   int buffer;
   uint8_t  outbyte = 0;
   fputc(0xF0, output);
   fputc(0x7D, output);
   fputc(0x01, output);
   

	while( i++ < len )
	{
      buffer = fgetc( (FILE*)data);
      /* Ensure all bytes sent are less than 128 or 0x7F */
      if( (buffer & 0x80) )
      {
         fputc(1, output);
      }
      else
      {
         fputc(0, output);   
      }
 
      fputc( (buffer) & 0x7F, output );
      //printf("%d\n", *buffer);
	}

   fputc(0xF7, output);
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

	if( !(argc > 2) || !(argc < 4) )
	{
      printf("Usage: dataparser.exe [input file] [outputfile] [minimum size (hex)\n"
             "input file: file to be converted to SysEx\n\n"
             "output file: in binary\n"
             "minimum size: in Hex eg 0xFFFF, will be padded with zeros"
             "F0 7D 01 ... F7");
             
      return;
   }
	
	strcpy(inputFileType, &argv[1][ strlen(argv[1]) -3 ]);
	
	printf("Attempting to open: %s\n", argv[1]);
	dataFile = fopen(argv[1], "rb");
	if( dataFile == 0 )
	{
		printf("File not found!\n");
		return 1;
	} 
	
    strcpy(outputFileName, argv[2]);
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
	
	uint32_t size; 
	uint32_t finalSize;
	
    {
       fseek(dataFile, 0, SEEK_END); // seek to end of file
       size = ftell(dataFile); // get current file pointer
    }
    fseek(dataFile, 0, SEEK_SET); // seek back to beginning of file

    SysexWrite(dataFile, size, outputFile);

	if( argc > 3)
	{
       if( argv[3] )
       sscanf(argv[3], "%X", &finalSize);
    }
    
    
   

	printf("Closing files...\n");
	
	fclose(outputFile);
	fclose(dataFile);
	
	printf("Done!\n"
			 "Parsing Completed! (%d bytes)\n", size);	

	return 0;	
}


























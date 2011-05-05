
#include <stdint.h>


uint8_t MSB2LSB(uint8_t data)
{
   uint8_t formattedData = 0;
   uint8_t i;
 
   for( i = 0; i < 8; i++)
   {
  
      formattedData = formattedData << 1;      
      if( data & (0x01 << i) )
      {
         formattedData = (formattedData + 0x01);
      }
   }
   return formattedData;
}


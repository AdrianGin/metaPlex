
#include <avr/io.h>
#include <util/delay.h>


int main(void)
{

   DDRD |= (1 << 4);
   DDRD |= (1 << 5);
   DDRD |= (1 << 6);
   DDRD |= (1 << 7);

   while(1)
   {
      _delay_ms(50);
      PORTD ^= (1<<5);
      _delay_ms(50);
      PORTD ^= (1<<4);
      _delay_ms(50);
      PORTD ^= (1<<6);
      _delay_ms(50);
      PORTD ^= (1<<7);
   }


}

/*

Copyright (c) 2011 Adrian Gin (adrian.gin[at]gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <stdint.h>
#include <string.h>
#include "Menu.h"
#include "UI_KP/UI_KP.h"
#include "hardwareSpecific.h"




/** Updates the state of the menu, if RESET_MENU is passed to options, the screen is wiped.
                                      HIDE_CHILDREN is passed to options, the submenu items are displayed */
void MenuUpdate(Menu_t* menu, uint8_t options)
{
   uint8_t i;   
   uint8_t sequenceIndex = 0;
   uint8_t MenuOffset;
   uint8_t MenuMax;
   
   uint8_t scrollUp = 0;
   uint8_t scrollDown = 0;
   
   uint8_t windowSize = menu->windowSize;
   
   char* outputString;
   
   if( options & RESET_MENU )
   {
      menu->MenuReset();
   }
      

   /** Only switch Menu input IF we are in a menu item which has NO associated
    * function */
   if( MENU_GET_PTR(menu->structure[ GetIndex(menu, menu->currentState) ].function) == 0 )
   {
      stateMachine(menu, menu->currentState ); 
   }
        
   /** Run the associated function if it has one */   
   if( !(options & NO_EXECUTE) )
   {
      executeState(menu, menu->currentState);  
   }
      
   

   if( !(menu->updateOptions & HIDE_CHILDREN) )
   {

      MenuOffset = SmallestSequence(menu, menu->currentState, IGNORE_INVISIBLE_STATES);
      MenuMax = LargestSequence(menu, menu->currentState, IGNORE_INVISIBLE_STATES);
   
      //menu->MenuSetPos(0, 0); 
      /** Print out the menu's sub-menu's */   
      /* Ensures that the screen limits are not exceeded */
      for( i = MenuOffset, sequenceIndex = 0;  MENU_GET_BYTE(menu->states[i].parent) != 0 ; i++)
      {  
         
         /* Find the current state's sub children. */        
         if( MENU_GET_BYTE(menu->states[i].parent) == menu->currentState )
         {  
            sequenceIndex = MENU_GET_BYTE(menu->states[i].sequence);

      
            /* The upper and lower limits mean that menus have a hysteresis type effect */
            if( menu->selectedItem > menu->upperLimit )
            {
               menu->upperLimit = menu->selectedItem;
               menu->lowerLimit = menu->upperLimit + MenuOffset - windowSize;
            }
         
            if( menu->selectedItem < menu->lowerLimit )
            {
               menu->lowerLimit = menu->selectedItem;
               menu->upperLimit = menu->lowerLimit + windowSize - MenuOffset;               
            }


            if( menu->selectedItem < MenuOffset )
            {  
               menu->lowerLimit = MenuOffset;
               menu->upperLimit = windowSize; 
            }

            if( menu->selectedItem > MenuMax )
            {  
               if( windowSize > (MenuOffset + MenuMax) )
               {
                   menu->lowerLimit = 0;
               }
               else
               {
                  menu->lowerLimit = MenuMax + MenuOffset - windowSize;
               }
               menu->upperLimit = MenuMax; 
            }

      
            /* If this is the selected item then prefix an asterix */
            if( (sequenceIndex <= menu->upperLimit) && (sequenceIndex >= menu->lowerLimit) )
            {

               outputString = MenuDescriptor(menu, MENU_GET_BYTE(menu->states[i].child) );
               if( outputString )
               {
                  if(sequenceIndex > windowSize)
                  {
                     scrollUp = SCROLL_BAR;
                  }
         
                  if( MenuMax - sequenceIndex + MenuOffset  > windowSize)
                  {
                     scrollDown = SCROLL_BAR;
                  }
         
                  if( MENU_GET_BYTE(menu->states[i].sequence) == menu->selectedItem)
                  {
                     menu->MenuChar( '*' );    
                  }
                  else
                  {
                     menu->MenuChar( ' ' );   
                  }     
                
                  menu->MenuPrint_P(outputString);
                  menu->MenuNewLine(); 
               }                  
            }
            
         }  
         
         
      }
      
      
      if( scrollUp == SCROLL_BAR)
      {
         menu->MenuSetPos(0, 19);
         menu->MenuChar( '^' );  
      }
      
      if( scrollDown == SCROLL_BAR)
      {
         menu->MenuSetPos(windowSize, 19);
         /* Down arrow assumed to be in CGRAM 1 */
         menu->MenuChar( 0 );
      }   
   }
   
}


/** Determines the new state depending on the user input and passed state */
void stateMachine(Menu_t* menu, uint8_t state)
{
   uint8_t maxStateItems;
   uint8_t parentIndex = 0;
   
   
   
   maxStateItems = SubItems(menu, state);
   
   

   switch( menu->MenuInput )
   {
      case KB_UP:
      case KP_UP:
         if(!(menu->selectedItem <= SmallestSequence(menu, menu->currentState, COUNT_INVISIBLE_STATES)) )
         {
            menu->selectedItem--;
         }
      break;
         
      case KB_DOWN:   
      case KP_DOWN:
         menu->selectedItem++;          
         if( menu->selectedItem - SmallestSequence(menu, menu->currentState, COUNT_INVISIBLE_STATES) > maxStateItems )
         {
            menu->selectedItem = LargestSequence(menu, menu->currentState, COUNT_INVISIBLE_STATES);
         }
      break;
      
      case KB_ENTER:
      case KP_ENTER:
         /* Go into child sub menu */
         menu->currentState = GetMenuState(menu, menu->currentState, menu->selectedItem);
         menu->selectedItem = SmallestSequence(menu, menu->currentState, COUNT_INVISIBLE_STATES);
         menu->upperLimit = menu->windowSize;
         menu->lowerLimit = 0;
         menu->firstEnter = 1; 
         
      break;
      
      case KB_BACK:
      case KP_BACK: 
         /** Need to reset the 'first' enter flag */
         /** This is so that the commands of a function are not
          * executed on entering the associated sub-menu */
         menu->firstEnter = 1;                
         parentIndex = GetParent(menu, menu->currentState);
         menu->lowerLimit = 0;
         menu->upperLimit = menu->windowSize;
         if(parentIndex != INVALID_STATE)
         {
            menu->upperLimit = menu->windowSize;
            menu->lowerLimit = 0;
            
            menu->currentState = MENU_GET_BYTE(menu->states[ parentIndex ].parent);
            menu->selectedItem = MENU_GET_BYTE(menu->states[ parentIndex ].sequence); 
         }
      break;     
      
      default:
      break;   
   }

   /* Revert back to initial state */
   if(menu->currentState == NO_STATE || parentIndex == INVALID_STATE)
   {
      menu->currentState = state;       
   }
}

/* Returns the Menu descriptor given the menu Item / Index
 * the pointer returned is of type MENU_SPACE_PTR */
char* MenuDescriptor(Menu_t* menu, uint8_t menuItem)
{

   uint8_t index;
   //static char buffer[21];
      
   index = GetIndex(menu, menuItem);
   
   if( index  != INVALID_STATE )
   {
      //strcpy(buffer, MENU_GET_PTR(&menu->structure[index].descriptor));
      return (char *)MENU_GET_PTR(menu->structure[index].descriptor);
   }
   return 0;
}


/* Returns the corresponding menu->states 
 * given the Parent State and its sequnce. */
uint8_t GetMenuState(Menu_t* menu, uint8_t state, uint8_t Sequence)
{
   int i;
   for( i = 0; MENU_GET_BYTE(menu->states[i].parent); i++)
   {
      if( MENU_GET_BYTE(menu->states[i].parent) == state
       && MENU_GET_BYTE(menu->states[i].sequence) == Sequence ){      
         return MENU_GET_BYTE(menu->states[i].child);
      }
   }
   return NO_STATE;
}

/* Get out the first instance of where the given state
 * origninates from */
uint8_t GetParent(Menu_t* menu, uint8_t state)
{
 
   int i;  
   for( i = 0; MENU_GET_BYTE(menu->states[i].parent); i++)
   {
      if( MENU_GET_BYTE(menu->states[i].child) == state) {      
         return i;
      }
   }
   return INVALID_STATE;
}

/* Updates the input last received by the menu */
void MenuSetInput(Menu_t* menu, uint8_t NewInput)
{
   menu->MenuInput = NewInput;    
}

uint8_t MenuGetInput(Menu_t* menu)
{
   return menu->MenuInput;
}

/** Calls the associated menu's function */
void executeState(Menu_t* menu, uint8_t state)
{
   uint8_t index;
   void (*funcPtr)(void* data);
   
   index = GetIndex(menu, state);
   
   if(index != INVALID_STATE)
   {
      if( MENU_GET_PTR(menu->structure[index].function) != 0)
      {
         funcPtr = (void*)MENU_GET_PTR(menu->structure[index].function);
         funcPtr(&menu->MenuInput);
      }
   }
   
   
}

/*& Get the element in MenuData which has 'parent' */
uint8_t GetIndex(Menu_t* menu, uint8_t parent)
{
   uint8_t i;
   for( i = 0; MENU_GET_BYTE(menu->structure[i].menu_item); i++)
   {
      if( MENU_GET_BYTE(menu->structure[i].menu_item) == parent)
      {
         return i;   
      } 
   }
   
   return INVALID_STATE;
   
}

/** Get the number of sub items in the passed state */
uint8_t SubItems(Menu_t* menu, uint8_t state)
{
   int i;
   uint8_t StateItems = 0;    
   for( i = 0; MENU_GET_BYTE(menu->states[i].parent); i++)
   {
      if( MENU_GET_BYTE(menu->states[i].parent) == state)
      {
         StateItems++;
      }
   }
   return (StateItems - 1);
}

/** Get the maximum sequence of the the passed state */
uint8_t LargestSequence(Menu_t* menu, uint8_t state, uint8_t options)
{
   int i;
   uint8_t StateItem = 0;    
   for( i = 0; MENU_GET_BYTE(menu->states[i].parent); i++)
   {
      if( MENU_GET_BYTE(menu->states[i].parent) == state)
      {
         /* Obtain the number of Menu Items in the given state */
         if( (MENU_GET_BYTE(menu->states[i].sequence) >= StateItem) )
         {
            if( options & IGNORE_INVISIBLE_STATES && (MENU_GET_BYTE(menu->states[i].child) >= VISIBLE_STATE))
            {
               StateItem = MENU_GET_BYTE(menu->states[i].sequence);
            }
            
            if( options & COUNT_INVISIBLE_STATES )
            {
               StateItem = MENU_GET_BYTE(menu->states[i].sequence);
            }
            
         } 
      }
   }
   return (StateItem);
}

uint8_t SmallestSequence(Menu_t* menu, uint8_t state, uint8_t options)
{
   int i;
   uint8_t StateItem = 0xFF;    
   for( i = 0; MENU_GET_BYTE(menu->states[i].parent); i++)
   {
      if( MENU_GET_BYTE(menu->states[i].parent) == state)
      {
         /* Obtain the number of Menu Items in the given state */
         if( (MENU_GET_BYTE(menu->states[i].sequence) <= StateItem) )
         {
            if( options & IGNORE_INVISIBLE_STATES && (MENU_GET_BYTE(menu->states[i].child) >= VISIBLE_STATE))
            {
               StateItem = MENU_GET_BYTE(menu->states[i].sequence);
            }
            
            if( options & COUNT_INVISIBLE_STATES )
            {
               StateItem = MENU_GET_BYTE(menu->states[i].sequence);
            }
            
         } 
      }
   }
   return (StateItem);
}


/* Returns the sequence given the menu state */
uint8_t GetSequence(Menu_t* menu, uint8_t parent, uint8_t child)
{
   uint8_t parentIndex;
   uint8_t i;
   
   parentIndex = GetIndex(menu, parent);
   

   for( i = 0; MENU_GET_BYTE(menu->states[i].parent); i++)
   {
      if( MENU_GET_BYTE(menu->states[i].parent) == parent &&
          MENU_GET_BYTE(menu->states[i].child)  == child )
      {
         return MENU_GET_BYTE(menu->states[i].sequence);     
      }
   }      
   return INVALID_SEQUENCE;
}


uint8_t GetState(Menu_t* menu)
{
   return menu->currentState;  
}



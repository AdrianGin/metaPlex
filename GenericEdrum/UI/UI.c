#include <stdint.h>
#include "UI.h"



#if VERSION_CODE == VERSION_WITH_PE
#include "MAX7300/max7300.h"
#endif

/* Turns the UI MAX7300 on and turns on interrupts */
void UI_Activate(void)
{       
#if VERSION_CODE == VERSION_WITH_PE	
   /* Enable Transition detection on Keypad Columns */   
   UI_SetRegister(MAX7300_TRANSITION_MASK, (1 << MAX7300_TM_P28) |
                                           (1 << MAX7300_TM_P27) |
                                           (1 << MAX7300_TM_P26) |
                                           (1 << MAX7300_TM_P25));
	   
   /* Set Config to turn on and Enable Transition detect */
   UI_SetRegister(MAX7300_CONFIG, (1 << MAX7300_SHUTDOWN_CONTROL) | (1 << MAX7300_TRANSITION_ENABLE));
#endif   

#if VERSION_CODE == VERSION_WITHOUT_PE	   
	/* Set Keypad Columns as inputs */
//	UI_KP_DIR &= ~(UI_INT_COL0 | UI_INT_COL1 | UI_INT_COL2 | UI_INT_COL3);
	
	/* Trigger interrupts on a logic high to low transition */
	//UI_INT_IES |= (UI_INT_COL0 | UI_INT_COL1 | UI_INT_COL2 | UI_INT_COL3);
	/* Enable Interrupts */
	//UI_INT_IE |= (UI_INT_COL0 | UI_INT_COL1 | UI_INT_COL2 | UI_INT_COL3);   
#endif   
   
}







/** \file UI.h
	 \brief User Interface Routines
*/

/**	 

	\ingroup edrum
 	\defgroup UI User Interface Module
 	\code #include "UI.h" \endcode

 	 
	 \par Description
    Sets up the User Interface Module for interrupts
*/
//@{

#ifndef	_UI_ROUTINES
#define	_UI_ROUTINES


#include "hardwareSpecific.h"
#include "version.h"

#define UI_INT_IFG	P1IFG

#if VERSION_CODE == VERSION_WITH_PE

#include "MAX7300/max7300.h"

#define UI_MAX7300_ADDRESS   (0x80)
#define UI_HIGH   (1)
#define UI_LOW    (0)
#define UI_INVALID_TRANSITION (8)


/* Interrupt Inputs */
#define UI_INTERRUPT    (MAX7300_P31)

#define UI_RTC_INT0     (MAX7300_TM_P29)
#define UI_RTC_INT1     (MAX7300_TM_P30)

#define UI_INT_COL0  (MAX7300_TM_P28)
#define UI_INT_COL1  (MAX7300_TM_P27)
#define UI_INT_COL2  (MAX7300_TM_P26)
#define UI_INT_COL3  (MAX7300_TM_P25)



#define UI_ReadRegister(addr) MAX7300_ReadRegister(UI_MAX7300_ADDRESS, addr)
#define UI_SetRegister(addr, data) MAX7300_SetRegister(UI_MAX7300_ADDRESS, addr, data)
#define UI_SetBlock(data, bytes) MAX7300_SetBlock_C(UI_MAX7300_ADDRESS, data, bytes)
#define UI_EnableInterrupt() UI_SetRegister(MAX7300_CONFIG, (1 << MAX7300_SHUTDOWN_CONTROL) | (1 << MAX7300_TRANSITION_ENABLE))


#endif

/* Without the use of a middleman port expander */
#if VERSION_CODE == VERSION_WITHOUT_PE

#define UI_KP_PIN	P1IN
#define UI_KP_DIR	P1DIR
#define UI_INT_IES	P1IES
#define UI_INT_IE		P1IE

#define UI_INT_COL0  (1<<4)
#define UI_INT_COL1  (1<<5)
#define UI_INT_COL2  (1<<6)
#define UI_INT_COL3  (1<<7)
#endif // Without the use of MAX7300



void UI_Activate(void);

#endif

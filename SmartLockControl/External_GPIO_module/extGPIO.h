#ifndef EXT_GPIO_H
#define EXT_GPIO_H


#include <linux/ctype.h>


enum{
	RED_LED,
	GREEN_LED,
	LOCK_OPEN_LED,
	LOCK_CLOSE_LED,
	Leds_Amount
};

enum {
	MODE_ON,
	MODE_OFF
};

enum {
	STATE_UNLOCK,
	STATE_LOCK,
};


/**
* @brief Function for set state of lock
* @param state - statee which should be set
*/
void SetLockState(u8 state);

/**
* @brief Function for take state of lock
* @return current state of lock
*/
u8   GetLockState(void);

/**
* @brief Function for set state of led
* @param led  - name of necesssary led
* @param mode - mode which should be set
* @return result of operation
*/
u8   SetLedMode(u8 led, u8 mode);

/**
* @brief Function for get number of button irq
* @return value of button irq
*/
int  GetInterruptCount(void);




#endif
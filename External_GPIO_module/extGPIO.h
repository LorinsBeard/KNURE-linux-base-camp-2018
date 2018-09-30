#ifndef EXT_GPIO_H
#define EXT_GPIO_H


#include <linux/ctype.h>


enum{
	RED_LED,
	GREEN_LED,
	Leds_Amount
};

enum {
	MODE_OFF,
	MODE_ON
};

enum {
	STATE_LOCK,
	STATE_UNLOCK
};



void SetLockState(u8 state);

u8   GetLockState(void);

u8   SetLedMode(u8 led, u8 mode);

int  GetInterruptNumber(void);






#endif
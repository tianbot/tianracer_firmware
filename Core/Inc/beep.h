#ifndef _BEEP_H_
#define _BEEP_H_

#include "stm32f4xx_hal.h"

void BeepOn(uint8_t sound);
void BeepOff(void);
void BeepInit(void);
#endif

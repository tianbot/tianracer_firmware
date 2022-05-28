#ifndef _BEEP_TASK_H_
#define _BEEP_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stdarg.h"

typedef struct{
	uint16_t sound;
	uint16_t time;
} __PACKED BeepMsg_t;

#define BEEP_MSG_QUENE_SIZE 10

extern osMailQId BeepMail;

extern osThreadId BeepTaskHandle;

void BeepTaskInit(void);
void Beep(uint16_t sound, uint16_t time);
#endif

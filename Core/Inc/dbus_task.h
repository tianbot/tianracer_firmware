#ifndef _DBUS_TASK_H_
#define _DBUS_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define DBUS_MSG_LEN	20

#define DBUS_MSG_QUENE_SIZE 16
#define CTRL_MSG_QUENE_SIZE 32

#define CTRL_TYPE_DBUS_RC 0
#define CTRL_TYPE_PC      1
#define CTRL_TYPE_MAG     2

#define RACECAR_MAX_SPEED 1800
#define RACECAR_MIN_SPEED 1200
#define RACECAR_MAX_OMEGA 120
#define RACECAR_MIN_OMEGA 60

typedef struct{
	uint8_t Msg[DBUS_MSG_LEN];
	uint16_t MsgLen;
}DbusMsg_t;

extern osMailQId DbusMail;
extern osMailQId CtrlMail;

extern DbusMsg_t *pDbusMsg;

extern volatile uint32_t CtrlFlag;

extern osThreadId DbusTaskHandle;

void DbusTaskInit(void);

#endif

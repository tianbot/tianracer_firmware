#ifndef _DBUS_H_
#define _DBUS_H_

#include "stm32f4xx_hal.h"

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define DBUS_TIMEOUT 20

#define RC_MIN_CTRL 5

/* -------------------------- RC Switch Definition RC Switch Definition RC Switch Definition RC Switch Definition----------------*/
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition PC Key Definition PC Key Definition ------------------*/
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01 << 7)
#define RC_FRAME_LENGTH 18u

/* -----------------------Data Struct Data Struct ------------------------------------- */
#pragma pack(push)
#pragma pack(1)
typedef struct
{
  struct
  {
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
    uint8_t s1;
    uint8_t s2;
  } rc;
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
  } mouse;
  struct
  {
    uint16_t v;
  } key;
} RC_Ctl_t;

typedef struct
{
  float vx;
  float vy;
  float w;
  float steering_angle;
} MotionCtrl_t;

#pragma pack(pop)

void RemoteDataProcess(uint8_t *pData, RC_Ctl_t *pCtrlData);

#endif

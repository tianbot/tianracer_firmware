#ifndef _PID_H_
#define _PID_H_

#include "stm32f4xx_hal.h"

enum
{
  LLAST = 0,
  LAST = 1,
  NOW = 2,

  POSITION_PID,
  DELTA_PID,
};

typedef struct __pid_t
{
  float p;
  float i;
  float d;

  float set[3]; //???,??NOW, LAST, LLAST???
  float get[3]; //???
  float err[3]; //??

  float pout; //p??
  float iout; //i??
  float dout; //d??

  float pos_out;      //???????
  float last_pos_out; //????
  float delta_u;      //?????
  float delta_out;    //??????? = last_delta_out + delta_u
  float last_delta_out;

  float max_err;
  float deadband; //err < deadband return
  uint32_t pid_mode;
  uint32_t MaxOutput;     //????
  uint32_t IntegralLimit; //????
} Pid_t;

void PidInit(
    Pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd);

float PidCalc(Pid_t *pid, float get, float set);

#endif

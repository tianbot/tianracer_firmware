#ifndef _PARAM_H_
#define _PARAM_H_

#include "stm32f4xx_hal.h"

#define PARAM_HEAD 0xaa5555aa
#define PARAM_TAIL 0x55aaaa55

#ifndef PI
#define PI 3.1415926f
#endif

#define SIN_60 0.8660254f
#define COS_60 0.5f

#pragma pack(push)
#pragma pack(1)

/******************************
//              ^^
//              ^^
//              ||
//
//           \    /|
//            \  / |
//          A---   |B
//            /  \
//           /    \
*******************************/
typedef struct
{
  float base_a; //width/2 left to center
  float base_b; //length/2 front to center
  float pwm_dead_zone;
  float max_steer_angle;
} RacecarBaseParam_t;

typedef struct
{
  float p;
  float i;
  float d;
  float max_out;
  float i_limit;
} PidParam_t;

typedef struct
{
  uint32_t param_head;
  float wheel_r;
  float motor_reduction_ratio;
  float max_w;
  float max_speed;
  RacecarBaseParam_t racecar;
  PidParam_t pid;
  uint16_t ticks_per_lap;
  int32_t max_ticks;
  int ctrl_period;
  int feedback_period;
  int pose_calc_period;
  float az_off;
  int servo_dir;
  uint32_t param_tail;
} Param_t;
#pragma pack(pop)

extern Param_t param;
extern const Param_t DefaultParam;

#define PARAM_SAVE_ADDR 0x080E0000

void InitParam(void);
void SaveParam(void *p);
#endif

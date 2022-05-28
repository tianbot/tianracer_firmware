#ifndef _RACECAR_TASK_H_
#define _RACECAR_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#ifndef PI
#define PI 3.1415926 
#endif

#define LIMIT(x, a, b) (((x)<(a)) ? (a) : (((x)>(b)) ? (b) : (x)))

#define RACECAR_SPEED_ZERO 1500
#define RACECAR_STEER_ANGLE_ZERO 90

#define MID_STEER_ANGLE 90

#define SERVO_CAL(X) ((2500 - 500)*(180-X)/180+500)
#define ANGLE_CAL(X) (-((X)-1500)/1000.0*PI/2.0)
#define MOTOR_CAL(X) (X + RACECAR_SPEED_ZERO)

#define MOTOR_MAX 2000
#define MOTOR_MIN 1000

#define RACECAR_CTRL_TIMEOUT  1000

#define STALL_OR_ENCODER_ERROR_PWM 50
#define MINIMAL_V 0.1

#define MOVE_DIR_FORWARD 0
#define MOVE_DIR_BACKWARD 1

void RacecarTaskInit(void);

#endif

#include "pid.h"
#include "string.h"

#define ABS(x) ((x > 0) ? (x) : (-x))

void abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}

void PidInit(
    Pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd)
{
  memset(pid, 0, sizeof(Pid_t));
  pid->IntegralLimit = intergral_limit;
  pid->MaxOutput = maxout;
  pid->pid_mode = mode;

  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
}

float PidCalc(Pid_t *pid, float get, float set)
{
  pid->get[NOW] = get;
  pid->set[NOW] = set;
  pid->err[NOW] = set - get;

  if (pid->pid_mode == POSITION_PID)
  {
    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
    abs_limit(&(pid->iout), pid->IntegralLimit);
    pid->pos_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->pos_out), pid->MaxOutput);
    pid->last_pos_out = pid->pos_out;
  }
  else if (pid->pid_mode == DELTA_PID)
  {
    pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
    pid->iout = pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

    abs_limit(&(pid->iout), pid->IntegralLimit);
    pid->delta_u = pid->pout + pid->iout + pid->dout;
    pid->delta_out = pid->last_delta_out + pid->delta_u;
    abs_limit(&(pid->delta_out), pid->MaxOutput);
    pid->last_delta_out = pid->delta_out;
  }

  pid->err[LLAST] = pid->err[LAST];
  pid->err[LAST] = pid->err[NOW];
  pid->get[LLAST] = pid->get[LAST];
  pid->get[LAST] = pid->get[NOW];
  pid->set[LLAST] = pid->set[LAST];
  pid->set[LAST] = pid->set[NOW];
  return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

#include "protocol_task.h"
#include "stdlib.h"
#include "param.h"
#include "stdio.h"

void pid_calc_auto(int argc, char *argv[], char *ret, uint16_t max_ret_len)
{
    snprintf(ret, max_ret_len, "cmd not support yet\r\n");
    return;
}

void az_offset(int argc, char *argv[], char *ret, uint16_t max_ret_len)
{
    if (argc != 2)
    {
        snprintf(ret, max_ret_len, "usage:\r\nz_offset 0.123\r\n");
        return;
    }
    param.az_off = atof(argv[1]);
    SaveParam(&param);
    snprintf(ret, max_ret_len, "accel z-axis offset set to %f\r\n", param.az_off);
}

void servo_dir(int argc, char *argv[], char *ret, uint16_t max_ret_len)
{
    if (argc != 2)
    {
        snprintf(ret, max_ret_len, "usage:\r\nservo_dir 0/1\r\n");
        return;
    }
    param.servo_dir = atoi(argv[1]);
    SaveParam(&param);
    snprintf(ret, max_ret_len, "servo dir set to %d\r\n", param.servo_dir);
}

void max_speed(int argc, char *argv[], char *ret, uint16_t max_ret_len)
{
    if (argc != 2)
    {
        snprintf(ret, max_ret_len, "usage:\r\nmax_speed 0.123\r\n");
        return;
    }
    param.max_speed = atof(argv[1]);
    SaveParam(&param);
    snprintf(ret, max_ret_len, "max speed set to %f\r\n", param.max_speed);
}

void get_type(int argc, char *argv[], char *ret, uint16_t max_ret_len)
{
    // if (argc != 2)
    // {
    //     snprintf(ret, max_ret_len, "\r\n");
    //     return;
    // }
    // param.max_speed = atof(argv[1]);
    // SaveParam(&param);
    snprintf(ret, max_ret_len, "base_type: ackermann\r\n");
}

ADD_DEBUG_CMD("az_offset", az_offset, "set accel z-axis offset.")
ADD_DEBUG_CMD("servo_dir", servo_dir, "set servo direction.")
ADD_DEBUG_CMD("max_speed", max_speed, "set max speed.")
ADD_DEBUG_CMD("param", get_type, "get type.")

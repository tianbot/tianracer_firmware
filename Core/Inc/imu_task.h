#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

#include "stm32f4xx_hal.h"
#include "bmi088.h"
#include "ist8310.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS        2


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100ԭʼ�����ڻ�����buf��λ��
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //�¶ȿ���PID��kp
#define TEMPERATURE_PID_KI 0.2f    //�¶ȿ���PID��ki
#define TEMPERATURE_PID_KD 0.0f    //�¶ȿ���PID��kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //�¶ȿ���PID��max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //�¶ȿ���PID��max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ MPU6500_TEMP_PWM_MAX - 1


#define IMU_TASK_INIT_TIME 10 //����ʼ���� delay һ��ʱ��

#define IMU_YAW_ADDRESS_OFFSET    0
#define IMU_PITCH_ADDRESS_OFFSET  1
#define IMU_ROLL_ADDRESS_OFFSET   2

#define IMU_GYRO_X_ADDRESS_OFFSET 0
#define IMU_GYRO_Y_ADDRESS_OFFSET 1
#define IMU_GYRO_Z_ADDRESS_OFFSET 2

#define IMU_ACCEL_X_ADDRESS_OFFSET 0
#define IMU_ACCEL_Y_ADDRESS_OFFSET 1
#define IMU_ACCEL_Z_ADDRESS_OFFSET 2

#define IMU_MAG_X_ADDRESS_OFFSET 0
#define IMU_MAG_Y_ADDRESS_OFFSET 1
#define IMU_MAG_Z_ADDRESS_OFFSET 2

extern bmi088_real_data_t bmi088_real_data;
extern ist8310_real_data_t ist8310_real_data;


extern float imu_quat[4];
extern float imu_angle[3]; //euler angle, unit rad.ŷ���� ��λ rad

void ImuTaskInit(void);
#endif

#include "imu_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bmi088.h"
#include "ist8310.h"
#include "pid.h"
#include "protocol_task.h"
#include "MahonyAHRS.h"
#include "math.h"
#include "spi.h"
#include "tim.h"
/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(float temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
void get_angle(float quat[4], float *yaw, float *pitch, float *roll);

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

osThreadId ImuTaskHandle;
osThreadId ImuFeedbackTaskHandle;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;

static uint8_t first_temperate;
static Pid_t imu_temp_pid;

float imu_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float imu_angle[3] = {0.0f, 0.0f, 0.0f}; //euler angle, unit rad.ŷ���� ��λ rad

void IMU_temp_PWM(uint16_t tempPWM)
{
  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, tempPWM);
}

void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);

  __HAL_SPI_ENABLE(&hspi1);

  //disable DMA
  //ʧЧDMA
  __HAL_DMA_DISABLE(&hdma_spi1_rx);

  while (hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_LISR_TCIF2);

  hdma_spi1_rx.Instance->PAR = (uint32_t) & (SPI1->DR);
  //memory buffer 1
  //�ڴ滺����1
  hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);
  //data length
  //���ݳ���
  __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);

  __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);

  //disable DMA
  //ʧЧDMA
  __HAL_DMA_DISABLE(&hdma_spi1_tx);

  while (hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);

  hdma_spi1_tx.Instance->PAR = (uint32_t) & (SPI1->DR);
  //memory buffer 1
  //�ڴ滺����1
  hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
  //data length
  //���ݳ���
  __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);
}

void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
  //disable DMA
  //ʧЧDMA
  __HAL_DMA_DISABLE(&hdma_spi1_rx);
  __HAL_DMA_DISABLE(&hdma_spi1_tx);
  while (hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
  }
  while (hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
  }
  //clear flag
  //�����־λ
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));

  __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));
  //set memory address
  //�������ݵ�ַ
  hdma_spi1_rx.Instance->M0AR = rx_buf;
  hdma_spi1_tx.Instance->M0AR = tx_buf;
  //set data length
  //�������ݳ���
  __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
  __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);
  //enable DMA
  //ʹ��DMA
  __HAL_DMA_ENABLE(&hdma_spi1_rx);
  __HAL_DMA_ENABLE(&hdma_spi1_tx);
}

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
static void ImuTaskEntry(void const *pvParameters)
{
  //wait a time
  osDelay(IMU_TASK_INIT_TIME);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);//temperature ctrl
  while (BMI088_init())
  {
    osDelay(100);
  }
  while (ist8310_init())
  {
    osDelay(100);
  }

  BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

  PidInit(&imu_temp_pid, POSITION_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT, TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD);

  AHRS_init(imu_quat, bmi088_real_data.accel, ist8310_real_data.mag);

  //set spi frequency
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

  imu_start_dma_flag = 1;

  while (1)
  {
    //wait spi DMA tansmit done
    //�ȴ�SPI DMA����
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
    {
    }

    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
    }

    if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
    }

    if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
      imu_temp_control(bmi088_real_data.temp);
    }

    AHRS_update(imu_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);
    get_angle(imu_quat, imu_angle + IMU_YAW_ADDRESS_OFFSET, imu_angle + IMU_PITCH_ADDRESS_OFFSET, imu_angle + IMU_ROLL_ADDRESS_OFFSET);
  }
}

void AHRS_init(float quat[4], float accel[3], float mag[3])
{
  quat[0] = 1.0f;
  quat[1] = 0.0f;
  quat[2] = 0.0f;
  quat[3] = 0.0f;
}

void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
  MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}
void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
  *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
  *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
  *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(float temp)
{
  uint16_t tempPWM;
  static uint8_t temp_constant_time = 0;
  if (first_temperate)
  {
    PidCalc(&imu_temp_pid, temp, 45.0f);
    if (imu_temp_pid.pos_out < 0.0f)
    {
      imu_temp_pid.pos_out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid.pos_out;
    IMU_temp_PWM(tempPWM);
  }
  else
  {
    //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
    //in beginning, max power
    if (temp > 45.0f)
    {
      temp_constant_time++;
      if (temp_constant_time > 200)
      {
        //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
        //
        first_temperate = 1;
        imu_temp_pid.iout = MPU6500_TEMP_PWM_MAX / 2.0f;
      }
    }

    IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
  }
}

void ImuFeedbackTaskEntry(void const *argument)
{
  /* USER CODE BEGIN ImuTaskEntry */
  struct imu_feedback imu_feedback;
  /* Infinite loop */
  for (;;)
  {
    imu_feedback.quat.w = imu_quat[0];
    imu_feedback.quat.x = imu_quat[1];
    imu_feedback.quat.y = imu_quat[2];
    imu_feedback.quat.z = imu_quat[3];
    imu_feedback.linear_acc.x = bmi088_real_data.accel[0];
    imu_feedback.linear_acc.y = bmi088_real_data.accel[1];
    imu_feedback.linear_acc.z = bmi088_real_data.accel[2];
    imu_feedback.angular_vel.x = bmi088_real_data.gyro[0];
    imu_feedback.angular_vel.y = bmi088_real_data.gyro[1];
    imu_feedback.angular_vel.z = bmi088_real_data.gyro[2];
    ProtocolSend(PACK_TYPE_IMU_REPONSE, (uint8_t *)&imu_feedback, sizeof(struct imu_feedback));
    osDelay(20);
  }
  /* USER CODE END ImuTaskEntry */
}


osThreadDef(ImuTask, ImuTaskEntry, osPriorityAboveNormal, 0, 512);
osThreadDef(ImuFeedbackTask, ImuFeedbackTaskEntry, osPriorityNormal, 0, 128);

void ImuTaskInit(void)
{
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);
  ImuFeedbackTaskHandle = osThreadCreate(osThread(ImuFeedbackTask), NULL);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT1_ACCEL_Pin)
  {
    accel_update_flag |= 1 << IMU_DR_SHFITS;
    accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag)
    {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == INT1_GYRO_Pin)
  {
    gyro_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag)
    {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == DRDY_IST8310_Pin)
  {
    mag_update_flag |= 1 << IMU_DR_SHFITS;

    if (mag_update_flag &= 1 << IMU_DR_SHFITS)
    {
      mag_update_flag &= ~(1 << IMU_DR_SHFITS);
      mag_update_flag |= (1 << IMU_SPI_SHFITS);

      ist8310_read_mag(ist8310_real_data.mag);
    }
  }
  else if (GPIO_Pin == GPIO_PIN_0)
  {
    //wake up the task
    //��������
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(ImuTaskHandle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{

  //���������ǵ�DMA����
  if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
    gyro_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    return;
  }
  //�������ٶȼƵ�DMA����
  if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    accel_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
    return;
  }

  if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
    return;
  }
}

void DMA2_Stream2_IRQHandler(void)
{

  if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
  {
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

    //gyro read over
    //�����Ƕ�ȡ���
    if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
    {
      gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
      gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    }

    //accel read over
    //���ٶȼƶ�ȡ���
    if (accel_update_flag & (1 << IMU_SPI_SHFITS))
    {
      accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }
    //temperature read over
    //�¶ȶ�ȡ���
    if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
    {
      accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }

    imu_cmd_spi_dma();

    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
    }
  }
}

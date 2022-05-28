#include "param.h"
#include "stm32f4xx_hal_flash.h"
#include "string.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "stdint.h"

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

const Param_t DefaultParam = {
  PARAM_HEAD,
  0.033,//wheel_r
  7.6,//motor_reduction_ratio
  PI,//max_w
  4.5,//max_speed
  {0.087, 0.13, 17, 30},//{base_a, base_b, pwm_dead_zone, max_steer_angle}
  {100, 10, 3, 500.0, 500.0},//p i d max_output i_limit
  4096,//tick per lap
  65536,//max ticks
  10,//ctrl period
  10,//feedback period
  10,//pose calc period
  0,//accel z offset
  1,//servo dir 0 or 1
  PARAM_TAIL
};


Param_t param;

void InitParam(void)
{
  int i;
  Param_t *p = (Param_t *)PARAM_SAVE_ADDR;

  if ((p->param_head != PARAM_HEAD) || (p->param_tail != PARAM_TAIL))
  {
    FLASH_EraseInitTypeDef EarseStructure;
    uint32_t SectorError = 0;
    HAL_FLASH_Unlock();
    //HAL_Delay(2000);
    EarseStructure.TypeErase = FLASH_TYPEERASE_SECTORS;
    EarseStructure.Banks = FLASH_BANK_1;
    EarseStructure.Sector = FLASH_SECTOR_11;
    EarseStructure.NbSectors = 1;
    EarseStructure.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&EarseStructure, &SectorError);
    //HAL_Delay(2000);
    for (i = 0; i < sizeof(Param_t); i++)
    {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, PARAM_SAVE_ADDR + i, *(((uint8_t *)&DefaultParam) + i));
    }
    HAL_FLASH_Lock();
  }

  memcpy(&param, (void *)PARAM_SAVE_ADDR, sizeof(Param_t));
}

void SaveParam(void *p)
{
  int i;
  FLASH_EraseInitTypeDef EarseStructure;
  uint32_t SectorError = 0;
  HAL_FLASH_Unlock();
  //HAL_Delay(2000);
  EarseStructure.TypeErase = FLASH_TYPEERASE_SECTORS;
  EarseStructure.Banks = FLASH_BANK_1;
  EarseStructure.Sector = FLASH_SECTOR_11;
  EarseStructure.NbSectors = 1;
  EarseStructure.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
  HAL_FLASHEx_Erase(&EarseStructure, &SectorError);

  for (i = 0; i < sizeof(Param_t); i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, PARAM_SAVE_ADDR + i, *(((uint8_t *)p) + i));
  }
  HAL_FLASH_Lock();
}

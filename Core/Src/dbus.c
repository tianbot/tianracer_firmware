#include "dbus.h"

void RemoteDataProcess(uint8_t *pData, RC_Ctl_t * pCtrlData)
{
  if((pData == NULL) || (pCtrlData == NULL))
  {
    return;
  }
  pCtrlData->rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
  pCtrlData->rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
  pCtrlData->rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] <<10)) & 0x07FF;
  pCtrlData->rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
  pCtrlData->rc.ch4 = ((int16_t)pData[16]) | ((int16_t)pData[17] << 8);
  pCtrlData->rc.s1 = ((pData[5] >> 6) & 0x0003);
  pCtrlData->rc.s2 = ((pData[5] >> 4) & 0x0003);
  pCtrlData->mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
  pCtrlData->mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
  pCtrlData->mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
  pCtrlData->mouse.press_l = pData[12];
  pCtrlData->mouse.press_r = pData[13];
  pCtrlData->key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
}

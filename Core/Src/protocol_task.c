#include "protocol_task.h"
#include "cmsis_os.h"
#include "dbus_task.h"
#include "beep_task.h"
#include "param.h"
#include "usart.h"
#include "dbus.h"
#include "stdlib.h"
#include "string.h"
#include "usbd_cdc_if.h"
osMailQId ProtocolRxMail;
osMailQId ProtocolTxMail;
osMailQDef(ProtocolRxMail, PROTOCOL_MSG_QUENE_SIZE, struct ProtocolMsg);
osMailQDef(ProtocolTxMail, PROTOCOL_MSG_QUENE_SIZE, struct ProtocolMsg);

osThreadId ProtocolSendTaskHandle;
osThreadId ProtocolRecvTaskHandle;

uint8_t ProtocolBuff[PROTOCOL_MSG_LEN];
uint8_t ConnectStatus = DISCONNECTED;
extern USBD_HandleTypeDef hUsbDeviceHS;

static void BeepConnect(void)
{
  Beep(0, 150);
  Beep(1, 150);
  Beep(2, 150);
}

static void BeepDisconnect(void)
{
  Beep(2, 150);
  Beep(1, 150);
  Beep(0, 150);
}

static int ParseLine(char *line, char *argv[])
{
  int nargs = 0;
  while (nargs < 10)
  {
    while ((*line == ' ') || (*line == '\t'))
    {
      ++line;
    }

    if (*line == '\0' || (*line == '\r') || (*line == '\n'))
    {
      *line = '\0';
      argv[nargs] = NULL;
      return nargs;
    }

    argv[nargs++] = line;

    while (*line && (*line != ' ') && (*line != '\t') && (*line != '\n') && (*line != '\r'))
    {
      ++line;
    }

    if ((*line == '\0') || (*line == '\r') || (*line == '\n'))
    {
      *line = '\0';
      argv[nargs] = NULL;
      return nargs;
    }

    *line++ = '\0';
  }

  return nargs;
}

void ProtocolSend(uint16_t pack_type, uint8_t *msg, uint16_t len)
{
  struct ProtocolMsg *p;
  p = osMailAlloc(ProtocolTxMail, osWaitForever);
  int i;
  if (p != NULL)
  {
    struct protocol_pack *pPack = (struct protocol_pack *)(p->Msg);
    pPack->head = PROTOCOL_HEAD;
    pPack->len = len + 2;
    pPack->pack_type = pack_type;
    if (len != 0)
    {
      memcpy(pPack->data, msg, len);
    }
    pPack->data[len] = 0;
    for (i = 0; i < pPack->len; i++)
    {
      pPack->data[len] ^= p->Msg[4 + i];
    }
    p->MsgLen = pPack->len + 4 + 1;
    osMailPut(ProtocolTxMail, p);
  }
}

static void ProtocolProcess(uint8_t *Buf, uint8_t Len)
{
  static int offset;
  static uint8_t local_buf[PROTOCOL_MSG_LEN];
  static struct protocol_pack *p = (struct protocol_pack *)local_buf;
  static char ret_buff[PROTOCOL_MSG_LEN - 12];
  uint8_t bcc = 0;
  uint8_t i;

  uint32_t length;
  uint32_t base;
  DebugCmd_t *pCmd;
  char *argv[CFG_MAXARGS];
  int argc;

  if (offset + Len >= PROTOCOL_MSG_LEN) // ignore msg
  {
    offset = 0;
    return;
  }
  memcpy(local_buf + offset, Buf, Len);
  offset += Len;
  if (offset < 4)
  {
    return;
  }

  if (p->head != PROTOCOL_HEAD) // ignore msg
  {
    offset = 0;
    return;
  }

  if (p->len >= PROTOCOL_MSG_LEN - 5) // ignore msg
  {
    offset = 0;
    return;
  }

  if (offset < p->len + 5)
  {
    return;
  }

  offset = 0;

  for (i = 4; i < p->len + 5; i++) // bcc calc from pack type
  {
    bcc ^= local_buf[i];
  }
  if (bcc == 0)
  {
    if (ConnectStatus == DISCONNECTED)
    {
      ConnectStatus = CONNECTED;
      BeepConnect();
    }
    if ((CtrlFlag != CTRL_TYPE_PC) && (p->pack_type != PACK_TYPE_DEBUG))
    {
      return;
    }
    switch (p->pack_type)
    {
    case PACK_TYPE_CMD_VEL:
      if (sizeof(struct twist) == p->len - 2)
      {
        MotionCtrl_t *pMotionData = osMailAlloc(CtrlMail, osWaitForever);
        if (pMotionData != NULL)
        {
          struct twist *pTwist = (struct twist *)(p->data);
          pMotionData->vx = pTwist->linear.x;
          pMotionData->vy = pTwist->linear.y;
          pMotionData->w = pTwist->angular.z;
          osMailPut(CtrlMail, pMotionData);
        }
      }
      break;

    case PACK_TYPE_ACKMAN_VEL:
      if (sizeof(struct ackermann_cmd) == p->len - 2)
      {
        MotionCtrl_t *pMotionData = osMailAlloc(CtrlMail, osWaitForever);
        if (pMotionData != NULL)
        {
          struct ackermann_cmd *pAckermann = (struct ackermann_cmd *)(p->data);
          pMotionData->vx = pAckermann->speed;
          pMotionData->steering_angle = pAckermann->steering_angle * RADIAN_COEF;
          osMailPut(CtrlMail, pMotionData);
        }
      }
      break;

    case PACK_TYPE_HEART_BEAT:
      // ProtocolSend(PACK_TYPE_HEART_BEAT_RESPONSE, NULL, 0);
      break;

    case PACK_TYPE_DEBUG:
      length = (uint32_t)(&DEBUG_CMD_SECTOR_Limit - &DEBUG_CMD_SECTOR_Base);
      base = (uint32_t)&DEBUG_CMD_SECTOR_Base;
      p->data[p->len - 2] = '\0';
      argc = ParseLine((char *)p->data, argv);
      pCmd = (DebugCmd_t *)base;
      for (i = 0; i < length / sizeof(DebugCmd_t); i++)
      {
        if (strcmp((char *)p->data, pCmd[i].CmdName) == 0)
        {
          ret_buff[0] = '\0';
          pCmd[i].pCmdEntry(argc, argv, ret_buff, sizeof(ret_buff));
          break;
        }
      }
      if (strlen(ret_buff) != 0)
      {
        ProtocolSend(PACK_TYPE_DEBUG_RESPONSE, (uint8_t *)ret_buff, strlen(ret_buff) + 1);
      }
      break;

    default:
      break;
    }
  }
}

static void ProtocolRecvTaskEntry(void const *argument)
{
  osEvent evt;
  struct ProtocolMsg *p;
  osDelay(5000);
  HAL_UART_Receive_DMA(&huart1, ProtocolBuff, PROTOCOL_MSG_LEN);
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* Infinite loop */
  for (;;)
  {
    evt = osMailGet(ProtocolRxMail, PROTOCOL_TIMEOUT);
    if (evt.status == osEventMail)
    {
      p = evt.value.p;
      ProtocolProcess(p->Msg, p->MsgLen);
      osMailFree(ProtocolRxMail, p);
    }
    else if (evt.status == osEventTimeout)
    {
      if (ConnectStatus == CONNECTED)
      {
        ConnectStatus = DISCONNECTED;
        BeepDisconnect();
      }
      // disconnect
      if (CtrlFlag == CTRL_TYPE_PC)
      {

        MotionCtrl_t *pMotionData = osMailAlloc(CtrlMail, osWaitForever);
        if (pMotionData != NULL)
        {
          pMotionData->vx = 0;
          pMotionData->vy = 0;
          pMotionData->w = 0;
          pMotionData->steering_angle = 0;
          osMailPut(CtrlMail, pMotionData);
        }
      }
    }
  }
}

static void ProtocolSendTaskEntry(void const *argument)
{
  struct ProtocolMsg *p;
  osEvent evt;
  /* Infinite loop */
  for (;;)
  {
    evt = osMailGet(ProtocolTxMail, osWaitForever);
    if (evt.status == osEventMail)
    {
      p = evt.value.p;
      if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED)
      {
        CDC_Transmit_HS(p->Msg, p->MsgLen);
        osSignalWait(USB_TX_FINISH, USB_TX_TIMEOUT);
      }
      else
      {
        HAL_UART_Transmit_DMA(&huart1, p->Msg, p->MsgLen);
        osSignalWait(UART1_TX_FINISH, osWaitForever);
      }
      osMailFree(ProtocolTxMail, p);
    }
  }
}
osThreadDef(ProtocolRecvTask, ProtocolRecvTaskEntry, osPriorityRealtime, 0, 512);
osThreadDef(ProtocolSendTask, ProtocolSendTaskEntry, osPriorityAboveNormal, 0, 512);
void ProtocolTaskInit(void)
{
  ProtocolRxMail = osMailCreate(osMailQ(ProtocolRxMail), NULL);
  ProtocolTxMail = osMailCreate(osMailQ(ProtocolTxMail), NULL);

  ProtocolRecvTaskHandle = osThreadCreate(osThread(ProtocolRecvTask), NULL);
  ProtocolSendTaskHandle = osThreadCreate(osThread(ProtocolSendTask), NULL);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    osSignalSet(ProtocolSendTaskHandle, UART1_TX_FINISH);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_AbortReceive(&huart1);
    struct ProtocolMsg *pProtocolMsg = osMailAlloc(ProtocolRxMail, 0);
    if (pProtocolMsg == NULL)
    {
      // error
    }
    else
    {
      pProtocolMsg->MsgLen = PROTOCOL_MSG_LEN - huart1.hdmarx->Instance->NDTR;
      memcpy(pProtocolMsg->Msg, ProtocolBuff, pProtocolMsg->MsgLen);
      osMailPut(ProtocolRxMail, pProtocolMsg);
      HAL_UART_Receive_DMA(&huart1, ProtocolBuff, PROTOCOL_MSG_LEN);
    }
  }
}

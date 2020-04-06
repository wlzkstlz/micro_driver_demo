/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include "main.h"

#define DEBUG_RX_CAN_ID1 0x001
#define DEBUG_RX_CAN_ID2 0x002
#define DEBUG_RX_CAN_ID3 0x003
#define DEBUG_RX_CAN_ID4 0x004

//#define DEBUG_TX_CAN_ID1 0x005

//2个3级深度的FIFO

#define CAN1FIFO CAN_RX_FIFO0
#define CAN2FIFO CAN_RX_FIFO1 //

CAN_TxHeaderTypeDef TxMeg;
CAN_RxHeaderTypeDef RxMeg;

void CAN_User_Init(CAN_HandleTypeDef *phcan) //用户初始化函数
{
  CAN_FilterTypeDef sFilterConfig;
  HAL_StatusTypeDef HAL_Status;

  TxMeg.IDE = CAN_ID_STD; //CAN_ID_EXT;
  TxMeg.RTR = CAN_RTR_DATA;

  sFilterConfig.FilterBank = 0;                     //过滤器0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; //设为列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = DEBUG_RX_CAN_ID1 << 5; //基本ID放入到STID中
  sFilterConfig.FilterIdLow = DEBUG_RX_CAN_ID2 << 5;
  sFilterConfig.FilterMaskIdHigh = DEBUG_RX_CAN_ID3 << 5;
  sFilterConfig.FilterMaskIdLow = DEBUG_RX_CAN_ID4 << 5;
  sFilterConfig.FilterFIFOAssignment = CAN1FIFO; //接收到的报文放入到FIFO0中
  sFilterConfig.FilterActivation = ENABLE; //激活过滤器
  sFilterConfig.SlaveStartFilterBank = 0;

  HAL_Status = HAL_CAN_ConfigFilter(phcan, &sFilterConfig);
  HAL_Status = HAL_CAN_Start(phcan); //开启CAN
  if (HAL_Status != HAL_OK)
  {
    //printf("开启CAN失败\r\n");
  }

  HAL_Status = HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (HAL_Status != HAL_OK)
  {
    //printf("开启挂起中段允许失败\r\n");
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan) //接收回调函数
{

//  uint8_t Data[8];

//  HAL_StatusTypeDef HAL_RetVal;

//  if (phcan == &hcan)
//  {

//    HAL_RetVal = HAL_CAN_GetRxMessage(&hcan, CAN1FIFO, &RxMeg, Data);

//    if (HAL_OK == HAL_RetVal)
//    {
//      //在这里接收数据
//    }
//  }
}

//发送数据函数

uint8_t CANx_SendNormalData(CAN_HandleTypeDef *phcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
  HAL_StatusTypeDef HAL_RetVal;
  uint16_t SendTimes, SendCNT = 0;
  uint8_t FreeTxNum = 0;
  TxMeg.StdId = ID;

  if (!phcan || !pData || !Len)
    return 1;

  SendTimes = Len / 8 + (Len % 8 ? 1 : 0);
  FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(phcan);
  TxMeg.DLC = 8;
  while (SendTimes--)
  {
    if (0 == SendTimes)
    {
      if (Len % 8)
        TxMeg.DLC = Len % 8;
    }
    while (0 == FreeTxNum)
    {
      FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(phcan);
    }
    HAL_Delay(1); //没有延时很有可能会发送失败
		
		uint32_t can_tx_mailbox;
    HAL_RetVal = HAL_CAN_AddTxMessage(phcan, &TxMeg, pData + SendCNT, &can_tx_mailbox);//(uint32_t *)CAN_TX_MAILBOX0);
    if (HAL_RetVal != HAL_OK)
    {
      return 2;
    }
    SendCNT += 8;
  }
  return 0;
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspInit 0 */

    /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN CAN1_MspInit 1 */

    /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

//#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

uint8_t u2_rxBuff[ U2_RXFRAMELEN ];
DynamicBuffer u2_txBuff;


/* USER CODE END 0 */

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* USART2_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  // RX

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, LL_USART_DMA_GetRegAddr(USART2));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)u2_rxBuff);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, U2_RXFRAMELEN);

  LL_DMA_DisableIT_HT(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);


  // TX

  DynamicBuffer_Init( &u2_txBuff, U2_TXFRAMELEN );

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, LL_USART_DMA_GetRegAddr(USART2));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)DynamicBuffer_GetBuffAddr(&u2_txBuff));
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, DynamicBuffer_GetBuffSize(&u2_txBuff));

  LL_DMA_DisableIT_HT(DMA1, LL_DMA_STREAM_6);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);



  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 57600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  //LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */


  LL_USART_EnableDMAReq_RX(USART2);
  LL_USART_EnableIT_IDLE(USART2);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);

  LL_USART_EnableDMAReq_TX(USART2);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);

  LL_USART_Enable(USART2);




  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 1 */

void UART_TransmitDMA( USART_TypeDef *USARTx, DynamicBuffer *dbuff )
{

    if ( (DynamicBuffer_GetBuffPos(dbuff) > 0) && (DynamicBuffer_GetTxSize(dbuff) == 0) ) {

        size_t txsize = DynamicBuffer_GetBuffPos(dbuff);

        if ( txsize > 10 ) txsize = 10;

        DynamicBuffer_SetTxSize( dbuff, txsize );

        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, DynamicBuffer_GetTxSize(dbuff));
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)DynamicBuffer_GetBuffAddr(dbuff));

        // Clear all flags
        LL_DMA_ClearFlag_TC6(DMA1);
        LL_DMA_ClearFlag_HT6(DMA1);
        LL_DMA_ClearFlag_DME6(DMA1);
        LL_DMA_ClearFlag_FE6(DMA1);
        LL_DMA_ClearFlag_TE6(DMA1);

        // Start transfer
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);

    }


}











/* USER CODE END 1 */

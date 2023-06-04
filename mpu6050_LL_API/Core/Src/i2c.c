/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

uint8_t i2cCEV = 0;

uint8_t i2cDevAddr=0x00;
uint8_t i2cMemAddr=0x00;

uint8_t i2cRxBufferIndex=0;
uint8_t *i2cRxBufferPtr;
uint8_t i2cRxBufferLen;


/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* I2C1 DMA Init */

  /* I2C1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_6, LL_I2C_DMA_GetRegAddr(I2C1));

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */



  // Any of these Buffer events will generate interrupt.
  // ! ONLY if event interrupts are enabled too !
  // 
  // Receive buffer not empty (RXNE)
  // Transmit buffer empty (TXE)
  LL_I2C_EnableIT_BUF(I2C1);


  // Any of these events will generate interrupt :
  // Start Bit (SB)
  // Address sent, Address matched (ADDR)
  // 10-bit header sent (ADD10)
  // Stop detection  (STOPF)
  // Byte transfer finished (BTF)
  LL_I2C_EnableIT_EVT(I2C1);

  // Enable RXNE interrupt
  //
  //LL_I2C_EnableIT_RX(I2C1);

  // Enable TXE interrupt
  //LL_I2C_EnableIT_TX(I2C1);


  // Bus Error detection (BERR)
  // Arbitration Loss (ARLO)
  // Acknowledge Failure(AF)
  // Overrun/Underrun (OVR)
  // SMBus Timeout detection (TIMEOUT)
  // SMBus PEC error detection (PECERR)
  // SMBus Alert pin event detection (SMBALERT)
  //LL_I2C_EnableIT_ERR(I2C1);

  //LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

  LL_I2C_Enable(I2C1);


  /* USER CODE END I2C1_Init 2 */

}

/* USER CODE BEGIN 1 */

void I2C_PrepMemDMA( uint32_t bf )
{

  //LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)bf);


  //LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_6,
  //		  sizeof(bf) / sizeof((bf)[0])  );
            

  //LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_6 );


}


void I2C_SendRequest( const uint8_t devAddr, const uint8_t memAddr, uint8_t* rxBuffPtr, uint8_t rxBuffLen  )
{


	i2cDevAddr = devAddr;
	i2cMemAddr = memAddr;

	i2cRxBufferPtr = rxBuffPtr;
	i2cRxBufferLen = rxBuffLen;

	// Disable POS
	LL_I2C_DisableBitPOS(I2C1);

	// Enable acknowledge
	//
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

	// Generate start condition
	//
	LL_I2C_GenerateStartCondition(I2C1);


	// Set i2c Custom Event Counter
	//
	i2cCEV = 2;
	__NOP();

}

const uint8_t I2C_ReadByte()
{

}
























void I2C_ErrataWorkaround()
{


	  // Errata ES096 - Rev15, 2.8.7 Workaround

	  LL_I2C_Disable(I2C1);

	  //PB6   ------> I2C1_SCL
	  //PB7   ------> I2C1_SDA

	  uint32_t rd1=0;

	  // Configure to Output, Open-Drain and Set LOW
	  //
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
	  rd1 = LL_GPIO_GetPinPull(GPIOB, LL_GPIO_PIN_6);


	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);
	  rd1 = LL_GPIO_GetPinPull(GPIOB, LL_GPIO_PIN_7);

	  // Set HIGH
	  //
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
	  rd1 = LL_GPIO_GetPinPull(GPIOB, LL_GPIO_PIN_6);

	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
	  rd1 = LL_GPIO_GetPinPull(GPIOB, LL_GPIO_PIN_7);


	  // Configure pins normally
	  //
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);

	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);

	  // Set and clear SWRST

	  LL_I2C_EnableReset(I2C1);
	  LL_I2C_DisableReset(I2C1);


	  LL_I2C_Enable(I2C1);


	  // End Workaround

}


/* USER CODE END 1 */

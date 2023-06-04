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
uint8_t i2cRxBufferLen=0;

uint8_t *i2cTxBytePtr;

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

	// Set i2c Custom Event Counter
	//
	i2cCEV = 2;

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

}





void I2C_WriteRequest( const uint8_t devAddr, const uint8_t memAddr, const uint8_t payload  )
{

	// Set i2c Custom Event Counter
	//
	i2cCEV = 12;

	i2cTxBytePtr = &payload;

	i2cDevAddr = devAddr;
	i2cMemAddr = memAddr;

	// Disable POS
	LL_I2C_DisableBitPOS(I2C1);

	// Generate start condition
	//
	LL_I2C_GenerateStartCondition(I2C1);

}



void I2C_IRQHandlerRead()
{
	//
	//	i2cCEV = 0: Read Memory request inactive
	//  i2cCEV = 1: Read Memory request active, but currently I2C is BUSY, waiting to clear
	//  i2cCEV = 2: ACK set and START CONDITION Generated
	//  i2cCEV = 3: Device Address + Write BIT transmitted
	//  i2cCEV = 4: ADDR Flag is received and cleared after CEV=3
	//  i2cCEV = 5: Memory Address transmitted
	//  i2cCEV = 6: New start condition generated
	//  i2cCEV = 7: Device Address + Read BIT transmitted
	//  i2cCEV = 8: ACK, ADDR flags cleared after CEV=7, and STOP CONDITION Generated
	//
	//  After i2cCEV=8, i2c registers are read as long as RXNE is set
	// 	then i2cCEV is set to 0, to finish and disable Read Memory Request
	//
	uint8_t dum;

	if ( LL_I2C_IsActiveFlag_SB(I2C1) ) {

		switch (i2cCEV) {
		case 2:
			LL_I2C_TransmitData8( I2C1, I2C_7BIT_ADD_WRITE(i2cDevAddr) );
			i2cCEV = 3;
			break;

		case 6:
			LL_I2C_TransmitData8(I2C1, I2C_7BIT_ADD_READ(i2cDevAddr));
			i2cCEV=7;
			break;

		default:
			break;
		}


	} else if ( LL_I2C_IsActiveFlag_ADDR(I2C1) ) {

		switch (i2cCEV) {
		case 3:
			LL_I2C_ClearFlag_ADDR(I2C1);
			i2cCEV = 4;
			break;

		case 7:

			if ( i2cRxBufferLen == 0) {

				LL_I2C_ClearFlag_ADDR(I2C1);
				LL_I2C_GenerateStopCondition(I2C1);

			} else if ( i2cRxBufferLen == 1 ) {

				// Disable ack
				LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

				__disable_irq();

				LL_I2C_ClearFlag_ADDR(I2C1);

				LL_I2C_GenerateStopCondition(I2C1);

				__enable_irq();

			} else if ( i2cRxBufferLen == 2 ) {

				LL_I2C_EnableBitPOS(I2C1);

				__disable_irq();

				LL_I2C_ClearFlag_ADDR(I2C1);

				// Disable ack
				LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

				__enable_irq();

			} else {

				//Enable ack
				LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

				LL_I2C_ClearFlag_ADDR(I2C1);

			}


			i2cCEV = 8;
			break;

		default:
			LL_I2C_ClearFlag_ADDR(I2C1);
			break;
		}


	} else if ( LL_I2C_IsActiveFlag_TXE(I2C1) ) {

		switch (i2cCEV) {
		case 4:
			LL_I2C_TransmitData8( I2C1, I2C_MEM_ADD_LSB(i2cMemAddr) );
			i2cCEV = 5;

			// Break, because TXE flag must be set again, after MemAddr transmit
			break;

		case 5:
			LL_I2C_GenerateStartCondition(I2C1);
			i2cCEV = 6;
			break;

		default:
			break;
		}


	} else if ( LL_I2C_IsActiveFlag_RXNE(I2C1) ) {

		switch (i2cCEV) {

		case 8:

			if ( LL_I2C_IsActiveFlag_RXNE(I2C1) )
			{

				if ( i2cRxBufferLen > 3  ) {

					*i2cRxBufferPtr = LL_I2C_ReceiveData8(I2C1);
					i2cRxBufferPtr++;

					i2cRxBufferLen--;

				} else if ( i2cRxBufferLen == 3 ) {

					// Disable ack
					LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

					//__disable_irq();

					*i2cRxBufferPtr = LL_I2C_ReceiveData8(I2C1);
					i2cRxBufferPtr++;

					i2cRxBufferLen--;

					//__enable_irq();
					//LL_I2C_GenerateStopCondition(I2C1);


				} else if ( i2cRxBufferLen == 2 ) {

					__disable_irq();

					LL_I2C_GenerateStopCondition(I2C1);

					*i2cRxBufferPtr = LL_I2C_ReceiveData8(I2C1);
					i2cRxBufferPtr++;

					i2cRxBufferLen--;

					__enable_irq();

				} else if ( i2cRxBufferLen == 1 ) {

					// Transmission complete
					*i2cRxBufferPtr = LL_I2C_ReceiveData8(I2C1);

					i2cRxBufferLen--;
					i2cCEV = 0;

				}  else {
					dum = LL_I2C_ReceiveData8(I2C1);
					__NOP();
				}

			} else {
				i2cCEV = 0;
				//i2cRxBufferIndex=0;
			}

			break;

		default:
			dum = LL_I2C_ReceiveData8(I2C1);
			break;
		}

	}

}




void I2C_IRQHandlerWrite()
{

	//
	//	i2cCEV < 11: Write Memory request inactive
	//  i2cCEV = 11: Write Memory request active, but currently I2C is BUSY, waiting to clear
	//  i2cCEV = 12: START CONDITION Generated
	//  i2cCEV = 13: Device Address + Write BIT transmitted
	//  i2cCEV = 14: ADDR Flag is received and cleared after CEV=13
	//  i2cCEV = 15: STOP CONDITION generated & Memory Address transmitted
	//  i2cCEV = 16: Another STOP CONDITION generated & payload transmitted
	//
	//  After i2cCEV=16,i2cCEV is set to 0, to finish and disable Write Memory Request
	//

	if ( LL_I2C_IsActiveFlag_SB(I2C1) ) {

		switch (i2cCEV) {
		case 12:
			LL_I2C_TransmitData8( I2C1, I2C_7BIT_ADD_WRITE(i2cDevAddr) );
			i2cCEV = 13;
			break;

		default:
			break;
		}


	} else if ( LL_I2C_IsActiveFlag_ADDR(I2C1) ) {

		switch (i2cCEV) {
		case 13:
			LL_I2C_ClearFlag_ADDR(I2C1);
			i2cCEV = 14;

			break;


		default:
			LL_I2C_ClearFlag_ADDR(I2C1);

			break;
		}


	} else if ( LL_I2C_IsActiveFlag_TXE(I2C1) ) {

		switch (i2cCEV) {
		case 14:

			LL_I2C_GenerateStopCondition(I2C1);
			LL_I2C_TransmitData8( I2C1, I2C_MEM_ADD_LSB(i2cMemAddr) );
			i2cCEV = 15;

			// Break, because TXE flag must be set again, after MemAddr transmit
			break;

		case 15:

			LL_I2C_GenerateStopCondition(I2C1);
			uint8_t test = *i2cTxBytePtr;
			__NOP();
			LL_I2C_TransmitData8( I2C1, *i2cTxBytePtr );
			i2cCEV = 16;

			break;

		default:
			break;
		}


	} else if ( LL_I2C_IsActiveFlag_BTF(I2C1) ) {

		switch (i2cCEV) {

		case 16:
			LL_I2C_GenerateStopCondition(I2C1);
			i2cCEV = 0;
			break;

		default:
			break;
		}

	}


}




void I2C_IRQHandlerDefault()
{

	uint8_t dum;

	if ( LL_I2C_IsActiveFlag_SB(I2C1) ) {

		__NOP();

	} else if ( LL_I2C_IsActiveFlag_ADDR(I2C1) ) {

		LL_I2C_ClearFlag_ADDR(I2C1);

	} else if ( LL_I2C_IsActiveFlag_TXE(I2C1) ) {

		__NOP();

	} else if ( LL_I2C_IsActiveFlag_RXNE(I2C1) ) {

		dum = LL_I2C_ReceiveData8(I2C1);

	}

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

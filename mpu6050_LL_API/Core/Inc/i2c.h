/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define I2C_7BIT_ADD_WRITE(__ADDRESS__)                    ((uint8_t)((__ADDRESS__) & (uint8_t)(~I2C_OAR1_ADD0)))
#define I2C_7BIT_ADD_READ(__ADDRESS__)                     ((uint8_t)((__ADDRESS__) | I2C_OAR1_ADD0))
#define I2C_MEM_ADD_LSB(__ADDRESS__)                       ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)0x00FF)))



extern uint8_t i2cCEV;

extern uint8_t i2cDevAddr;
extern uint8_t i2cMemAddr;

extern uint8_t i2cRxBufferIndex;
extern uint8_t *i2cRxBufferPtr;
extern uint8_t i2cRxBufferLen;

extern uint8_t *i2cTxBytePtr;


/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

void I2C_ErrataWorkaround();

void I2C_PrepMemDMA( uint32_t bf );

void I2C_SendRequest( const uint8_t devAddr, const uint8_t memAddr, uint8_t* rxBuffPtr, uint8_t rxBuffLen );

void I2C_WriteRequest( const uint8_t devAddr, const uint8_t memAddr, const uint8_t payload  );


void I2C_IRQHandlerRead();

void I2C_IRQHandlerWrite();

void I2C_IRQHandlerDefault();



const uint8_t I2C_ReadByte();



/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */


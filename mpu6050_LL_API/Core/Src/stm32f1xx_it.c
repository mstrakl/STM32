/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

	uint8_t dum;
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

	__NOP();

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

			// Disable ack
			LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

			__disable_irq();

			LL_I2C_ClearFlag_ADDR(I2C1);

			LL_I2C_GenerateStopCondition(I2C1);

			__enable_irq();


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

				if ( i2cRxBufferIndex < i2cRxBufferLen  ) {

					*i2cRxBufferPtr = LL_I2C_ReceiveData8(I2C1);
					i2cRxBufferPtr++;

					i2cRxBufferIndex++;

				} else {
					dum = LL_I2C_ReceiveData8(I2C1);
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





  /* USER CODE END I2C1_EV_IRQn 0 */

  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */

  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

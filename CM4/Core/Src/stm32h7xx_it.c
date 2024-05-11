/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ERR_Port LED2_GPIO_Port
#define ERR_Pin  LED2_Pin

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
extern MDMA_HandleTypeDef hmdma_mdma_channel0_sw_0;
extern DMA_HandleTypeDef hdma_dcmi;
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern HCD_HandleTypeDef hhcd_USB_OTG_HS;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  __disable_irq();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __disable_irq();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  __disable_irq();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  __disable_irq();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  __disable_irq();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_SET);
    my_Delay(250);
    HAL_GPIO_WritePin(ERR_Port, ERR_Pin, GPIO_PIN_RESET);
    my_Delay(500);
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
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

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE END DMA1_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  /* USER CODE BEGIN TIM7_IRQn 1 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dcmi);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS End Point 1 Out global interrupt.
  */
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_EP1_OUT_IRQn 0 */

  /* USER CODE END OTG_HS_EP1_OUT_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_EP1_OUT_IRQn 1 */

  /* USER CODE END OTG_HS_EP1_OUT_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS End Point 1 In global interrupt.
  */
void OTG_HS_EP1_IN_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_EP1_IN_IRQn 0 */

  /* USER CODE END OTG_HS_EP1_IN_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_EP1_IN_IRQn 1 */

  /* USER CODE END OTG_HS_EP1_IN_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/**
  * @brief This function handles DCMI global interrupt.
  */
void DCMI_IRQHandler(void)
{
  /* USER CODE BEGIN DCMI_IRQn 0 */

  /* USER CODE END DCMI_IRQn 0 */
  HAL_DCMI_IRQHandler(&hdcmi);
  /* USER CODE BEGIN DCMI_IRQn 1 */

  /* USER CODE END DCMI_IRQn 1 */
}

/**
  * @brief This function handles HSEM2 global interrupt.
  */
void HSEM2_IRQHandler(void)
{
  /* USER CODE BEGIN HSEM2_IRQn 0 */

  /* USER CODE END HSEM2_IRQn 0 */
  HAL_HSEM_IRQHandler();
  /* USER CODE BEGIN HSEM2_IRQn 1 */

  /* USER CODE END HSEM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void my_Delay(uint32_t Delay)
{
  uint32_t count = Delay * 50000;
  while (count > 0)
  {
    asm("nop");
    count -= 1;
  }
}

/* USER CODE END 1 */

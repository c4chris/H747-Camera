/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_azure_rtos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#define HSEM_ID_1 (1U) /* HW semaphore 1*/
#define HSEM_ID_2 (2U) /* HW semaphore 2*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

volatile uint32_t Notified = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static int32_t ADV7533_Configure(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

  /* we want to hear from CM7 about when to initialize HDMI */
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C4_Init();
  MX_USART1_UART_Init();
  MX_AZURE_RTOS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

  MX_AZURE_RTOS_Process();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x009034B6;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x009034B6;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* wait for synchro from CM7 to send some HDMI commands on I2C4 */
  if (HAL_HSEM_FastTake(HSEM_ID_2) != HAL_OK)
  {
  	Error_Handler();
  }
  int32_t timeout = 0xFFF;
  while (Notified != __HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1) && (timeout-- > 0))
  {
  	HAL_Delay(1);
  }
  if ( timeout < 0 )
  {
  	Error_Handler();
  }
  /* do init */
  if (ADV7533_Configure() != 0)
  {
  	Error_Handler();
  }
  /* Release HSEM in order to notify the CPU1 (CM7) */
  HAL_HSEM_Release(HSEM_ID_2, 0);

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NE4_A_GPIO_Port, NE4_A_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NE4_B_GPIO_Port, NE4_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NE1_A_GPIO_Port, NE1_A_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, LED1_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NE1_B_GPIO_Port, NE1_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : NE4_A_Pin */
  GPIO_InitStruct.Pin = NE4_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NE4_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NE4_B_Pin */
  GPIO_InitStruct.Pin = NE4_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NE4_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NE1_A_Pin */
  GPIO_InitStruct.Pin = NE1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NE1_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : NE1_B_Pin */
  GPIO_InitStruct.Pin = NE1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NE1_B_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

/**
  * @brief  Configure the DSI-HDMI ADV7533 bridge for video.
  * @param  pObj pointer to component object
  * @param  LaneNumber Number of lanes to be configured
  * @retval Component status
  */
#define LaneNumber 2
static int32_t ADV7533_Configure(void)
{
  int32_t ret;  
  uint8_t tmp, val;

  /* Configure the IC2 address for CEC_DSI interface */
  tmp = ADV7533_CEC_DSI_I2C_ADDR;
  ret = HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xE1, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  
  /* ADV7533 Power Settings */
  /* Power down */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x41, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x40U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x41, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  /* HPD Override */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xD6, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp |= 0x40U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xD6, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  
  /* Gate DSI LP Oscillator and DSI Bias Clock Powerdown */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x02U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);

  /* Fixed registers that must be set on power-up */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x16, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x3EU;
  tmp |= 0x20U; 
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x16, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  val = 0xE0;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x9A, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xBA, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0xF8U;
  tmp |= 0x70U; 
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xBA, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  val = 0x82;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xDE, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
  
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xE4, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000); 
  tmp |= 0x40U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xE4, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  val = 0x80;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xE5, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
  
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x15, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x30U;
  tmp |= 0x10U; 
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x17, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0xF0U;
  tmp |= 0xD0U; 
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x17, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x24, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x10U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x24, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x57, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp |= 0x01U;
  tmp |= 0x10U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x57, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  
  /* Configure the number of DSI lanes */
  val = (LaneNumber << 4);
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
  
  /* Setup video output mode */
  /* Select HDMI mode */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xAF, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp |= 0x02U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0xAF, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);

  /* HDMI Output Enable */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp |= 0x80U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);

  /* GC packet enable */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp |= 0x80U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  /* Input color depth 24-bit per pixel */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x4C, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x0FU;
  tmp |= 0x03U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x4C, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  /* Down dither output color depth */
  val = 0xFC;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, 0x49, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);

  /* Internal timing disabled */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x27, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x80U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_CEC_DSI_I2C_ADDR, 0x27, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  
  /* Power on */
  ret += HAL_I2C_Mem_Read(&hi2c4, ADV7533_MAIN_I2C_ADDR, ADV7533_MAIN_POWER_DOWN_REG, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  tmp &= ~0x40U;
  ret += HAL_I2C_Mem_Write(&hi2c4, ADV7533_MAIN_I2C_ADDR, ADV7533_MAIN_POWER_DOWN_REG, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1000);
  
  return ret;  
}

/**
  * @brief Semaphore Released Callback.
  * @param SemMask: Mask of Released semaphores
  * @retval None
  */
void HAL_HSEM_FreeCallback(uint32_t SemMask)
{
	Notified |= SemMask;
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

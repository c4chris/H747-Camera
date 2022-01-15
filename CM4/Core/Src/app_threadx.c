/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_STACK_SIZE               (1 * 1024)
/* fx_sd_thread priority */
#define DEFAULT_THREAD_PRIO              10

/* fx_sd_thread preemption priority */
#define DEFAULT_PREEMPTION_THRESHOLD      DEFAULT_THREAD_PRIO

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Define ThreadX global data structures.  */
TX_THREAD            cm4_main_thread;
TX_THREAD            cm4_i2c4_thread;
TX_THREAD            cm4_uart_thread;
TX_EVENT_FLAGS_GROUP cm4_event_group;

/* ...  */
volatile unsigned int u2rc;
volatile unsigned int u2hrc;
volatile unsigned int u2tc;
volatile unsigned int u2htc;
volatile unsigned int u2ec;
volatile unsigned int u2ic;
__attribute__((section(".sram3.touchData"))) volatile uint16_t touchData[4], touchData2[4];
unsigned char dbgBuf[256];
unsigned char input[64];
unsigned char u2tx[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm4_main_thread_entry(ULONG thread_input);
void tx_cm4_i2c4_thread_entry(ULONG thread_input);
void tx_cm4_uart_thread_entry(ULONG thread_input);
void Error_Handler(void);

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  CHAR *pointer;

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

  /*Allocate memory for main_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&cm4_main_thread, "tx_cm4_main_thread", tx_cm4_main_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
		  DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }

  /* Create an event flags group. */
  ret = tx_event_flags_create(&cm4_event_group, "cm4_event_group_name");

  /* If status equals TX_SUCCESS, my_event_group is ready for get and set services. */
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }

  /*Allocate memory for i2c4_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }

  /* Create the i2c4 thread.  */
  ret = tx_thread_create(&cm4_i2c4_thread, "tx_cm4_i2c4_thread", tx_cm4_i2c4_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
		  DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check i2c4 thread creation */
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }

  /*Allocate memory for uart_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }

  /* Create the uart thread.  */
  ret = tx_thread_create(&cm4_uart_thread, "tx_cm4_uart_thread", tx_cm4_uart_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
		  DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check uart thread creation */
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

int UART_Receive(unsigned char *dest, const unsigned char *rx, UART_HandleTypeDef *huart, unsigned int *uxcc, const unsigned int max)
{
	unsigned int cc = __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (*uxcc != cc)
	{
		HAL_UART_DMAPause(huart);
  	int len = 0;
		if (cc > *uxcc)
		{
			for (unsigned int i = max - *uxcc; i < max; i++)
				dest[len++] = rx[i];
			for (unsigned int i = 0; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		else
		{
			for (unsigned int i = max - *uxcc; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		HAL_UART_DMAResume(huart);
  	*uxcc = cc;
  	return len;
	}
	return 0;
}

void tx_cm4_main_thread_entry(ULONG thread_input)
{
	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
	ULONG actual_events;
	ULONG prev_ticks = tx_time_get();
	UINT frame_cnt = 0;
	/* Infinite loop */
	for(;;)
	{
		/* Request that event flags 0 is set. If it is set it should be cleared. If the event
		flags are not set, this service suspends for a maximum of 200 timer-ticks. */
		UINT status = tx_event_flags_get(&cm4_event_group, 0x1, TX_AND_CLEAR, &actual_events, TX_TIMER_TICKS_PER_SECOND);
		ULONG ticks = tx_time_get();

		/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
		if (status == TX_SUCCESS)
		{
			frame_cnt += 1;
			if (ticks - prev_ticks >= TX_TIMER_TICKS_PER_SECOND)
			{
				printf("WS %5lu %u %u",ticks / TX_TIMER_TICKS_PER_SECOND,hdcmi.State,frame_cnt);
				for (unsigned int i = 0; i < 24; i++)
				{
					printf(" %04x",cameraBuffer[i]);
				}
				printf("\r\n");
				prev_ticks = ticks;
				frame_cnt = 0;
			}
			if (hdcmi.State == HAL_DCMI_STATE_SUSPENDED)
			{
				tx_thread_sleep(2);
				HAL_DCMI_Resume(&hdcmi);
			}
		}
		else
		{
			printf("WS %5lu %u",ticks / TX_TIMER_TICKS_PER_SECOND,hdcmi.State);
			printf("\r\n");
		}
	}
}

void tx_cm4_i2c4_thread_entry(ULONG thread_input)
{
	memset((void *)touchData, 0, sizeof(touchData));
	/* Infinite loop */
	for(;;)
	{
		ULONG ticks_target = tx_time_get() + (TX_TIMER_TICKS_PER_SECOND / 20);
		/* this is active low */
		GPIO_PinState touch = HAL_GPIO_ReadPin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin);
		if (!touch || touchData[0] != 0)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			// read status register
			uint8_t nb_touch;
			uint8_t data[4];
			uint32_t touchX, touchY;
		  if (HAL_I2C_Mem_Read(&hi2c4, TS_I2C_ADDRESS, FT6X06_TD_STAT_REG, I2C_MEMADD_SIZE_8BIT, &nb_touch, 1, 1000) == HAL_OK)
		  {
		  	nb_touch &= FT6X06_TD_STATUS_BIT_MASK;
		    if(HAL_I2C_Mem_Read(&hi2c4, TS_I2C_ADDRESS, FT6X06_P1_XH_REG, I2C_MEMADD_SIZE_8BIT, data, 4, 1000) == HAL_OK)
		    {
		      touchX = (((uint32_t)data[0] & FT6X06_P1_XH_TP_BIT_MASK) << 8) | ((uint32_t)data[1] & FT6X06_P1_XL_TP_BIT_MASK);
		      touchY = (((uint32_t)data[2] & FT6X06_P1_YH_TP_BIT_MASK) << 8) | ((uint32_t)data[3] & FT6X06_P1_YL_TP_BIT_MASK);
		      touchData[0] = nb_touch;
		      touchData[1] = touchX;
		      touchData[2] = touchY;
		      touchData[3] += 1;
		    }
		  }
		}
		else
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		ULONG ticks = tx_time_get();
		if (ticks < ticks_target)
			tx_thread_sleep(ticks_target - ticks);
	}
}

void tx_cm4_uart_thread_entry(ULONG thread_input)
{
	//UINT status;
	UCHAR read_buffer[32];
	CHAR data[] = "This is ThreadX working on STM32 CM4";

	printf("\r\n%s\r\nStarting Run on %s\r\n", data, _tx_version_id);
	/* Infinite Loop */
	for( ;; )
	{

		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);

	}
	HAL_UART_Receive_DMA(&huart1, read_buffer, 32);
	unsigned int u2cc = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	//HAL_StatusTypeDef res;
	int inLen = 0;
	/* Infinite loop */
	for(;;)
	{
		int len = UART_Receive(input + inLen, read_buffer, &huart1, &u2cc, 32);
		if (len > 0)
		{
			printf("%.*s", len, input + inLen);
			inLen += len;
			if (input[inLen - 1] == '\r')
			{
				int cmdLen = inLen - 1;
				printf("\nReceived command '%.*s'\r\n# ", cmdLen, input);
				inLen = 0;
				if (strncmp((char *) input, "UART", cmdLen) == 0)
					printf("u2rc = %u u2hrc = %u u2tc = %u u2htc = %u u2ec = %u u2ic = %u u2cc = %u\r\n# ", u2rc, u2hrc, u2tc, u2htc, u2ec, u2ic, u2cc);
			}
		}
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 20);
	}
}

/* USER CODE END 1 */

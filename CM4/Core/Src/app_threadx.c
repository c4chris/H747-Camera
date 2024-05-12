/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "app_filex.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_STACK_SIZE               (2 * 1024)
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
TX_THREAD cm4_main_thread;
TX_THREAD cm4_i2c4_thread;
TX_THREAD cm4_uart_thread;
TX_THREAD cm4_cm7_printf_thread;
/* 
 * event flag 0 (1) is from DCMI transfer done
 * event flag 1 (2) is from TOUCH_INT
 * event flag 2 (4) is for UART transmit done
 * event flag 3 (8) is for new printf data from CM7
 */
TX_EVENT_FLAGS_GROUP cm4_event_group;
/* mutex for printf */
TX_MUTEX mutex_0;

extern FX_MEDIA *media;
extern FX_FILE  *file;

/* ...  */
volatile unsigned int u2rc;
volatile unsigned int u2hrc;
volatile unsigned int u2tc;
volatile unsigned int u2htc;
volatile unsigned int u2ec;
volatile unsigned int u2ic;
__attribute__((section(".sram4.sharedData"))) volatile CM4_CM7_SharedDataTypeDef sharedData;
unsigned char dbgBuf[256];
volatile unsigned int dbgBufCnt;
unsigned char input[64];
ULONG gCounter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm4_main_thread_entry(ULONG thread_input);
void tx_cm4_i2c4_thread_entry(ULONG thread_input);
void tx_cm4_uart_thread_entry(ULONG thread_input);
void tx_cm4_cm7_printf_thread_entry(ULONG thread_input);
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
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR *pointer;

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

  /*Allocate memory for main_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, 2 * DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
	  Error_Handler();
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&cm4_main_thread, "tx_cm4_main_thread", tx_cm4_main_thread_entry, 0, pointer, 2 * DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
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

  /* Create the mutex used by printf without priority inheritance. */
  tx_mutex_create(&mutex_0, "mutex 0", TX_NO_INHERIT);

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

  /*Allocate memory for cm7_printf_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
  	Error_Handler();
  }

  /* Create the cm7_printf thread.  */
  ret = tx_thread_create(&cm4_cm7_printf_thread, "tx_cm4_cm7_printf_thread", tx_cm4_cm7_printf_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
												 DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check uart thread creation */
  if (ret != TX_SUCCESS)
  {
  	Error_Handler();
  }
  /* signal the data structures are ready */
  threadInitDone = 1;
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
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

/**
  * @brief  Tx Transfer completed callback.
  * @param  huart UART handle.
  * @retval none
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Set tx completed flag */
  if (tx_event_flags_set(&cm4_event_group, 0x4, TX_OR) != TX_SUCCESS)
  {
    Error_Handler();
  }
}

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

int _write(int file, char *ptr, int len)
{
	if (len <= 0)
		return len;
	/* Get the mutex with suspension. */
	UINT status = tx_mutex_get(&mutex_0, TX_WAIT_FOREVER);
	/* Check status. */
	if (status != TX_SUCCESS)
	{
		Error_Handler();
	}
	/* Copy the data to output. */
	int max = 256 - dbgBufCnt;
	if (len < max) max = len;
	/* copy th eend of the message in case it is too long */
	memcpy(dbgBuf + dbgBufCnt, ptr + len - max, max);
	dbgBufCnt += max;
	/* Release the mutex. */
	status = tx_mutex_put(&mutex_0);
	/* Check status. */
	if (status != TX_SUCCESS)
	{
		Error_Handler();
	}
	return len;
}

void tx_cm4_main_thread_entry(ULONG thread_input)
{
	printf("Main thread going to sleep\r\n");
	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
	ULONG actual_events;
	ULONG prev_ticks = tx_time_get();
	UINT frame_cnt = 0;
	// We are ready to receive events
	if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) cameraBuffer, (800 * 96 * 2 / 4)) != HAL_OK)
	//if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) cameraBuffer, 192000UL) != HAL_OK)
	//if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) cameraBuffer, 153600UL) != HAL_OK)
	//if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) cameraBuffer, 38400UL) != HAL_OK)
	{
		printf("Error starting DCMI DMA receive\r\n");
	  Error_Handler();
	}
	printf("Started DCMI DMA receive\r\n");
	/* Infinite loop */
	for(;;)
	{
		/* Request that event flags 0 is set. If it is set it should be cleared. If the event
		flags are not set, this service suspends for a maximum of TX_TIMER_TICKS_PER_SECOND timer-ticks. */
		UINT status = tx_event_flags_get(&cm4_event_group, 0x1, TX_AND_CLEAR, &actual_events, TX_TIMER_TICKS_PER_SECOND);
		ULONG ticks = tx_time_get();

		/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
		if (status == TX_SUCCESS)
		{
		  /* Signal CM7 that we have new camera data */
		  HAL_HSEM_FastTake(HSEM_ID_2);
		  HAL_HSEM_Release(HSEM_ID_2,0);
			frame_cnt += 1;
			if ((media != NULL) && (sharedData.CM4_to_CM7_USB_info & USB_INFO_RECORDING))
			{
				UINT res;
				if (sharedData.CM4_to_CM7_USB_stored_count == 0)
				{
					// TODO - we seem to deal with 16 sectors of 512 bytes per cluster, so let's try to write 1 cluster at a time
					printf("Cluster size %u sectors of %u bytes per cluster\r\n", media->fx_media_sectors_per_cluster, media->fx_media_bytes_per_sector);
					printf("Starting to write on USB stick\r\n");
				}
				sharedData.CM4_USB_writing = 1;
				res = fx_file_write(file, (void *) cameraBuffer, 800 * 96 * 2);
				if (res != FX_SUCCESS)
					printf("Failed to write on USB stick : %u\r\n", res);
				sharedData.CM4_USB_writing = 0;
				sharedData.CM4_to_CM7_USB_stored_count += 1;
			}
			if (ticks - prev_ticks >= 60 * TX_TIMER_TICKS_PER_SECOND)
			{
				printf("WS %5lu %u %u",ticks / TX_TIMER_TICKS_PER_SECOND,hdcmi.State,frame_cnt);
				for (unsigned int i = 0; i < 16; i++)
				{
					uint16_t v = cameraBuffer[i];
					printf(" %02u%02u%02u",v >> 11, (v >> 5) & 0x3f, v & 0x1f);
				}
				printf("\r\n");
				prev_ticks = ticks;
				frame_cnt = 0;
			}
			/* TODO - resume when CM7 signals it is done with this camera data - special case when recording to USB stick */
			if ((hdcmi.State == HAL_DCMI_STATE_SUSPENDED) && (media != NULL) && (sharedData.CM4_to_CM7_USB_info & USB_INFO_RECORDING))
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

/*
Working Mode Register Map
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Address Name      Default | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |  Host
                   Value  |      |      |      |      |      |      |      |      | Access
==========================================================================================
0x00    DEV_MODE   0x00   |      | [2:0]Device Mode   |                           |  R/W
0x01    GEST_ID    0x00   | [7:0]Gesture ID                                       |   R
0x02    TD_STATUS  0x00   |                           | [3:0] Number of           |   R
                          |                           |       touch points        |
0x03    P1_XH      0xFF   | [7:6]1st    |             | [3:0] 1st Touch           |   R
                          |  Event Flag |             |       X Position[11:8]    |
0x04    P1_XL      0xFF   | [7:0] 1st Touch X Position                            |   R
0x05    P1_YH      0xFF   | [7:4] 1st Touch ID        | [3:0] 1st Touch           |   R
                          |                           |       Y Position[11:8]    |
0x06    P1_YL      0xFF   | [7:0] 1st Touch Y Position                            |   R
0x07    P1_WEIGHT  0xFF   | [7:0] 1st Touch Weight                                |   R
0x08    P1_MISC    0xFF   | [7:4] 1st Touch Area      |                           |   R
0x09    P2_XH      0xFF   | [7:6]2nd    |             | [3:0]2nd Touch            |   R
                          |  Event Flag |             |       X Position[11:8]    |
0x0A    P2_XL      0xFF   | [7:0] 2nd Touch X Position                            |   R
0x0B    P2_YH      0xFF   | [7:4] 2ndTouch ID         | [3:0] 2nd Touch           |   R
                          |                           |       Y Position[11:8]    |
0x0C    P2_YL      0xFF   | [7:0] 2nd Touch Y Position                            |   R
0x0D    P2_WEIGHT  0xFF   | [7:0] 2nd Touch Weight                                |   R
0x0E    P2_MISC    0xFF   | [7:4] 2nd Touch Area      |                           |   R
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
void tx_cm4_i2c4_thread_entry(ULONG thread_input)
{
	ULONG actual_events;
	memset((void *)sharedData.touchData, 0, sizeof(sharedData.touchData));
	/* Infinite loop */
	for(;;)
	{
		UINT status = tx_event_flags_get(&cm4_event_group, 0x2, TX_AND_CLEAR, &actual_events, TX_WAIT_FOREVER);
		// TODO - could read all the touch info and gestures maybe ?
		if (status == TX_SUCCESS)
		{
			// read status register
			uint8_t nb_touch;
			uint8_t data[4];
			uint32_t touchX, touchY;
			//ULONG ticks = tx_time_get();
		  if (HAL_I2C_Mem_Read(&hi2c4, TS_I2C_ADDRESS, FT6X06_TD_STAT_REG, I2C_MEMADD_SIZE_8BIT, &nb_touch, 1, 1000) == HAL_OK)
		  {
		  	nb_touch &= FT6X06_TD_STATUS_BIT_MASK;
		    if(HAL_I2C_Mem_Read(&hi2c4, TS_I2C_ADDRESS, FT6X06_P1_XH_REG, I2C_MEMADD_SIZE_8BIT, data, 4, 1000) == HAL_OK)
		    {
		      touchX = (((uint32_t)data[0] & FT6X06_P1_XH_TP_BIT_MASK) << 8) | ((uint32_t)data[1] & FT6X06_P1_XL_TP_BIT_MASK);
		      touchY = (((uint32_t)data[2] & FT6X06_P1_YH_TP_BIT_MASK) << 8) | ((uint32_t)data[3] & FT6X06_P1_YL_TP_BIT_MASK);
		      sharedData.touchData[0] = nb_touch;
		      sharedData.touchData[1] = touchX;
		      sharedData.touchData[2] = touchY;
		      sharedData.touchData[3] += 1;
		      /* Signal CM7 that we have new touch data */
		      HAL_HSEM_FastTake(HSEM_ID_1);
		      HAL_HSEM_Release(HSEM_ID_1,0);
		    }
		  }
		}
		else
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
}

void tx_cm4_uart_thread_entry(ULONG thread_input)
{
	UINT status;
	ULONG actual_events;
	UCHAR read_buffer[32];
	CHAR data[] = "This is ThreadX working on STM32 CM4";

	printf("\r\n%s\r\nStarting Run on %s\r\n", data, _tx_version_id);
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
		if (dbgBufCnt > 0)
		{
			status = tx_mutex_get(&mutex_0, TX_WAIT_FOREVER);
			if (status != TX_SUCCESS)
			{
				Error_Handler();
			}
			/* Send the data via UART */
			if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dbgBuf, dbgBufCnt) != HAL_OK)
			{
				Error_Handler();
			}
			/* Wait until the requested flag TX_NEW_TRANSMITTED_DATA is received */
			if (tx_event_flags_get(&cm4_event_group, 0x4, TX_OR_CLEAR,
														 &actual_events, TX_WAIT_FOREVER) != TX_SUCCESS)
			{
				Error_Handler();
			}
			dbgBufCnt = 0;
			status = tx_mutex_put(&mutex_0);
			if (status != TX_SUCCESS)
			{
				Error_Handler();
			}
		}
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 20);
	}
}

void tx_cm4_cm7_printf_thread_entry(ULONG thread_input)
{
	ULONG actual_events;
	/* Infinite loop */
	for(;;)
	{
		UINT status = tx_event_flags_get(&cm4_event_group, 0x8, TX_AND_CLEAR, &actual_events, TX_WAIT_FOREVER);
		if (status == TX_SUCCESS)
		{
			if (sharedData.writePos != sharedData.readPos)
			{
				int r = sharedData.readPos;
				int w = sharedData.writePos;
				status = tx_mutex_get(&mutex_0, TX_WAIT_FOREVER);
				if (status != TX_SUCCESS)
				{
					Error_Handler();
				}
				// leave room for escape sequences
				if (dbgBufCnt < 246)
				{
					memcpy(dbgBuf + dbgBufCnt, "\e[31m", 5);
					dbgBufCnt += 5;
					while (r != w)
					{
						if (dbgBufCnt < 251) dbgBuf[dbgBufCnt++] = sharedData.charBuffer[r];
						if (sharedData.charBuffer[r] == '\n')
						{
							if (dbgBufCnt < 252) dbgBuf[dbgBufCnt++] = '\r';
						}
						r = (r + 1) & 0xff;
					}
					memcpy(dbgBuf + dbgBufCnt, "\e[0m", 4);
					dbgBufCnt += 4;
				}
				status = tx_mutex_put(&mutex_0);
				if (status != TX_SUCCESS)
				{
					Error_Handler();
				}
				sharedData.readPos = w;
			}
		}
		else
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
}

// vim: noet ci pi sts=0 sw=2 ts=2
/* USER CODE END 1 */

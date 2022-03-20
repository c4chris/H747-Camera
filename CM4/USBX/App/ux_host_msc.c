/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_host_msc.c
  * @author  MCD Application Team
  * @brief   USBX host applicative file
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
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "app_filex.h"
#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern UX_HOST_CLASS_STORAGE        *storage;
extern FX_MEDIA                     *media;
extern FX_FILE                      *file;
extern TX_QUEUE                     ux_app_MsgQueue_msc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void  msc_process_thread_entry(ULONG arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  msc_process_thread_entry .
  * @param  ULONG arg
  * @retval Void
  */
void  msc_process_thread_entry(ULONG arg)
{

  UINT        status;

  while(1)
  {
	  ULONG msg;
    status = tx_queue_receive(&ux_app_MsgQueue_msc, &msg, TX_WAIT_FOREVER);
	  if ((storage != NULL) && (media != NULL) && (msg == APP_MSG_MEDIA_READY))
	  {
		  ULONG64 space = 0;
		  CHAR directory_name[FX_MAX_LONG_NAME_LEN];
		  UINT attributes;
		  ULONG size;
		  UINT year, month, day;
		  UINT hour, minute, second;
		  printf("USB inserted\r\n");
		  sharedData.CM4_to_CM7_USB_info |= USB_INFO_STICK_INSERTED;
		  sharedData.CM4_to_CM7_USB_stored_count = 0;
		  fx_media_extended_space_available(media, &space);
		  sharedData.CM4_to_CM7_USB_free_size_kb = space / 1024;
		  /* signal new USB state to CM7 */
		  HAL_HSEM_FastTake(HSEM_ID_3);
		  HAL_HSEM_Release(HSEM_ID_3, 0); 
		  /* need to use newlib instead of newlib-nano if we want to use %llu in printf - for now just print kb instead of b */
		  printf("%lu kbytes space available\r\n", sharedData.CM4_to_CM7_USB_free_size_kb);
		  status = fx_directory_first_full_entry_find(media, directory_name, &attributes, &size, &year, &month, &day, &hour, &minute, &second);
		  if (status == FX_SUCCESS)
		  {
			  printf("%.20s %u %lu %u %u %u %u:%u:%u\r\n", directory_name, attributes, size, year, month, day, hour, minute, second);
			  status = fx_directory_next_full_entry_find(media, directory_name, &attributes, &size, &year, &month, &day, &hour, &minute, &second);
		  }
	  }
	  else if ((storage != NULL) && (media != NULL) && (msg == APP_MSG_START_RECORDING))
	  {
		  /* Create a file */
		  file = App_File_Create(media);

		  /* check status */
		  if (file != NULL)
		  {
			  USBH_UsrLog("File CAMERA.RAW Created");
				sharedData.CM4_to_CM7_USB_info |= USB_INFO_RECORDING;
				/* signal new USB state to CM7 */
				HAL_HSEM_FastTake(HSEM_ID_3);
				HAL_HSEM_Release(HSEM_ID_3, 0); 
		  }
	  }
	  else if ((storage != NULL) && (media != NULL) && (msg == APP_MSG_STOP_RECORDING))
	  {
			/* signal end of recording */
			sharedData.CM4_to_CM7_USB_info &= ~USB_INFO_RECORDING;

		  /* wait in case there is data currently being written */
		  while (sharedData.CM4_USB_writing)
		  {
				tx_thread_sleep(MS_TO_TICK(10));
		  }

		  /* Close the file */
		  status = App_File_Close(media);
		  file = NULL;

		  /* check status */
		  if (status == FX_SUCCESS)
		  {
			  USBH_UsrLog("File CAMERA.RAW Closed");
				/* signal new USB state to CM7 */
				HAL_HSEM_FastTake(HSEM_ID_3);
				HAL_HSEM_Release(HSEM_ID_3, 0); 
		  }
	  }
    else
    {
      tx_thread_sleep(MS_TO_TICK(10));
    }
  }

}

/* USER CODE END 1 */

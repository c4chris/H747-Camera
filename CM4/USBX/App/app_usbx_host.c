/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.c
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

#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define APP_QUEUE_SIZE                               5
#define USBX_APP_STACK_SIZE                          (2 * 1024)
#define USBX_MEMORY_SIZE                             (64 * 1024)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static TX_THREAD ux_host_app_thread;

/* USER CODE BEGIN PV */

extern HCD_HandleTypeDef                  hhcd_USB_OTG_HS;
TX_THREAD                                 ux_app_thread;
TX_THREAD                                 msc_app_thread;
TX_QUEUE                                  ux_app_MsgQueue;
TX_QUEUE                                  ux_app_MsgQueue_msc;
UX_HOST_CLASS_STORAGE                     *storage;
UX_HOST_CLASS_STORAGE_MEDIA               *storage_media;
FX_MEDIA                                  *media;
FX_FILE                                   *file;
__ALIGN_BEGIN ux_app_devInfotypeDef        ux_dev_info  __ALIGN_END;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID app_ux_host_thread_entry(ULONG thread_input);
static UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);
static VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code);
/* USER CODE BEGIN PFP */

UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);
extern void msc_process_thread_entry(ULONG arg);
extern void Error_Handler(void);

/* USER CODE END PFP */

/**
  * @brief  Application USBX Host Initialization.
  * @param  memory_ptr: memory pointer
  * @retval status
  */
UINT MX_USBX_Host_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  UCHAR *pointer;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_USBX_Host_Init0 */

  /* USER CODE END MX_USBX_Host_Init0 */

  /* Allocate the stack for USBX Memory */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_HOST_MEMORY_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    /* USER CODE BEGIN USBX_ALLOCATE_STACK_ERORR */
    return TX_POOL_ERROR;
    /* USER CODE END USBX_ALLOCATE_STACK_ERORR */
  }

  /* Initialize USBX Memory */
  if (ux_system_initialize(pointer, USBX_HOST_MEMORY_STACK_SIZE, UX_NULL, 0) != UX_SUCCESS)
  {
    /* USER CODE BEGIN USBX_SYSTEM_INITIALIZE_ERORR */
    return UX_ERROR;
    /* USER CODE END USBX_SYSTEM_INITIALIZE_ERORR */
  }

  /* Install the host portion of USBX */
  if (ux_host_stack_initialize(ux_host_event_callback) != UX_SUCCESS)
  {
    /* USER CODE BEGIN USBX_HOST_INITIALIZE_ERORR */
    return UX_ERROR;
    /* USER CODE END USBX_HOST_INITIALIZE_ERORR */
  }

  /* Register a callback error function */
  ux_utility_error_callback_register(&ux_host_error_callback);

  /* Initialize the host storage class */
  if (ux_host_stack_class_register(_ux_system_host_class_storage_name,
                                   ux_host_class_storage_entry) != UX_SUCCESS)
  {
    /* USER CODE BEGIN USBX_HOST_STORAGE_REGISTER_ERORR */
    return UX_ERROR;
    /* USER CODE END USBX_HOST_STORAGE_REGISTER_ERORR */
  }

  /* Allocate the stack for host application main thread */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, UX_HOST_APP_THREAD_STACK_SIZE,
                       TX_NO_WAIT) != TX_SUCCESS)
  {
    /* USER CODE BEGIN MAIN_THREAD_ALLOCATE_STACK_ERORR */
    return TX_POOL_ERROR;
    /* USER CODE END MAIN_THREAD_ALLOCATE_STACK_ERORR */
  }

  /* Create the host application main thread */
  if (tx_thread_create(&ux_host_app_thread, UX_HOST_APP_THREAD_NAME, app_ux_host_thread_entry,
                       0, pointer, UX_HOST_APP_THREAD_STACK_SIZE, UX_HOST_APP_THREAD_PRIO,
                       UX_HOST_APP_THREAD_PREEMPTION_THRESHOLD, UX_HOST_APP_THREAD_TIME_SLICE,
                       UX_HOST_APP_THREAD_START_OPTION) != TX_SUCCESS)
  {
    /* USER CODE BEGIN MAIN_THREAD_CREATE_ERORR */
    return TX_THREAD_ERROR;
    /* USER CODE END MAIN_THREAD_CREATE_ERORR */
  }

  /* USER CODE BEGIN MX_USBX_Host_Init1 */

  /* Register all the USB host controllers available in this system. */
  if (ux_host_stack_hcd_register(_ux_system_host_hcd_stm32_name,
                                 _ux_hcd_stm32_initialize,
                                 USB_OTG_HS_PERIPH_BASE,
                                 (ULONG)&hhcd_USB_OTG_HS) != UX_SUCCESS)
  {
    return UX_ERROR;
  }

  /* Enable USB Global Interrupt */
  HAL_HCD_Start(&hhcd_USB_OTG_HS);

  /* USER CODE END MX_USBX_Host_Init1 */

  return ret;
}

/**
  * @brief  Function implementing app_ux_host_thread_entry.
  * @param  thread_input: User thread input parameter.
  * @retval none
  */
static VOID app_ux_host_thread_entry(ULONG thread_input)
{
  /* USER CODE BEGIN app_ux_host_thread_entry */

  TX_PARAMETER_NOT_USED(thread_input);

  /* Start Application */
  USBH_UsrLog(" **** USB OTG HS in FS MSC Host **** \n");
  USBH_UsrLog("USB Host library started.\n");

  /* Initialize Application and MSC process */
  USBH_UsrLog("Starting MSC Application");
  USBH_UsrLog("Connect your MSC Device\n");

  while (1)
  {
    /* wait for message queue from callback event */
    if (tx_queue_receive(&ux_app_MsgQueue, &ux_dev_info, TX_WAIT_FOREVER)!= TX_SUCCESS)
    {
     Error_Handler();
    }

    if (ux_dev_info.Dev_state == Device_connected)
    {
      switch (ux_dev_info.Device_Type)
      {
        case MSC_Device :
          if (media ==NULL)
          {
						break;
          }
          else
          {
	          ULONG msg = APP_MSG_MEDIA_READY;
						/* Device_information */
						USBH_UsrLog("USB Mass Storage Device Found");
						USBH_UsrLog("PID: %#x ", (UINT)storage -> ux_host_class_storage_device -> ux_device_descriptor.idProduct);
						USBH_UsrLog("VID: %#x ", (UINT)storage -> ux_host_class_storage_device -> ux_device_descriptor.idVendor);

						/* start File operations */
						USBH_UsrLog("\n*** Start Files operations ***\n");
						/* send queue to msc_app_process*/
						tx_queue_send(&ux_app_MsgQueue_msc, &msg, TX_NO_WAIT);
          }
          break;

        case Unknown_Device :
          USBH_ErrLog("!! Unsupported MSC_Device plugged !!");
          ux_dev_info.Dev_state = No_Device;
          break;

        case Unsupported_Device :
          USBH_ErrLog("!! Unabble to start Device !!");
          break;

        default :
          break;
      }
    }
    else
    {
      /*clear storage instance*/
      storage_media  = NULL;
      media = NULL;
      /* TODO - report device was disconnected */
      tx_thread_sleep(50);
    }
  }
  /* USER CODE END app_ux_host_thread_entry */
}

/**
  * @brief  ux_host_event_callback
  *         This callback is invoked to notify application of instance changes.
  * @param  event: event code.
  * @param  current_class: Pointer to class.
  * @param  current_instance: Pointer to class instance.
  * @retval status
  */
UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN ux_host_event_callback0 */

  /* USER CODE END ux_host_event_callback0 */

  switch (event)
  {
    case UX_DEVICE_INSERTION:

      /* USER CODE BEGIN UX_DEVICE_INSERTION */

      /* Get current Hid Class */
      UX_HOST_CLASS *msc_class;
      status = ux_host_stack_class_get(_ux_system_host_class_storage_name, &msc_class);

      if (status == UX_SUCCESS)
      {
        if ((msc_class == current_class) && (storage == NULL))
        {
          /* get current msc Instance */
          storage = current_instance;

          if (storage == NULL)
          {
            USBH_UsrLog("unable to start media ");
            ux_dev_info.Device_Type = Unsupported_Device;
            ux_dev_info.Dev_state   = Device_connected;
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
          else
          {
            /* get the storage media */
            storage_media = (UX_HOST_CLASS_STORAGE_MEDIA *)msc_class->ux_host_class_media;
            media = &storage_media->ux_host_class_storage_media;
          }

          if (storage->ux_host_class_storage_state != (ULONG) UX_HOST_CLASS_INSTANCE_LIVE)
          {
            ux_dev_info.Device_Type = Unsupported_Device;
            ux_dev_info.Dev_state = Device_connected;
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
          else
          {
            /* USB _MSC_ Device found */
            USBH_UsrLog("USB Device Plugged");

            /* update USB device Type */
            ux_dev_info.Device_Type = MSC_Device;
            ux_dev_info.Dev_state = Device_connected ;

            /* put a message queue to usbx_app_thread_entry */
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
        }
      }
      else
      {
        /* No MSC class found */
        USBH_ErrLog("NO MSC Class found");
      }

      /* USER CODE END UX_DEVICE_INSERTION */

      break;

    case UX_DEVICE_REMOVAL:

      /* USER CODE BEGIN UX_DEVICE_REMOVAL */

      if (current_instance == storage)
      {
        /* free Instance */
        storage = NULL;
        USBH_UsrLog("USB Device Unplugged");
        ux_dev_info.Dev_state   = No_Device;
        ux_dev_info.Device_Type = Unknown_Device;
        tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
      }

      /* USER CODE END UX_DEVICE_REMOVAL */

      break;

#if defined (UX_HOST_CLASS_STORAGE_NO_FILEX)
    case UX_STORAGE_MEDIA_INSERTION:

      /* USER CODE BEGIN UX_STORAGE_MEDIA_INSERTION */

      /* USER CODE END UX_STORAGE_MEDIA_INSERTION */

      break;

    case UX_STORAGE_MEDIA_REMOVAL:

      /* USER CODE BEGIN UX_STORAGE_MEDIA_REMOVAL */

      /* USER CODE END UX_STORAGE_MEDIA_REMOVAL */

      break;
#endif

    case UX_DEVICE_CONNECTION:

      /* USER CODE BEGIN UX_DEVICE_CONNECTION */

      /* USER CODE END UX_DEVICE_CONNECTION */

      break;

    case UX_DEVICE_DISCONNECTION:

      /* USER CODE BEGIN UX_DEVICE_DISCONNECTION */

      /* USER CODE END UX_DEVICE_DISCONNECTION */

      break;

    default:

      /* USER CODE BEGIN EVENT_DEFAULT */

      /* USER CODE END EVENT_DEFAULT */

      break;
  }

  /* USER CODE BEGIN ux_host_event_callback1 */

  /* USER CODE END ux_host_event_callback1 */

  return status;
}

/**
  * @brief ux_host_error_callback
  *         This callback is invoked to notify application of error changes.
  * @param  system_level: system level parameter.
  * @param  system_context: system context code.
  * @param  error_code: error event code.
  * @retval Status
  */
VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code)
{
  /* USER CODE BEGIN ux_host_error_callback0 */
  UX_PARAMETER_NOT_USED(system_level);
  UX_PARAMETER_NOT_USED(system_context);
  /* USER CODE END ux_host_error_callback0 */

  switch (error_code)
  {
    case UX_DEVICE_ENUMERATION_FAILURE:

      /* USER CODE BEGIN UX_DEVICE_ENUMERATION_FAILURE */

      ux_dev_info.Device_Type = Unknown_Device;
      ux_dev_info.Dev_state   = Device_connected;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);

      /* USER CODE END UX_DEVICE_ENUMERATION_FAILURE */

      break;

    case  UX_NO_DEVICE_CONNECTED:

      /* USER CODE BEGIN UX_NO_DEVICE_CONNECTED */

      USBH_UsrLog("USB Device disconnected");

      /* USER CODE END UX_NO_DEVICE_CONNECTED */

      break;

    default:

      /* USER CODE BEGIN ERROR_DEFAULT */

      /* USER CODE END ERROR_DEFAULT */

      break;
  }

  /* USER CODE BEGIN ux_host_error_callback1 */

  /* USER CODE END ux_host_error_callback1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

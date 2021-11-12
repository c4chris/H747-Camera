/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.h
  * @author  MCD Application Team
  * @brief   USBX Host applicative header file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_USBX_HOST_H__
#define __APP_USBX_HOST_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ux_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ux_system.h"
#include "ux_utility.h"
#include "ux_hcd_stm32.h"
#include "stm32h7xx_hal.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"
#include "ux_host_stack.h"
#include "gx_api.h"
#include "gx_display.h"
#include "H747_Camera_specifications.h"
#include "H747_Camera_resources.h"
#include "main.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern TX_EVENT_FLAGS_GROUP cm7_event_group;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define USBH_UsrLog(...)   printf(__VA_ARGS__);\
                           printf("\n");

#define USBH_ErrLog(...)   printf("ERROR: ") ;\
                           printf(__VA_ARGS__);\
                           printf("\n");
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_USBX_Host_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */
UINT  MX_USB_Host_Init(void);
void  USBH_DriverVBUS(uint8_t state);
void  usbx_app_thread_entry(ULONG arg);
void  hid_mouse_thread_entry(ULONG arg);
void  hid_keyboard_thread_entry(ULONG arg);
void  hid_touchscreen_thread_entry(ULONG arg);
VOID  ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code);
UINT  ux_host_event_callback(ULONG event, UX_HOST_CLASS *p_host_class, VOID *p_instance);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LCD_FRAME_BUFFER        LCD_LAYER_0_ADDRESS
#define CLOCK_TIMER             0x01

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

typedef enum
{
  USB_VBUS_FALSE = 0,
  USB_VBUS_TRUE,
} USB_VBUS_State;

typedef enum
{
  Mouse_Device = 1,
  Keyboard_Device,
	Touchscreen_Device,
  Unknown_Device,
} HID_Device_Type;

typedef enum
{
  Device_disconnected = 1,
  Device_connected,
  No_Device,
} Device_state;

typedef struct
{
  HID_Device_Type Device_Type;
  Device_state    Dev_state;
} ux_app_devInfotypeDef;

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_USBX_HOST_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

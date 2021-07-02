/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.c
  * @author  MCD Application Team
  * @brief   USBX host applicative file
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
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USBX_APP_STACK_SIZE                          1024
#define USBX_MEMORY_SIZE                             (64 * 1024)
#define APP_QUEUE_SIZE                               5

#define DEFAULT_STACK_SIZE                           (1 * 1024)
/* fx_sd_thread priority */
#define DEFAULT_THREAD_PRIO                          10

/* fx_sd_thread preemption priority */
#define DEFAULT_PREEMPTION_THRESHOLD                 DEFAULT_THREAD_PRIO

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern HCD_HandleTypeDef                 hhcd_USB_OTG_HS;
TX_THREAD                                ux_app_thread;
TX_THREAD                                mouse_app_thread;
TX_QUEUE                                 ux_app_MsgQueue;
UX_HOST_CLASS_HID                        *hid;
UX_HOST_CLASS_HID_CLIENT                 *hid_client;
UX_HOST_CLASS_HID_MOUSE                  *mouse;

__ALIGN_BEGIN ux_app_devInfotypeDef       ux_dev_info  __ALIGN_END;

/* Define ThreadX global data structures.  */
TX_THREAD       cm7_main_thread;
TX_THREAD       cm7_lcd_thread;
TX_EVENT_FLAGS_GROUP cm7_event_group;

/* Define the GUIX resources. */

/* Define the root window pointer. */

GX_WINDOW_ROOT *root_window;

/* data comning from CM4 core */
__attribute__((section(".sram3.bridgeError"))) volatile unsigned int bridgeError[4];
__attribute__((section(".sram3.bridgeCount"))) volatile unsigned int bridgeCount[4];
__attribute__((section(".sram3.bridgeStale"))) volatile unsigned int bridgeStale[4];
__attribute__((section(".sram3.bridgeBadstatus"))) volatile unsigned int bridgeBadstatus[4];
__attribute__((section(".sram3.bridgeValue"))) volatile uint32_t bridgeValue[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

extern void Error_Handler(void);

void tx_cm7_main_thread_entry(ULONG thread_input);
void tx_cm7_lcd_thread_entry(ULONG thread_input);
void Error_Handler(void);
static void stm32h7_32argb_buffer_toggle(GX_CANVAS *canvas, GX_RECTANGLE *dirty_area);
UINT stm32h7_graphics_driver_setup_32argb(GX_DISPLAY *display);

/* USER CODE END PFP */
/**
  * @brief  Application USBX Host Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_USBX_Host_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  /* USER CODE BEGIN App_USBX_Host_Init */
  CHAR *pointer;

  /* Allocate the stack for thread 0. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Initialize USBX memory. */
  if (ux_system_initialize(pointer, USBX_MEMORY_SIZE, UX_NULL, 0) != UX_SUCCESS)

  {
    ret = UX_ERROR;
  }

  /* register a callback error function */
  _ux_utility_error_callback_register(&ux_host_error_callback);

  /* Allocate the stack for thread 0. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create the main App thread. */
  if (tx_thread_create(&ux_app_thread, "thread 0", usbx_app_thread_entry, 0,
                       pointer, USBX_APP_STACK_SIZE, 25, 25, 1,
                       TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

  /* Allocate the stack for thread 1. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create the HID mouse App thread. */
  if (tx_thread_create(&mouse_app_thread, "thread 1", hid_mouse_thread_entry, 0,
                       pointer, USBX_APP_STACK_SIZE, 30, 30, 1,
                       TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

  /* Allocate Memory for the Queue */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       APP_QUEUE_SIZE * sizeof(ux_app_devInfotypeDef), TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create the MsgQueue */
  if (tx_queue_create(&ux_app_MsgQueue, "Message Queue app", sizeof(ux_app_devInfotypeDef),
                      pointer, APP_QUEUE_SIZE * sizeof(ux_app_devInfotypeDef)) != TX_SUCCESS)
  {
    ret = TX_QUEUE_ERROR;
  }

  /*Allocate memory for fx_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check FILEX_DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&cm7_main_thread, "tx_cm7_main_thread", tx_cm7_main_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create an event flags group. */
  ret = tx_event_flags_create(&cm7_event_group, "cm7_event_group_name");

  /* If status equals TX_SUCCESS, my_event_group is ready for get and set services. */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }


  /*Allocate memory for fx_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check FILEX_DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&cm7_lcd_thread, "tx_cm7_lcd_thread", tx_cm7_lcd_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* USER CODE END App_USBX_Host_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Application_thread_entry .
  * @param  ULONG arg
  * @retval Void
  */
void  usbx_app_thread_entry(ULONG arg)
{
  /* Initialize USBX_Host */
  MX_USB_Host_Init();

  /* Start Application Message */
  USBH_UsrLog(" **** USB OTG HS in FS HID Host **** \n");
  USBH_UsrLog("USB Host library started.\n");

  /* Wait for Device to be attached */
  USBH_UsrLog("Starting HID Application");
  USBH_UsrLog("Connect your HID Device\n");

  while (1)
  {
    /* Wait for a hid device to be connected */
    if (tx_queue_receive(&ux_app_MsgQueue, &ux_dev_info, TX_WAIT_FOREVER)!= TX_SUCCESS)
    {
     /*Error*/
     Error_Handler();
    }

    if (ux_dev_info.Dev_state == Device_connected)
    {
      switch (ux_dev_info.Device_Type)
      {
        case Mouse_Device :
          mouse = hid_client-> ux_host_class_hid_client_local_instance;
          USBH_UsrLog("HID_Mouse_Device");
          USBH_UsrLog("PID: %#x ", (UINT)mouse ->ux_host_class_hid_mouse_hid->ux_host_class_hid_device->ux_device_descriptor.idProduct);
          USBH_UsrLog("VID: %#x ", (UINT)mouse ->ux_host_class_hid_mouse_hid->ux_host_class_hid_device->ux_device_descriptor.idVendor);
          USBH_UsrLog("USB HID Host Mouse App...");
          USBH_UsrLog("Mouse is ready...\n");
          break;

        case Unknown_Device :
          USBH_ErrLog("Unsupported USB device");
          break;

        default :
          break;
      }
    }
    else
    {
      /* clear hid_client local instance */
      mouse = NULL;
    }
  }
}

/**
* @brief ux_host_event_callback
* @param ULONG event
           This parameter can be one of the these values:
             1 : UX_DEVICE_INSERTION
             2 : UX_DEVICE_REMOVAL
             3 : UX_HID_CLIENT_INSERTION
             4 : UX_HID_CLIENT_REMOVAL
         UX_HOST_CLASS * Current_class
         VOID * Current_instance
* @retval Status
*/
UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *Current_class, VOID *Current_instance)
{
  UINT status;
  UX_HOST_CLASS *hid_class;

  switch (event)
  {
    case UX_DEVICE_INSERTION :
      /* Get current Hid Class */
      status = ux_host_stack_class_get(_ux_system_host_class_hid_name, &hid_class);

      if (status == UX_SUCCESS)
      {
        if ((hid_class == Current_class) && (hid == NULL))
        {
          /* Get current Hid Instance */
          hid = Current_instance;
          /* Get the HID Client */
          hid_client = hid ->ux_host_class_hid_client;

          if (hid->ux_host_class_hid_client->ux_host_class_hid_client_status != (ULONG) UX_HOST_CLASS_INSTANCE_LIVE)
          {
            ux_dev_info.Device_Type = Unknown_Device;
          }
          /* Check the HID_client if this is a HID mouse device. */
          if (ux_utility_memory_compare(hid_client -> ux_host_class_hid_client_name,
                                        _ux_system_host_class_hid_client_mouse_name,
                                        ux_utility_string_length_get(_ux_system_host_class_hid_client_mouse_name)) == UX_SUCCESS)
          {
            /* update HID device Type */
            ux_dev_info.Device_Type = Mouse_Device;

            /* put a message queue to usbx_app_thread_entry */
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
          else
          {
            ux_dev_info.Device_Type = Unknown_Device;
            ux_dev_info.Dev_state = Device_connected;
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
        }
      }
      else
      {
        /* No HID class found */
        USBH_ErrLog("NO HID Class found");
      }
      break;

    case UX_DEVICE_REMOVAL :

      if (Current_instance == hid)
      {
        /* Free Instance */
        hid = NULL;
        USBH_UsrLog("USB Device Unplugged");
        ux_dev_info.Dev_state   = No_Device;
        ux_dev_info.Device_Type = Unknown_Device;
      }
      break;

    case UX_HID_CLIENT_INSERTION :
      USBH_UsrLog("HID Client Plugged");
      ux_dev_info.Dev_state = Device_connected;
      break;

    case UX_HID_CLIENT_REMOVAL:
      USBH_UsrLog("HID Client Unplugged");
      ux_dev_info.Dev_state   =  Device_disconnected;
      ux_dev_info.Device_Type =  Unknown_Device;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);

      break;

    default:
      break;

  }

  return (UINT) UX_SUCCESS;
}

/**
* @brief ux_host_error_callback
* @param ULONG event
         UINT system_context
         UINT error_code
* @retval Status
*/
VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code)
{
  switch (error_code)
  {
    case UX_DEVICE_ENUMERATION_FAILURE :

      ux_dev_info.Device_Type = Unknown_Device;
      ux_dev_info.Dev_state   = Device_connected;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
      break;

    case  UX_NO_DEVICE_CONNECTED :
      USBH_UsrLog("USB Device disconnected");
      break;

    default:
      break;
  }
}

/**
  * @brief MX_USB_Host_Init
  *        Initialization of USB device.
  * Init USB Host Library, add supported class and start the library
  * @retval None
  */
UINT MX_USB_Host_Init(void)
{
  UINT ret = UX_SUCCESS;

  /* The code below is required for installing the host portion of USBX. */
  if (ux_host_stack_initialize(ux_host_event_callback) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Register hid class. */
  if (ux_host_stack_class_register(_ux_system_host_class_hid_name,
                                   _ux_host_class_hid_entry) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Register HID Mouse client */
  if (ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name,
                                        ux_host_class_hid_mouse_entry) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Register all the USB host controllers available in this system.  */
  if (ux_host_stack_hcd_register(_ux_system_host_hcd_stm32_name,
                                 _ux_hcd_stm32_initialize, USB_OTG_HS_PERIPH_BASE,
                                 (ULONG)&hhcd_USB_OTG_HS) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Drive vbus */
  USBH_DriverVBUS(USB_VBUS_TRUE);

  /* Enable USB Global Interrupt*/
  HAL_HCD_Start(&hhcd_USB_OTG_HS);

  return ret;
}

/**
* @brief  Drive VBUS.
* @param  state : VBUS state
*          This parameter can be one of the these values:
*           1 : VBUS Active
*           0 : VBUS Inactive
* @retval Status
*/
void USBH_DriverVBUS(uint8_t state)
{

  if (state == USB_VBUS_TRUE)
  {
    /* Drive high Charge pump */
    /* Add IOE driver control */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
  }
  else
  {
    /* Drive low Charge pump */
    /* Add IOE driver control */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
  }

  HAL_Delay(200);
}

void tx_cm7_main_thread_entry(ULONG thread_input)
{
#if 0
	uint16_t curTouch[4];
	uint16_t prevTouch[4] = {0};
#endif
	ULONG toggleTicks = tx_time_get();

  /* Infinite Loop */
  for( ;; )
  {
  	ULONG ticks = tx_time_get();
  	if (ticks >= toggleTicks)
  	{
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  		toggleTicks = ticks + TX_TIMER_TICKS_PER_SECOND;
  	}
#if 0
  	if (touchData[3] != prevTouch[3])
  	{
  		/* Send a pen event for processing. */
  		GX_EVENT e = {0};
  		//e.gx_event_display_handle = LCD_FRAME_BUFFER;
    	curTouch[0] = touchData[0];
    	curTouch[1] = touchData[1];
    	curTouch[2] = touchData[2];
    	curTouch[3] = touchData[3];
  		e.gx_event_payload.gx_event_pointdata.gx_point_x = curTouch[2];       // Y value of touchscreen driver
  		e.gx_event_payload.gx_event_pointdata.gx_point_y = 480 - curTouch[1]; // X value of touchscreen driver, but reversed
  		if (curTouch[0] == 0)
  		{
    		e.gx_event_type = GX_EVENT_PEN_UP; // pen UP
  			//BSP_LED_Off(LED_RED);
  		}
  		else if (prevTouch[0] == 0)
  		{
    		e.gx_event_type = GX_EVENT_PEN_DOWN; // pen DOWN
  			//BSP_LED_On(LED_RED);
  		}
  		else
    		e.gx_event_type = GX_EVENT_PEN_DRAG; // pen DRAG
  		/* Push the event to event pool. */
  		if (e.gx_event_type == GX_EVENT_PEN_DRAG)
    		gx_system_event_fold(&e);
  		else
  			gx_system_event_send(&e);
  		memcpy(prevTouch, curTouch, sizeof(curTouch));
  	}
#endif
  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 5);
  }
}

void tx_cm7_lcd_thread_entry(ULONG thread_input)
{
  extern GX_STUDIO_DISPLAY_INFO H747_WeighingStation_display_table[]; // meh...

  /* Initialize GUIX. */
  gx_system_initialize();

  /* Configure the main display. */
  H747_WeighingStation_display_table[MAIN_DISPLAY].canvas_memory = (GX_COLOR *) Buffers[0]; // I think...
  gx_studio_display_configure(MAIN_DISPLAY,                         /* Display to configure*/
															stm32h7_graphics_driver_setup_32argb, /* Driver to use */
                              LANGUAGE_ENGLISH,                     /* Language to install */
                              MAIN_DISPLAY_DEFAULT_THEME,           /* Theme to install */
                              &root_window);                        /* Root window pointer */

  /* Create the screen - attached to root window. */

  gx_studio_named_widget_create("main_window", (GX_WIDGET *) root_window, GX_NULL);

  /* Show the root window to make it visible. */
  gx_widget_show(root_window);

  /* Let GUIX run. */
  gx_system_start();
}

/*************************************************************************************/
VOID weight_update()
{
	uint32_t low[4] = { 950, 950, 950, 950 };
	uint32_t total = 0;
  /* Set a value to "my_numeric_pix_prompt". */
#if 0
	for (unsigned int i = 0; i < 4; i++)
	{
		uint32_t weight = (bridgeValue[i] >> 16) & 0x3fff;
		if (weight < low[i])
			weight = 0;
		else
			weight -= low[i];
		weight *= 5000;
		weight /= 14000;
		total += weight;
	}
	total /= 10;
#endif
	ULONG ticks = tx_time_get();
	total = ticks % 10000;
  gx_numeric_pixelmap_prompt_value_set(&main_window.main_window_weight_prompt, total);
}

/*************************************************************************************/
UINT main_screen_event_handler(GX_WINDOW *window, GX_EVENT *event_ptr)
{
	switch (event_ptr->gx_event_type)
	{
		case GX_EVENT_SHOW:
			/* Set current weight. */
			weight_update();

			/* Start a timer to update weight. */
			gx_system_timer_start(&main_window, CLOCK_TIMER, GX_TICKS_SECOND / 2, GX_TICKS_SECOND / 2);
			break;

		case GX_EVENT_TIMER:
			if (event_ptr->gx_event_payload.gx_event_timer_id == CLOCK_TIMER)
			{
				weight_update();
			}
			break;

		default:
			//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			break;
	}
	return gx_window_event_process(window, event_ptr);
}

/* Define my numeric format function. */
VOID weight_format_func(GX_NUMERIC_PIXELMAP_PROMPT *prompt, INT value)
{
	/* If the value is "1234", the new format will be "123.4". */

	INT length;
	gx_utility_ltoa(value / 10,
									prompt->gx_numeric_pixelmap_prompt_buffer,
									GX_NUMERIC_PROMPT_BUFFER_SIZE);
	length = GX_STRLEN(prompt->gx_numeric_pixelmap_prompt_buffer);
	prompt->gx_numeric_pixelmap_prompt_buffer[length++] = '.';
	gx_utility_ltoa(value % 10,
									prompt->gx_numeric_pixelmap_prompt_buffer + length,
									GX_NUMERIC_PROMPT_BUFFER_SIZE - length);
}

UINT weight_prompt_event(GX_NUMERIC_PIXELMAP_PROMPT *widget, GX_EVENT *event_ptr)
{
	UINT status = GX_SUCCESS;

	switch(event_ptr->gx_event_type)
	{
		//case xyz:
			/* Insert custom event handling here */
			//break;

		default:
			/* Pass all other events to the default tree view event processing */
			status = gx_prompt_event_process((GX_PROMPT *) widget, event_ptr);
			break;
	}
	return status;
}

/*
 * VOID (*gx_display_driver_buffer_toggle)(struct GX_CANVAS_STRUCT *canvas, GX_RECTANGLE *dirty_area)
 * This is a pointer to a function to toggle between the working and visible frame buffers for
 * double-buffered memory systems. This function must first instruct the hardware to begin using
 * the new frame buffer, then copy the modified portion of the new visible buffer to the companion
 * buffer, to insure the two buffers stay in synch.
 */
static void stm32h7_32argb_buffer_toggle(GX_CANVAS *canvas, GX_RECTANGLE *dirty_area)
{
	ULONG offset;
	INT   copy_width;
	INT   copy_height;
	ULONG *get;
	ULONG *put;
	ULONG actual_events;

	/* FIXME - maybe make sure the event is cleared here ?  */

	/* swap the buffers */
	if (pend_buffer < 0)
	{
		/* Switch to other buffer */
		pend_buffer = 1 - front_buffer;

		/* Refresh the display */
		HAL_LTDC_ProgramLineEvent(&hltdc, 0);
		//HAL_DSI_Refresh(&hdsi);
	}

	/* Request that event flags 0 is set. If it is set it should be cleared. If the event
	flags are not set, this service suspends for a maximum of 200 timer-ticks. */
	UINT status = tx_event_flags_get(&cm7_event_group, 0x1, TX_AND_CLEAR, &actual_events, 200);

	/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
	if (status == TX_SUCCESS)
	{
		/* FIXME - issues with uninitialized offline buffer... copy whole thing for now */
		/* now refresh offline buffer and switch buffers in canvas  */

		copy_width = 720; //dirty_area->gx_rectangle_right - dirty_area->gx_rectangle_left + 1;
		copy_height = 576; //dirty_area->gx_rectangle_bottom - dirty_area->gx_rectangle_top + 1;

		/* Read the update area from the canvas */
		offset = dirty_area->gx_rectangle_top * canvas->gx_canvas_x_resolution;
		offset += dirty_area->gx_rectangle_left;
		get = canvas->gx_canvas_memory;
		//get += offset;

		/* Read the area to be updated from the LCD video memory and copy the updated data from the canvas */
		put = (ULONG *) Buffers[1 - front_buffer];
		offset = (canvas->gx_canvas_display_offset_y + dirty_area->gx_rectangle_top) * MAIN_DISPLAY_X_RESOLUTION;
		offset += canvas->gx_canvas_display_offset_x + dirty_area->gx_rectangle_left;
		//put += offset;

		// RM0388 - pp 780...  not sure about interrupt vs polling yet
    DMA2D->CR = 0x00000000UL; // | DMA2D_CR_TCIE;
    DMA2D->FGMAR = (uint32_t) get;
    DMA2D->OMAR = (uint32_t) put;
    DMA2D->FGOR = canvas->gx_canvas_x_resolution - copy_width;
    DMA2D->OOR = MAIN_DISPLAY_X_RESOLUTION - copy_width;
    DMA2D->FGPFCCR = LTDC_PIXEL_FORMAT_ARGB8888;
    DMA2D->OPFCCR = LTDC_PIXEL_FORMAT_ARGB8888;
    DMA2D->NLR = (uint32_t) (copy_width << 16) | (uint16_t) copy_height;
    DMA2D->CR |= DMA2D_CR_START;
    while (DMA2D->CR & DMA2D_CR_START) {}

		/* Assign canvas memory block. */
		status = gx_canvas_memory_define(canvas, (GX_COLOR *) Buffers[1 - front_buffer], (720*576*4));

		/* If status is GX_SUCCESS, the canvas memory pointer has been reassigned. */

	}
	else
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
}

UINT stm32h7_graphics_driver_setup_32argb(GX_DISPLAY *display)
{
	// Display hardware should have been setup already

	_gx_display_driver_32argb_setup(display, GX_NULL, stm32h7_32argb_buffer_toggle);

	return(GX_SUCCESS);
}

/* USER CODE END 1 */

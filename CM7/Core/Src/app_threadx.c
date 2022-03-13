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

#define COLUMNS                              88
#define LINES                                20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Buffer for FileX FX_MEDIA sector cache. this should be 32-Bytes
   aligned to avoid cache maintenance issues */
//ALIGN_32BYTES (uint32_t media_memory[512]);

/* Define ThreadX global data structures.  */
TX_THREAD            cm7_main_thread;
TX_THREAD            cm7_touch_thread;
TX_THREAD            cm7_lcd_thread;
TX_THREAD            cm7_usb_stick_thread;
TX_THREAD            cm7_camera_thread;
/* 
 * event flag 0 is from LCD refresh done
 * event flag 1 is from HSEM_1 when core M4 signals touch data is available
 */
TX_EVENT_FLAGS_GROUP cm7_event_group;

/* Define the GUIX resources. */

/* Define the root window pointer. */

GX_WINDOW_ROOT *root_window;

/* data comning from CM4 core */
__attribute__((section(".sram4.sharedData"))) volatile CM4_CM7_SharedDataTypeDef sharedData;
__attribute__((section(".sram2.camera"))) volatile uint16_t cameraBuffer[(800 * 96)];

char textBuffer[LINES*COLUMNS];
UINT lineEnd[LINES];
UINT curLine;
volatile UINT txCnt;
ULONG gCounter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm7_main_thread_entry(ULONG thread_input);
void tx_cm7_touch_thread_entry(ULONG thread_input);
void tx_cm7_lcd_thread_entry(ULONG thread_input);
void tx_cm7_usb_stick_thread_entry(ULONG thread_input);
void tx_cm7_camera_thread_entry(ULONG thread_input);
void Error_Handler(void);
static void stm32h7_32argb_buffer_toggle(GX_CANVAS *canvas, GX_RECTANGLE *dirty_area);
UINT stm32h7_graphics_driver_setup_32argb(GX_DISPLAY *display);

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

  /*Allocate memory for tx_cm7_main_thread_entry */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
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

  /*Allocate memory for tx_cm7_lcd_thread_entry */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
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

  /*Allocate memory for tx_cm7_touch_thread_entry */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the touch thread.  */
  ret = tx_thread_create(&cm7_touch_thread, "tx_cm7_touch_thread", tx_cm7_touch_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /*Allocate memory for tx_cm7_usb_stick_thread_entry */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the usb_stick thread.  */
  ret = tx_thread_create(&cm7_usb_stick_thread, "tx_cm7_usb_stick_thread", tx_cm7_usb_stick_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check touch thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /*Allocate memory for tx_cm7_camera_thread_entry */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the camera thread.  */
  ret = tx_thread_create(&cm7_camera_thread, "tx_cm7_camera_thread", tx_cm7_camera_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check touch thread creation */
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

#if 0
/* Table for touchscreen event information display on LCD : table indexed on enum @ref TS_TouchEventTypeDef information */
char * ts_event_string_tab[4] = { "None",
"Press down",
"Lift up",
"Contact"};

/* Table for touchscreen gesture Id information display on LCD : table indexed on enum @ref TS_GestureIdTypeDef information */
char * ts_gesture_id_string_tab[7] = { "No Gesture",
"Move Up",
"Move Right",
"Move Down",
"Move Left",
"Zoom In",
"Zoom Out"};

/*******************************************************************************
* Function Name  : ft6x06_gest_id
* Description    : Read Gesture ID
* Input          : Pointer to uint8_t
* Output         : Status of GEST_ID register
* Return         : Status [FT6X06_ERROR, FT6X06_OK]
*******************************************************************************/
int32_t  ft6x06_gest_id(ft6x06_ctx_t *ctx, uint8_t *value)
{
  return ft6x06_read_reg(ctx, FT6X06_GEST_ID_REG, (uint8_t *)value, 1);
}
#endif
/*
 * TODO -
 *
 * TS_State_t  TS_State = {0};
 * TS_MultiTouch_State_t  TS_MTState = {0};
 * uint32_t GestureId = GESTURE_ID_NO_GESTURE;
 * ts_status = BSP_TS_GetState(0, &TS_State);
 *   if(TS_State.TouchDetected)
 * ts_status = BSP_TS_Get_MultiTouchState(0, &TS_MTState);
 * ts_status = BSP_TS_GetGestureId(0, &GestureId);
 *
GX_EVENT_PEN_DOWN
- Description: This event is generated by touch screen and mouse input drivers to indicate user pen-down (or left mouse button click) event.
- Payload: gx_event_pointdata.gx_point_x = pen x position in pixels
           gx_event_pointdata.gx_point_y = pen y position in pixels
           gx_event_display_handle = handle of the target display

GX_EVENT_PEN_UP
- Description: This event is generated by touch screen and mouse input drivers to indicate user pen-up (or left mouse button released) event.
- Payload: gx_event_pointdata.gx_point_x = pen x position in pixels
           gx_event_pointdata.gx_point_y = pen y position in pixels
           gx_event_display_handle = handle of the target display

GX_EVENT_PEN_DRAG
- Description: This event is generated by mouse and touch input drivers to indicate the pen is being dragged across the screen, or the mouse is being moved while the left mouse button is pressed.
- Payload: gx_event_pointdata.gx_point_x = pen x position in pixels
           gx_event_pointdata.gx_point_y = pen y position in pixels
           gx_event_display_handle = handle of the target display

GX_EVENT_ZOOM_IN
- Description: This event is generated by multi-touch touch input drivers to indicate a zoom-in gesture has been input by the user.
- Payload: None

GX_EVENT_ZOOM_OUT
- Description: This event is generated by multi-touch touch input drivers to indicate a zoom-out gesture has been input by the user. ]
- Payload: None

 */

void tx_cm7_touch_thread_entry(ULONG thread_input)
{
	uint16_t curTouch[4];
	uint16_t prevTouch[4] = {0};

  /* Infinite Loop */
  for( ;; )
  {
		ULONG actual_events;
		/* Request that event flag 1 is set. If it is set it should be cleared. */
		UINT status = tx_event_flags_get(&cm7_event_group, 0x2, TX_AND_CLEAR, &actual_events, TX_WAIT_FOREVER);

		/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
		if (status == TX_SUCCESS)
		{
  		sharedData.touchData2[0] = sharedData.touchData[0];
  		sharedData.touchData2[1] = sharedData.touchData[1];
  		sharedData.touchData2[2] = sharedData.touchData[2];
  		sharedData.touchData2[3] = sharedData.touchData[3];
  		if (sharedData.touchData[3] != prevTouch[3])
  		{
  			/* Send a pen event for processing. */
  			GX_EVENT e = {0};
  			//e.gx_event_display_handle = LCD_FRAME_BUFFER;
    		curTouch[0] = sharedData.touchData[0];
    		curTouch[1] = sharedData.touchData[1];
    		curTouch[2] = sharedData.touchData[2];
    		curTouch[3] = sharedData.touchData[3];
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
		}
  }
}

void tx_cm7_main_thread_entry(ULONG thread_input)
{
  //UINT status;
  //CHAR read_buffer[32];
  //CHAR data[] = "This is ThreadX working on STM32 CM7";
	ULONG toggleTicks = tx_time_get();

	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND/5);
	printf("Starting CM7 Run on %s\n", _tx_version_id);
  /* Infinite Loop */
  for( ;; )
  {
  	ULONG ticks = tx_time_get();
  	if (ticks >= toggleTicks)
  	{
    	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  		toggleTicks = ticks + TX_TIMER_TICKS_PER_SECOND;
	  	gCounter += 1;
  	}
  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 5);
  }
}

void tx_cm7_lcd_thread_entry(ULONG thread_input)
{
  extern GX_STUDIO_DISPLAY_INFO H747_Camera_display_table[]; // meh...

  /* Initialize GUIX. */
  gx_system_initialize();

  /* Configure the main display. */
  H747_Camera_display_table[MAIN_DISPLAY].canvas_memory = (GX_COLOR *) Buffers[0]; // I think...
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

void tx_cm7_usb_stick_thread_entry(ULONG thread_input)
{
  /* Infinite Loop */
  for( ;; )
  {
		ULONG actual_events;
		/* Request that event flag 1 is set. If it is set it should be cleared. */
		UINT status = tx_event_flags_get(&cm7_event_group, 0x8, TX_AND_CLEAR, &actual_events, TX_WAIT_FOREVER);

		/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
		if (status == TX_SUCCESS)
		{
			if (sharedData.CM4_to_CM7_USB_info & USB_INFO_STICK_INSERTED)
			{
				printf("Select USB button\n");
				gx_widget_style_add((GX_WIDGET *)&main_window.main_window_usb_icon, GX_STYLE_DRAW_SELECTED);
				gx_widget_style_add((GX_WIDGET *)&main_window.main_window_eject_icon, GX_STYLE_DRAW_SELECTED);
			}
		}
  }
}

void tx_cm7_camera_thread_entry(ULONG thread_input)
{
	ULONG prevUpdate = 0;
	UINT cnt = 0;
  /* Infinite Loop */
  for( ;; )
  {
		ULONG actual_events;
		/* Request that event flag 1 is set. If it is set it should be cleared. */
		UINT status = tx_event_flags_get(&cm7_event_group, 0x4, TX_AND_CLEAR, &actual_events, TX_WAIT_FOREVER);

		/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
		if (status == TX_SUCCESS)
		{
			ULONG ticks = tx_time_get();
			cnt += 1;
			if (prevUpdate + TX_TIMER_TICKS_PER_SECOND <= ticks)
			{
				gx_numeric_prompt_value_set(&main_window.main_window_fps_value, cnt);
				cnt = 0;
				prevUpdate = ticks;
			}
		}
  }
}

/*************************************************************************************/
VOID status_update()
{
	ULONG ticks = tx_time_get();
	printf("WS %5lu %5u",ticks / TX_TIMER_TICKS_PER_SECOND,txCnt);
	for (unsigned int i = 0; i < 8; i++)
	{
		uint16_t v = cameraBuffer[i];
		printf(" %02u%02u%02u",v >> 11, (v >> 5) & 0x3f, v & 0x1f);
	}
	printf("\n");
}

/*************************************************************************************/
static void shiftLines(void)
{
	/* we need to remove the first line by shifting everything up */
	if (curLine < 1)
		return;
	curLine -= 1;
	UINT shift = lineEnd[0];
	memmove(textBuffer, textBuffer + shift, lineEnd[curLine] - shift);
	for (unsigned int i = 0; i < curLine; i++)
		lineEnd[i] = lineEnd[i + 1] - shift;
}

int _write(int file, char *ptr, int len)
{
	if (len <= 0)
		return len;
	int cur = 0;
	while (cur < len)
	{
		UINT pos = lineEnd[curLine];
		UINT ll = pos;
		if (curLine > 0)
			ll -= lineEnd[curLine - 1];
		if (ll + 1 == COLUMNS || ptr[cur] == '\n')
		{
			/* we have or need a new line */
			textBuffer[pos] = '\n';
			if (ptr[cur] == '\n')
				cur += 1;
			lineEnd[curLine] += 1;
			curLine += 1;
			if (curLine > (LINES - 1))
				shiftLines();
			lineEnd[curLine] = lineEnd[curLine - 1];
		}
		else
		{
			textBuffer[pos] = ptr[cur];
			cur += 1;
			lineEnd[curLine] += 1;
		}
	}
	/* add terminating 0 to the string; we should have room */
	textBuffer[lineEnd[curLine]] = 0;
	gx_multi_line_text_view_text_set(&main_window.main_window_text_view, textBuffer);
	return len;
}

char *gEventName[] = {
	"NO_EVENT",
	"TERMINATE",
	"REDRAW",
	"SHOW",
	"HIDE",
	"RESIZED",
	"SLIDE",
	"FOCUS_GAINED",
	"FOCUS_LOST",
	"HORIZONTAL_SCROLL",
	"VERTICAL_SCROLL",
	"TIMER",
	"PEN_DOWN",
	"PEN_UP",
	"PEN_MOVE",
	"PEN_DRAG",
	"KEY_DOWN",
	"KEY_UP",
	"CLOSE",
	"DELETE",
	"SLIDER_VALUE",
	"TOGGLE_ON",
	"TOGGLE_OFF",
	"RADIO_SELECT",
	"RADIO_DESELECT",
	"CLICKED",
	"LIST_SELECT",
	"VERTICAL_FLICK",
	"HORIZONTAL_FLICK" };
char *gWidgetName[] = {
	"NONE",
	"TEXT_VIEW",
	"FPS_ICON",
	"FPS_VALUE",
	"USB_ICON",
	"USB_FILL_BAR",
	"FRAMES_ICON",
	"FRAMES_VALUE",
	"RECORD_ICON",
	"EJECT_ICON"
	};

/*************************************************************************************/
UINT main_screen_event_handler(GX_WINDOW *window, GX_EVENT *event_ptr)
{
	switch (event_ptr->gx_event_type)
	{
		case GX_EVENT_SHOW:
			/* Set current weight. */
			//status_update();

			/* Start a timer to update weight. */
			gx_system_timer_start(&main_window, CLOCK_TIMER, 60 * GX_TICKS_SECOND, 60 * GX_TICKS_SECOND);
			break;

		case GX_EVENT_TIMER:
			if (event_ptr->gx_event_payload.gx_event_timer_id == CLOCK_TIMER)
			{
				status_update();
			}
			break;

		default:
			//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
			if ((event_ptr->gx_event_type & 0xff) == GX_EVENT_CLICKED)
			{
				printf("-> We have a click in %s (%lu)\n", gWidgetName[event_ptr->gx_event_type >> 8], event_ptr->gx_event_type);
			}
			else
				printf("Got main screen event %lu %lu %s\n", event_ptr->gx_event_type, event_ptr->gx_event_type >> 8, gEventName[event_ptr->gx_event_type & 0xff]);
	}
	return gx_window_event_process(window, event_ptr);
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
	static int notfirst = 0;

	/* FIXME - maybe make sure the event is cleared here ?  */

	/* swap the buffers */
	if (pend_buffer < 0)
	{
		/* Switch to other buffer */
		pend_buffer = 1 - front_buffer;

		/* Configure the DMA2D Mode, Color Mode and output offset */
		hdma2d.Init.Mode          = DMA2D_M2M_PFC;
		hdma2d.Init.ColorMode     = DMA2D_OUTPUT_ARGB8888; /* Output color out of PFC */
		hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
		hdma2d.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

		/* Output offset in pixels == nb of pixels to be added at end of line to come to the  */
		/* first pixel of the next line : on the output side of the DMA2D computation         */
		hdma2d.Init.OutputOffset = 0;

		/* Foreground Configuration */
		hdma2d.LayerCfg[1].AlphaMode      = DMA2D_NO_MODIF_ALPHA;
		hdma2d.LayerCfg[1].InputAlpha     = 0xFF; /* fully opaque */
		hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
		hdma2d.LayerCfg[1].InputOffset    = 0;
		hdma2d.LayerCfg[1].RedBlueSwap    = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
		hdma2d.LayerCfg[1].AlphaInverted  = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

		hdma2d.Instance = DMA2D;

		/* DMA2D Initialization */
		if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
		{
			if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
			{
				if (HAL_DMA2D_Start(&hdma2d, (uint32_t)cameraBuffer, Buffers[pend_buffer], 800, 96) == HAL_OK)
				{
					/* Polling For DMA transfer */
					if(HAL_DMA2D_PollForTransfer(&hdma2d, 10) == HAL_OK)
					{
						/* return good status on exit */
						txCnt += 1;
					}
				}
			}
		}

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
		/* now refresh offline buffer and switch buffers in canvas  */
		if (!notfirst)
		{
			memcpy((void *)Buffers[1], (void *)Buffers[0], 800 * 480 * 4); /* maybe ? */
			notfirst = 1;
		}

		copy_width = dirty_area->gx_rectangle_right - dirty_area->gx_rectangle_left + 1;
		copy_height = dirty_area->gx_rectangle_bottom - dirty_area->gx_rectangle_top + 1;

		/* Read the update area from the canvas */
		offset = dirty_area->gx_rectangle_top * canvas->gx_canvas_x_resolution;
		offset += dirty_area->gx_rectangle_left;
		get = canvas->gx_canvas_memory;
		get += offset;

		/* Read the area to be updated from the LCD video memory and copy the updated data from the canvas */
		put = (ULONG *) Buffers[1 - front_buffer];
		offset = (canvas->gx_canvas_display_offset_y + dirty_area->gx_rectangle_top) * MAIN_DISPLAY_X_RESOLUTION;
		offset += canvas->gx_canvas_display_offset_x + dirty_area->gx_rectangle_left;
		put += offset;

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
		status = gx_canvas_memory_define(canvas, (GX_COLOR *) Buffers[1 - front_buffer], (800*480*4));

		/* If status is GX_SUCCESS, the canvas memory pointer has been reassigned. */

	}
	else
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
}

UINT stm32h7_graphics_driver_setup_32argb(GX_DISPLAY *display)
{
	// Display hardware should have been setup already

	_gx_display_driver_32argb_setup(display, GX_NULL, stm32h7_32argb_buffer_toggle);

	return(GX_SUCCESS);
}

// vim: noet ci pi sts=0 sw=2 ts=2
/* USER CODE END 1 */

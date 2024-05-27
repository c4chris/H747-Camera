
/**
  ******************************************************************************
  * @file    app_x-cube-ai.c
  * @author  X-CUBE-AI C code generator
  * @brief   AI program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

 /*
  * Description
  *   v1.0 - Minimum template to show how to use the Embedded Client API
  *          model. Only one input and one output is supported. All
  *          memory resources are allocated statically (AI_NETWORK_XX, defines
  *          are used).
  *          Re-target of the printf function is out-of-scope.
  *   v2.0 - add multiple IO and/or multiple heap support
  *
  *   For more information, see the embeded documentation:
  *
  *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
  *
  *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
  *   typical : C:\Users\<user_name>\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\7.1.0
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#if defined ( __ICCARM__ )
#elif defined ( __CC_ARM ) || ( __GNUC__ )
#endif

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"
#include "ai_datatypes_defines.h"
#include "beluga.h"
#include "beluga_data.h"

/* USER CODE BEGIN includes */
#include "app_threadx.h"
/* USER CODE END includes */

/* IO buffers ----------------------------------------------------------------*/

#if !defined(AI_BELUGA_INPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_in_1[AI_BELUGA_IN_1_SIZE_BYTES];
ai_i8* data_ins[AI_BELUGA_IN_NUM] = {
data_in_1
};
#else
ai_i8* data_ins[AI_BELUGA_IN_NUM] = {
NULL
};
#endif

#if !defined(AI_BELUGA_OUTPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_out_1[AI_BELUGA_OUT_1_SIZE_BYTES];
ai_i8* data_outs[AI_BELUGA_OUT_NUM] = {
data_out_1
};
#else
ai_i8* data_outs[AI_BELUGA_OUT_NUM] = {
NULL
};
#endif

/* Activations buffers -------------------------------------------------------*/

AI_ALIGNED(32)
static uint8_t pool0[AI_BELUGA_DATA_ACTIVATION_1_SIZE];

ai_handle data_activations0[] = {pool0};

/* AI objects ----------------------------------------------------------------*/

static ai_handle beluga = AI_HANDLE_NULL;

static ai_buffer* ai_input;
static ai_buffer* ai_output;

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
  if (fct)
    printf("TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
        err.type, err.code);
  else
    printf("TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type, err.code);

  do {} while (1);
  /* USER CODE END log */
}

static int ai_boostrap(ai_handle *act_addr)
{
  ai_error err;

  /* Create and initialize an instance of the model */
  err = ai_beluga_create_and_init(&beluga, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    ai_log_err(err, "ai_beluga_create_and_init");
    return -1;
  }

  ai_input = ai_beluga_inputs_get(beluga, NULL);
  ai_output = ai_beluga_outputs_get(beluga, NULL);

#if defined(AI_BELUGA_INPUTS_IN_ACTIVATIONS)
  /*  In the case where "--allocate-inputs" option is used, memory buffer can be
   *  used from the activations buffer. This is not mandatory.
   */
  for (int idx=0; idx < AI_BELUGA_IN_NUM; idx++) {
	data_ins[idx] = ai_input[idx].data;
  }
#else
  for (int idx=0; idx < AI_BELUGA_IN_NUM; idx++) {
	  ai_input[idx].data = data_ins[idx];
  }
#endif

#if defined(AI_BELUGA_OUTPUTS_IN_ACTIVATIONS)
  /*  In the case where "--allocate-outputs" option is used, memory buffer can be
   *  used from the activations buffer. This is no mandatory.
   */
  for (int idx=0; idx < AI_BELUGA_OUT_NUM; idx++) {
	data_outs[idx] = ai_output[idx].data;
  }
#else
  for (int idx=0; idx < AI_BELUGA_OUT_NUM; idx++) {
	ai_output[idx].data = data_outs[idx];
  }
#endif

  return 0;
}

static int ai_run(void)
{
  ai_i32 batch;

  batch = ai_beluga_run(beluga, ai_input, ai_output);
  if (batch != 1) {
    ai_log_err(ai_beluga_get_error(beluga),
        "ai_beluga_run");
    return -1;
  }

  return 0;
}

/* USER CODE BEGIN 2 */
#define NB_SUB_IMAGES 9
#define SUB_IMAGE_STEP 88

int acquire_and_process_data(ai_i8* data[], int subImg)
{
  /* fill the inputs of the c-model
  for (int idx=0; idx < AI_BELUGA_IN_NUM; idx++ )
  {
      data[idx] = ....
  }

  */
	volatile uint16_t *get = cameraBuffer + subImg * SUB_IMAGE_STEP;
	ULONG actual_events;
	UINT status = tx_mutex_get(&cm7_mutex_0, TX_WAIT_FOREVER);
	if (status != TX_SUCCESS)
	{
		Error_Handler();
	}
	/* Invalidate camera memory */
	SCB_InvalidateDCache_by_Addr((void *)data[0], 96*96*3);
	/* Configure the DMA2D Mode, Color Mode and output offset */
	hdma2d.Init.Mode          = DMA2D_M2M_PFC;
	hdma2d.Init.ColorMode     = DMA2D_OUTPUT_RGB888; /* Output color out of PFC */
	hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
	hdma2d.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

	/* Output offset in pixels == nb of pixels to be added at end of line to come to the  */
	/* first pixel of the next line : on the output side of the DMA2D computation         */
	hdma2d.Init.OutputOffset = 0;

	/* Foreground Configuration */
	hdma2d.LayerCfg[1].AlphaMode      = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha     = 0xFF; /* fully opaque */
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[1].InputOffset    = 800 - 96;
	hdma2d.LayerCfg[1].RedBlueSwap    = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
	hdma2d.LayerCfg[1].AlphaInverted  = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

	/* DMA2D Initialization */
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)get, (uint32_t)data[0], 96, 96) != HAL_OK)
	{
		Error_Handler();
	}
	status = tx_event_flags_get(&cm7_event_group, 0x20, TX_AND_CLEAR, &actual_events, TX_WAIT_FOREVER);
	status = tx_mutex_put(&cm7_mutex_0);
	if (status != TX_SUCCESS)
	{
		Error_Handler();
	}
  return 0;
}

int post_process(ai_i8* data[], int subImg)
{
  /* process the predictions
  for (int idx=0; idx < AI_BELUGA_OUT_NUM; idx++ )
  {
      data[idx] = ....
  }

  */
	float *res = (float *)data[0];
	int color = GX_COLOR_ID_BTN_BORDER;
	if (res[0] >= 0.7) color = GX_COLOR_ID_SELECTED_TEXT; // white
	if (res[1] >= 0.7) color = GX_COLOR_ID_BTN_LOWER; // green
	if (res[2] >= 0.7) color = GX_COLOR_ID_BTN_UPPER; // red-ish
	//printf("Results %d : %f %f %f\n", subImg, res[0], res[1], res[2]);
	btnColor[subImg] = color;
  return 0;
}
/* USER CODE END 2 */

/* Entry points --------------------------------------------------------------*/

void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
  printf("\r\nTEMPLATE - initialization\r\n");

  ai_boostrap(data_activations0);
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
  int res = -1;

  printf("TEMPLATE - run - main loop\r\n");

  if (beluga) {

    do {
    	ULONG actual_events;
    	/* Request that event flag 2 is set (camera data available). If it is set it should be cleared. */
  		res = 0;
    	UINT status = tx_event_flags_get(&cm7_event_group, 0x4, TX_AND_CLEAR, &actual_events, TX_TIMER_TICKS_PER_SECOND/4);
    	if (status == TX_SUCCESS)
    	{
    		for (int j = 0; (res == 0) && (j < NB_SUB_IMAGES); j++)
    		{
    			/* 1 - acquire and pre-process input data */
    			res = acquire_and_process_data(data_ins, j);
    			/* 2 - process the data - call inference engine */
    			if (res == 0)
    				res = ai_run();
    			/* 3- post-process the predictions */
    			if (res == 0)
    				res = post_process(data_outs, j);
    		}
    	}
    	/* Signal we are done with the camera data */
    	HAL_HSEM_FastTake(HSEM_ID_6);
    	HAL_HSEM_Release(HSEM_ID_6, 0);
    	if (tx_event_flags_set(&cm7_event_group, 0x16, TX_OR) != TX_SUCCESS)
    	{
    		res = 1;
    	}
    	tx_thread_sleep(5);
    } while (res==0);
  }

  if (res) {
    ai_error err = {AI_ERROR_INVALID_STATE, AI_ERROR_CODE_NETWORK};
    ai_log_err(err, "Process has FAILED");
  }
    /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif

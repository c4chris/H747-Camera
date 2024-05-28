
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
const uint8_t lut[] = {
/* 0 -1.0 */ 0,
/* 1 -1.0 */ 1,
/* 2 -1.0 */ 2,
/* 3 -1.0 */ 4,
/* 4 -1.0 */ 6,
/* 5 -1.0 */ 7,
/* 6 -1.0 */ 9,
/* 7 -0.9 */ 11,
/* 8 -0.9 */ 12,
/* 9 -0.9 */ 14,
/* 10 -0.9 */ 15,
/* 11 -0.9 */ 17,
/* 12 -0.9 */ 19,
/* 13 -0.9 */ 20,
/* 14 -0.9 */ 22,
/* 15 -0.9 */ 24,
/* 16 -0.9 */ 25,
/* 17 -0.9 */ 27,
/* 18 -0.9 */ 29,
/* 19 -0.9 */ 30,
/* 20 -0.8 */ 32,
/* 21 -0.8 */ 33,
/* 22 -0.8 */ 35,
/* 23 -0.8 */ 37,
/* 24 -0.8 */ 38,
/* 25 -0.8 */ 40,
/* 26 -0.8 */ 42,
/* 27 -0.8 */ 43,
/* 28 -0.8 */ 45,
/* 29 -0.8 */ 46,
/* 30 -0.8 */ 48,
/* 31 -0.8 */ 50,
/* 32 -0.7 */ 51,
/* 33 -0.7 */ 53,
/* 34 -0.7 */ 55,
/* 35 -0.7 */ 56,
/* 36 -0.7 */ 58,
/* 37 -0.7 */ 60,
/* 38 -0.7 */ 61,
/* 39 -0.7 */ 63,
/* 40 -0.7 */ 64,
/* 41 -0.7 */ 66,
/* 42 -0.7 */ 68,
/* 43 -0.7 */ 69,
/* 44 -0.7 */ 71,
/* 45 -0.6 */ 73,
/* 46 -0.6 */ 74,
/* 47 -0.6 */ 76,
/* 48 -0.6 */ 78,
/* 49 -0.6 */ 79,
/* 50 -0.6 */ 81,
/* 51 -0.6 */ 82,
/* 52 -0.6 */ 84,
/* 53 -0.6 */ 86,
/* 54 -0.6 */ 87,
/* 55 -0.6 */ 89,
/* 56 -0.6 */ 91,
/* 57 -0.6 */ 92,
/* 58 -0.5 */ 94,
/* 59 -0.5 */ 96,
/* 60 -0.5 */ 97,
/* 61 -0.5 */ 99,
/* 62 -0.5 */ 100,
/* 63 -0.5 */ 102,
/* 64 -0.5 */ 104,
/* 65 -0.5 */ 105,
/* 66 -0.5 */ 107,
/* 67 -0.5 */ 109,
/* 68 -0.5 */ 110,
/* 69 -0.5 */ 112,
/* 70 -0.5 */ 114,
/* 71 -0.4 */ 115,
/* 72 -0.4 */ 117,
/* 73 -0.4 */ 118,
/* 74 -0.4 */ 120,
/* 75 -0.4 */ 122,
/* 76 -0.4 */ 123,
/* 77 -0.4 */ 125,
/* 78 -0.4 */ 127,
/* 79 -0.4 */ 128,
/* 80 -0.4 */ 130,
/* 81 -0.4 */ 131,
/* 82 -0.4 */ 133,
/* 83 -0.3 */ 135,
/* 84 -0.3 */ 136,
/* 85 -0.3 */ 138,
/* 86 -0.3 */ 140,
/* 87 -0.3 */ 141,
/* 88 -0.3 */ 143,
/* 89 -0.3 */ 145,
/* 90 -0.3 */ 146,
/* 91 -0.3 */ 148,
/* 92 -0.3 */ 149,
/* 93 -0.3 */ 151,
/* 94 -0.3 */ 153,
/* 95 -0.3 */ 154,
/* 96 -0.2 */ 156,
/* 97 -0.2 */ 158,
/* 98 -0.2 */ 159,
/* 99 -0.2 */ 161,
/* 100 -0.2 */ 163,
/* 101 -0.2 */ 164,
/* 102 -0.2 */ 166,
/* 103 -0.2 */ 167,
/* 104 -0.2 */ 169,
/* 105 -0.2 */ 171,
/* 106 -0.2 */ 172,
/* 107 -0.2 */ 174,
/* 108 -0.2 */ 176,
/* 109 -0.1 */ 177,
/* 110 -0.1 */ 179,
/* 111 -0.1 */ 181,
/* 112 -0.1 */ 182,
/* 113 -0.1 */ 184,
/* 114 -0.1 */ 185,
/* 115 -0.1 */ 187,
/* 116 -0.1 */ 189,
/* 117 -0.1 */ 190,
/* 118 -0.1 */ 192,
/* 119 -0.1 */ 194,
/* 120 -0.1 */ 195,
/* 121 -0.1 */ 197,
/* 122 -0.0 */ 199,
/* 123 -0.0 */ 200,
/* 124 -0.0 */ 202,
/* 125 -0.0 */ 203,
/* 126 -0.0 */ 205,
/* 127 -0.0 */ 207,
/* 128 0.0 */ 208,
/* 129 0.0 */ 210,
/* 130 0.0 */ 212,
/* 131 0.0 */ 213,
/* 132 0.0 */ 215,
/* 133 0.0 */ 216,
/* 134 0.1 */ 218,
/* 135 0.1 */ 220,
/* 136 0.1 */ 221,
/* 137 0.1 */ 223,
/* 138 0.1 */ 225,
/* 139 0.1 */ 226,
/* 140 0.1 */ 228,
/* 141 0.1 */ 230,
/* 142 0.1 */ 231,
/* 143 0.1 */ 233,
/* 144 0.1 */ 234,
/* 145 0.1 */ 236,
/* 146 0.1 */ 238,
/* 147 0.2 */ 239,
/* 148 0.2 */ 241,
/* 149 0.2 */ 243,
/* 150 0.2 */ 244,
/* 151 0.2 */ 246,
/* 152 0.2 */ 248,
/* 153 0.2 */ 249,
/* 154 0.2 */ 251,
/* 155 0.2 */ 252,
/* 156 0.2 */ 254,
/* 157 0.2 */ 255,
/* 158 0.2 */ 255,
/* 159 0.2 */ 255,
/* 160 0.3 */ 255,
/* 161 0.3 */ 255,
/* 162 0.3 */ 255,
/* 163 0.3 */ 255,
/* 164 0.3 */ 255,
/* 165 0.3 */ 255,
/* 166 0.3 */ 255,
/* 167 0.3 */ 255,
/* 168 0.3 */ 255,
/* 169 0.3 */ 255,
/* 170 0.3 */ 255,
/* 171 0.3 */ 255,
/* 172 0.3 */ 255,
/* 173 0.4 */ 255,
/* 174 0.4 */ 255,
/* 175 0.4 */ 255,
/* 176 0.4 */ 255,
/* 177 0.4 */ 255,
/* 178 0.4 */ 255,
/* 179 0.4 */ 255,
/* 180 0.4 */ 255,
/* 181 0.4 */ 255,
/* 182 0.4 */ 255,
/* 183 0.4 */ 255,
/* 184 0.4 */ 255,
/* 185 0.5 */ 255,
/* 186 0.5 */ 255,
/* 187 0.5 */ 255,
/* 188 0.5 */ 255,
/* 189 0.5 */ 255,
/* 190 0.5 */ 255,
/* 191 0.5 */ 255,
/* 192 0.5 */ 255,
/* 193 0.5 */ 255,
/* 194 0.5 */ 255,
/* 195 0.5 */ 255,
/* 196 0.5 */ 255,
/* 197 0.5 */ 255,
/* 198 0.6 */ 255,
/* 199 0.6 */ 255,
/* 200 0.6 */ 255,
/* 201 0.6 */ 255,
/* 202 0.6 */ 255,
/* 203 0.6 */ 255,
/* 204 0.6 */ 255,
/* 205 0.6 */ 255,
/* 206 0.6 */ 255,
/* 207 0.6 */ 255,
/* 208 0.6 */ 255,
/* 209 0.6 */ 255,
/* 210 0.6 */ 255,
/* 211 0.7 */ 255,
/* 212 0.7 */ 255,
/* 213 0.7 */ 255,
/* 214 0.7 */ 255,
/* 215 0.7 */ 255,
/* 216 0.7 */ 255,
/* 217 0.7 */ 255,
/* 218 0.7 */ 255,
/* 219 0.7 */ 255,
/* 220 0.7 */ 255,
/* 221 0.7 */ 255,
/* 222 0.7 */ 255,
/* 223 0.7 */ 255,
/* 224 0.8 */ 255,
/* 225 0.8 */ 255,
/* 226 0.8 */ 255,
/* 227 0.8 */ 255,
/* 228 0.8 */ 255,
/* 229 0.8 */ 255,
/* 230 0.8 */ 255,
/* 231 0.8 */ 255,
/* 232 0.8 */ 255,
/* 233 0.8 */ 255,
/* 234 0.8 */ 255,
/* 235 0.8 */ 255,
/* 236 0.9 */ 255,
/* 237 0.9 */ 255,
/* 238 0.9 */ 255,
/* 239 0.9 */ 255,
/* 240 0.9 */ 255,
/* 241 0.9 */ 255,
/* 242 0.9 */ 255,
/* 243 0.9 */ 255,
/* 244 0.9 */ 255,
/* 245 0.9 */ 255,
/* 246 0.9 */ 255,
/* 247 0.9 */ 255,
/* 248 0.9 */ 255,
/* 249 1.0 */ 255,
/* 250 1.0 */ 255,
/* 251 1.0 */ 255,
/* 252 1.0 */ 255,
/* 253 1.0 */ 255,
/* 254 1.0 */ 255,
/* 255 1.0 */ 255,
};

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
	SCB_InvalidateDCache_by_Addr((void *)(data[0]), 96*96*3);
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
	hdma2d.LayerCfg[1].RedBlueSwap    = DMA2D_RB_SWAP; /* Try SWAP */ /* No ForeGround Red/Blue swap */
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
	if (HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)get, (uint32_t)(data[0]), 96, 96) != HAL_OK)
	{
		Error_Handler();
	}
	status = tx_event_flags_get(&cm7_event_group, 0x20, TX_AND_CLEAR, &actual_events, TX_WAIT_FOREVER);
	status = tx_mutex_put(&cm7_mutex_0);
	if (status != TX_SUCCESS)
	{
		Error_Handler();
	}
	// python3 -m tensorflow.lite.tools.visualize model_quant.tflite visualized_model.html
	// perl -e 'for $i (0..255){$f=$i/127.5-1; $q = $f/0.004798154812306166 + 208; $q = 255 if $q > 255; printf "/* %u %.1f */ %u,\n", $i, $f, $q}' >lut.txt
	// mask = np.logical_and.reduce((im[...,0]>=96,im[...,1]>=88,im[...,2]>=96))
	// im[mask] = [136,112,136]
	// try to mimick my masking stuff
	uint8_t *ptr = (uint8_t *)(data[0]);
	for (uint32_t i = 0; i < 96*96; i++)
	{
	  if (ptr[0] >= 96 && ptr[1] >= 88 && ptr[2] >= 96)
	  {
	    ptr[0] = 136;
	    ptr[1] = 112;
	    ptr[2] = 136;
	  }
	  ptr[0] = lut[ptr[0]];
	  ptr[1] = lut[ptr[1]];
	  ptr[2] = lut[ptr[2]];
	  ptr += 3;
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

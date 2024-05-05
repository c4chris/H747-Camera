/**
  ******************************************************************************
  * @file    beluga_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Sun May  5 21:48:36 2024
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef BELUGA_DATA_PARAMS_H
#define BELUGA_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_BELUGA_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_beluga_data_weights_params[1]))
*/

#define AI_BELUGA_DATA_CONFIG               (NULL)


#define AI_BELUGA_DATA_ACTIVATIONS_SIZES \
  { 92352, }
#define AI_BELUGA_DATA_ACTIVATIONS_SIZE     (92352)
#define AI_BELUGA_DATA_ACTIVATIONS_COUNT    (1)
#define AI_BELUGA_DATA_ACTIVATION_1_SIZE    (92352)



#define AI_BELUGA_DATA_WEIGHTS_SIZES \
  { 219324, }
#define AI_BELUGA_DATA_WEIGHTS_SIZE         (219324)
#define AI_BELUGA_DATA_WEIGHTS_COUNT        (1)
#define AI_BELUGA_DATA_WEIGHT_1_SIZE        (219324)



#define AI_BELUGA_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_beluga_activations_table[1])

extern ai_handle g_beluga_activations_table[1 + 2];



#define AI_BELUGA_DATA_WEIGHTS_TABLE_GET() \
  (&g_beluga_weights_table[1])

extern ai_handle g_beluga_weights_table[1 + 2];


#endif    /* BELUGA_DATA_PARAMS_H */

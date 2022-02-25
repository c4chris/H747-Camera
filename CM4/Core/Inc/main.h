/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ov5640.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart1;
extern DCMI_HandleTypeDef hdcmi;
extern volatile uint16_t cameraBuffer[(800 * 96)];
extern volatile uint32_t threadInitDone;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FMC_D28_Pin GPIO_PIN_6
#define FMC_D28_GPIO_Port GPIOI
#define FMC_NBL3_Pin GPIO_PIN_5
#define FMC_NBL3_GPIO_Port GPIOI
#define FMC_NBL2_Pin GPIO_PIN_4
#define FMC_NBL2_GPIO_Port GPIOI
#define ULPI_D7_Pin GPIO_PIN_5
#define ULPI_D7_GPIO_Port GPIOB
#define FMC_D25_Pin GPIO_PIN_1
#define FMC_D25_GPIO_Port GPIOI
#define FMC_D24_Pin GPIO_PIN_0
#define FMC_D24_GPIO_Port GPIOI
#define FMC_D29_Pin GPIO_PIN_7
#define FMC_D29_GPIO_Port GPIOI
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define FMC_D26_Pin GPIO_PIN_2
#define FMC_D26_GPIO_Port GPIOI
#define FMC_D23_Pin GPIO_PIN_15
#define FMC_D23_GPIO_Port GPIOH
#define FMC_D22_Pin GPIO_PIN_14
#define FMC_D22_GPIO_Port GPIOH
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define FMC_D27_Pin GPIO_PIN_3
#define FMC_D27_GPIO_Port GPIOI
#define FMC_SDCAS_Pin GPIO_PIN_15
#define FMC_SDCAS_GPIO_Port GPIOG
#define TOUCH_INT_Pin GPIO_PIN_7
#define TOUCH_INT_GPIO_Port GPIOK
#define CAM_PWR_DWN_Pin GPIO_PIN_14
#define CAM_PWR_DWN_GPIO_Port GPIOJ
#define FMC_D2_Pin GPIO_PIN_0
#define FMC_D2_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA
#define FMC_D21_Pin GPIO_PIN_13
#define FMC_D21_GPIO_Port GPIOH
#define FMC_D30_Pin GPIO_PIN_9
#define FMC_D30_GPIO_Port GPIOI
#define FMC_D3_Pin GPIO_PIN_1
#define FMC_D3_GPIO_Port GPIOD
#define CEC_CK_MCO1_Pin GPIO_PIN_8
#define CEC_CK_MCO1_GPIO_Port GPIOA
#define FMC_D31_Pin GPIO_PIN_10
#define FMC_D31_GPIO_Port GPIOI
#define ULPI_DIR_Pin GPIO_PIN_11
#define ULPI_DIR_GPIO_Port GPIOI
#define FMC_SDCLK_Pin GPIO_PIN_8
#define FMC_SDCLK_GPIO_Port GPIOG
#define FMC_A2_Pin GPIO_PIN_2
#define FMC_A2_GPIO_Port GPIOF
#define FMC_A1_Pin GPIO_PIN_1
#define FMC_A1_GPIO_Port GPIOF
#define FMC_A0_Pin GPIO_PIN_0
#define FMC_A0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOI
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOI
#define FMC_A3_Pin GPIO_PIN_3
#define FMC_A3_GPIO_Port GPIOF
#define FMC_BA0_Pin GPIO_PIN_4
#define FMC_BA0_GPIO_Port GPIOG
#define FMC_A12_Pin GPIO_PIN_2
#define FMC_A12_GPIO_Port GPIOG
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define FMC_A5_Pin GPIO_PIN_5
#define FMC_A5_GPIO_Port GPIOF
#define FMC_A4_Pin GPIO_PIN_4
#define FMC_A4_GPIO_Port GPIOF
#define ULPI_STP_Pin GPIO_PIN_0
#define ULPI_STP_GPIO_Port GPIOC
#define FMC_D7_Pin GPIO_PIN_10
#define FMC_D7_GPIO_Port GPIOE
#define ULPI_NXT_Pin GPIO_PIN_4
#define ULPI_NXT_GPIO_Port GPIOH
#define FMC_SDNWE_Pin GPIO_PIN_5
#define FMC_SDNWE_GPIO_Port GPIOH
#define FMC_A7_Pin GPIO_PIN_13
#define FMC_A7_GPIO_Port GPIOF
#define FMC_A8_Pin GPIO_PIN_14
#define FMC_A8_GPIO_Port GPIOF
#define FMC_D6_Pin GPIO_PIN_9
#define FMC_D6_GPIO_Port GPIOE
#define FMC_D8_Pin GPIO_PIN_11
#define FMC_D8_GPIO_Port GPIOE
#define ULPI_D3_Pin GPIO_PIN_10
#define ULPI_D3_GPIO_Port GPIOB
#define ULPI_D4_Pin GPIO_PIN_11
#define ULPI_D4_GPIO_Port GPIOB
#define FMC_D18_Pin GPIO_PIN_10
#define FMC_D18_GPIO_Port GPIOH
#define FMC_D19_Pin GPIO_PIN_11
#define FMC_D19_GPIO_Port GPIOH
#define FMC_D1_Pin GPIO_PIN_15
#define FMC_D1_GPIO_Port GPIOD
#define FMC_D0_Pin GPIO_PIN_14
#define FMC_D0_GPIO_Port GPIOD
#define FMC_A6_Pin GPIO_PIN_12
#define FMC_A6_GPIO_Port GPIOF
#define FMC_A9_Pin GPIO_PIN_15
#define FMC_A9_GPIO_Port GPIOF
#define FMC_D9_Pin GPIO_PIN_12
#define FMC_D9_GPIO_Port GPIOE
#define FMC_D12_Pin GPIO_PIN_15
#define FMC_D12_GPIO_Port GPIOE
#define FMC_D17_Pin GPIO_PIN_9
#define FMC_D17_GPIO_Port GPIOH
#define FMC_D20_Pin GPIO_PIN_12
#define FMC_D20_GPIO_Port GPIOH
#define I2C4_SCL_Pin GPIO_PIN_12
#define I2C4_SCL_GPIO_Port GPIOD
#define I2C4_SDA_Pin GPIO_PIN_13
#define I2C4_SDA_GPIO_Port GPIOD
#define ULPI_CK_Pin GPIO_PIN_5
#define ULPI_CK_GPIO_Port GPIOA
#define ULPI_D2_Pin GPIO_PIN_1
#define ULPI_D2_GPIO_Port GPIOB
#define FMC_SDRAS_Pin GPIO_PIN_11
#define FMC_SDRAS_GPIO_Port GPIOF
#define FMC_A10_Pin GPIO_PIN_0
#define FMC_A10_GPIO_Port GPIOG
#define FMC_D5_Pin GPIO_PIN_8
#define FMC_D5_GPIO_Port GPIOE
#define FMC_D10_Pin GPIO_PIN_13
#define FMC_D10_GPIO_Port GPIOE
#define FMC_SDNE1_Pin GPIO_PIN_6
#define FMC_SDNE1_GPIO_Port GPIOH
#define FMC_D16_Pin GPIO_PIN_8
#define FMC_D16_GPIO_Port GPIOH
#define ULPI_D5_Pin GPIO_PIN_12
#define ULPI_D5_GPIO_Port GPIOB
#define FMC_D15_Pin GPIO_PIN_10
#define FMC_D15_GPIO_Port GPIOD
#define FMC_D14_Pin GPIO_PIN_9
#define FMC_D14_GPIO_Port GPIOD
#define ULPI_D0_Pin GPIO_PIN_3
#define ULPI_D0_GPIO_Port GPIOA
#define ULPI_D1_Pin GPIO_PIN_0
#define ULPI_D1_GPIO_Port GPIOB
#define FMC_A11_Pin GPIO_PIN_1
#define FMC_A11_GPIO_Port GPIOG
#define FMC_D4_Pin GPIO_PIN_7
#define FMC_D4_GPIO_Port GPIOE
#define FMC_D11_Pin GPIO_PIN_14
#define FMC_D11_GPIO_Port GPIOE
#define FMC_SDCKE1_Pin GPIO_PIN_7
#define FMC_SDCKE1_GPIO_Port GPIOH
#define ULPI_D6_Pin GPIO_PIN_13
#define ULPI_D6_GPIO_Port GPIOB
#define FMC_D13_Pin GPIO_PIN_8
#define FMC_D13_GPIO_Port GPIOD
void   MX_FMC_Init(void);
/* USER CODE BEGIN Private defines */

#define HSEM_ID_0 (0U) /* HW semaphore 0 - used to coordinate boot with CM4 */
#define HSEM_ID_1 (1U) /* HW semaphore 1 - CM4 sends touchdata to CM7 */
#define HSEM_ID_2 (2U) /* HW semaphore 2 - CM4 signals camera data to CM7 */
#define HSEM_ID_3 (3U) /* HW semaphore 3 - CM4 signals USB stick status change to CM7 */
#define HSEM_ID_4 (4U) /* HW semaphore 4 - CM7 asks CM4 to perform some actions on USB stick */
#define HSEM_0 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0))
#define HSEM_1 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1))
#define HSEM_2 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_2))
#define HSEM_3 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_3))
#define HSEM_4 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_4))

#define ADV7533_MAIN_I2C_ADDR           0x7AU
#define ADV7533_CEC_DSI_I2C_ADDR        0x78U
#define ADV7533_MAIN_POWER_DOWN_REG     0x41U

#define I2Cx_TIMEOUT_MAX               0x3000 /* The value of the maximal timeout for I2C waiting loops */

#define CAMERA_OV5640_ADDRESS           0x78U

#define SDRAM_BANK_0         0xD0000000UL
#define SDRAM_BANK_1         0xD0800000UL
#define SDRAM_BANK_2         0xD1000000UL
#define SDRAM_BANK_3         0xD1800000UL

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

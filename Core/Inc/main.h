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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_14
#define LD4_GPIO_Port GPIOC
#define currentAC_Pin GPIO_PIN_0
#define currentAC_GPIO_Port GPIOA
#define voltageAC_Pin GPIO_PIN_1
#define voltageAC_GPIO_Port GPIOA
#define tempPCB_Pin GPIO_PIN_2
#define tempPCB_GPIO_Port GPIOA
#define voltageBUS_Pin GPIO_PIN_3
#define voltageBUS_GPIO_Port GPIOA
#define currentW_Pin GPIO_PIN_4
#define currentW_GPIO_Port GPIOA
#define currentV_Pin GPIO_PIN_6
#define currentV_GPIO_Port GPIOA
#define voltageINPUT_Pin GPIO_PIN_7
#define voltageINPUT_GPIO_Port GPIOA
#define currentU_Pin GPIO_PIN_0
#define currentU_GPIO_Port GPIOB
#define tempIPM_Pin GPIO_PIN_1
#define tempIPM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define FULLBRIDGE_PERIOD                   (uint32_t)2800
#define MAX_FULLBRIDGE_PERIOD               (uint32_t)(FULLBRIDGE_PERIOD * 90)/100  // 90%
#define MIN_FULLBRIDGE_PERIOD               (uint32_t)(FULLBRIDGE_PERIOD * 10)/100  // 10%

#define RISE_TIME                           (uint16_t)(FULLBRIDGE_PERIOD * 5)/100   // 5%
#define FALL_TIME                           (uint16_t)(FULLBRIDGE_PERIOD * 5)/100   // 5%

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

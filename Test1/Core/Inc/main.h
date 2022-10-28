/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motors.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DEBUG_STATE
#define FULL_CIRCLE (360u)
#define LIDAR_ON (1u)
#define LIDAR_OFF (0u)
#define CHANGED_MAN (1u)
#define NOT_CHANGED_MAN (0u)
#define ANG_DIST_ERR (0.5f)
#define MIN_DIST (160.0f)
#define AUTO_SPD (6u)
#define MAX_CNT (100u)
#define MIN_PRESSED_VAL (75)
#define MIN_UNPRESSED_VAL (-75)
#define AUTO_CMD ('p')
#define MAN_CMD ('P')
#define MV_FWD_CMD (70u)
#define MV_BWD_CMD (66u)
#define MV_LEFT_CMD (76u)
#define MV_RIGHT_CMD (82u)
#define MIN_SPEED_CMD (48u)
#define MAX_SPEED_CMD (57u)
#define STOP_CMD ('S')
#define LIGHT_ON_CMD ('w')
#define LIGHT_OFF_CMD ('W')
#define WAIT_TIME (100u)
#define BT_BUFF_SIZE (1u)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

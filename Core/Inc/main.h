/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSOR1_Pin GPIO_PIN_0
#define SENSOR1_GPIO_Port GPIOA
#define SENSOR1_EXTI_IRQn EXTI0_IRQn
#define SENSOR2_Pin GPIO_PIN_1
#define SENSOR2_GPIO_Port GPIOA
#define SENSOR2_EXTI_IRQn EXTI1_IRQn
#define SENSOR3_Pin GPIO_PIN_2
#define SENSOR3_GPIO_Port GPIOA
#define SENSOR3_EXTI_IRQn EXTI2_IRQn
#define SENSOR11_Pin GPIO_PIN_10
#define SENSOR11_GPIO_Port GPIOB
#define SENSOR11_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR13_Pin GPIO_PIN_12
#define SENSOR13_GPIO_Port GPIOB
#define SENSOR13_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR14_Pin GPIO_PIN_13
#define SENSOR14_GPIO_Port GPIOB
#define SENSOR14_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR12_Pin GPIO_PIN_11
#define SENSOR12_GPIO_Port GPIOA
#define SENSOR12_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR15_Pin GPIO_PIN_14
#define SENSOR15_GPIO_Port GPIOA
#define SENSOR15_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR16_Pin GPIO_PIN_15
#define SENSOR16_GPIO_Port GPIOA
#define SENSOR16_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR4_Pin GPIO_PIN_3
#define SENSOR4_GPIO_Port GPIOB
#define SENSOR4_EXTI_IRQn EXTI3_IRQn
#define SENSOR5_Pin GPIO_PIN_4
#define SENSOR5_GPIO_Port GPIOB
#define SENSOR5_EXTI_IRQn EXTI4_IRQn
#define SENSOR6_Pin GPIO_PIN_5
#define SENSOR6_GPIO_Port GPIOB
#define SENSOR6_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR7_Pin GPIO_PIN_6
#define SENSOR7_GPIO_Port GPIOB
#define SENSOR7_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR8_Pin GPIO_PIN_7
#define SENSOR8_GPIO_Port GPIOB
#define SENSOR8_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR9_Pin GPIO_PIN_8
#define SENSOR9_GPIO_Port GPIOB
#define SENSOR9_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR10_Pin GPIO_PIN_9
#define SENSOR10_GPIO_Port GPIOB
#define SENSOR10_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

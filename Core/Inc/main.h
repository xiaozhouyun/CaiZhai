/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define key2_Pin GPIO_PIN_2
#define key2_GPIO_Port GPIOC
#define key2_EXTI_IRQn EXTI2_IRQn
#define user_led_Pin GPIO_PIN_2
#define user_led_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_4
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOB
#define key3_Pin GPIO_PIN_1
#define key3_GPIO_Port GPIOE
#define key3_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/** 系统全局异常标志：0=正常，非0=异常。Tick Hook 检测到此标志后切换为 2Hz 可见闪烁 */
extern volatile uint8_t g_system_error;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

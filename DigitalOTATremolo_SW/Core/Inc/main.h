/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t sine1_toggle;
extern uint8_t sine2_toggle;
extern uint32_t tap_count;
extern uint8_t display_flash;
extern uint8_t sin1_max_arr;
extern uint8_t sin2_max_arr;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SIN1_TAPTEMPO_EXTI13_Pin GPIO_PIN_13
#define SIN1_TAPTEMPO_EXTI13_GPIO_Port GPIOC
#define SIN1_TAPTEMPO_EXTI13_EXTI_IRQn EXTI4_15_IRQn
#define SINE1_TOGGLE_EXTI1_Pin GPIO_PIN_1
#define SINE1_TOGGLE_EXTI1_GPIO_Port GPIOC
#define SINE1_TOGGLE_EXTI1_EXTI_IRQn EXTI0_1_IRQn
#define SINE2_TOGGLE_EXTI2_Pin GPIO_PIN_2
#define SINE2_TOGGLE_EXTI2_GPIO_Port GPIOC
#define SINE2_TOGGLE_EXTI2_EXTI_IRQn EXTI2_3_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

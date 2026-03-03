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
#include "stm32h7xx_hal.h"

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
#define EN_STEPPERS_Pin GPIO_PIN_2
#define EN_STEPPERS_GPIO_Port GPIOE
#define STEP_Z2_Pin GPIO_PIN_3
#define STEP_Z2_GPIO_Port GPIOE
#define DIR_Z2_Pin GPIO_PIN_4
#define DIR_Z2_GPIO_Port GPIOE
#define SPI4_CSS_X_Pin GPIO_PIN_13
#define SPI4_CSS_X_GPIO_Port GPIOC
#define SPI4_CSS_Y1_Pin GPIO_PIN_14
#define SPI4_CSS_Y1_GPIO_Port GPIOC
#define SPI4_CSS_Y2_Pin GPIO_PIN_15
#define SPI4_CSS_Y2_GPIO_Port GPIOC
#define SPI2_CSS_Pin GPIO_PIN_3
#define SPI2_CSS_GPIO_Port GPIOC
#define MY1_A_Pin GPIO_PIN_0
#define MY1_A_GPIO_Port GPIOA
#define MY1_B_Pin GPIO_PIN_1
#define MY1_B_GPIO_Port GPIOA
#define N_NTC_Pin GPIO_PIN_2
#define N_NTC_GPIO_Port GPIOA
#define H_NTC_Pin GPIO_PIN_3
#define H_NTC_GPIO_Port GPIOA
#define MY2_A_Pin GPIO_PIN_6
#define MY2_A_GPIO_Port GPIOA
#define MY2_B_Pin GPIO_PIN_7
#define MY2_B_GPIO_Port GPIOA
#define SPI3_CSS_Pin GPIO_PIN_1
#define SPI3_CSS_GPIO_Port GPIOB
#define DIR_F_Pin GPIO_PIN_7
#define DIR_F_GPIO_Port GPIOE
#define STEP_F_Pin GPIO_PIN_8
#define STEP_F_GPIO_Port GPIOE
#define MX_IN1_Pin GPIO_PIN_9
#define MX_IN1_GPIO_Port GPIOE
#define MX_IN2_Pin GPIO_PIN_11
#define MX_IN2_GPIO_Port GPIOE
#define MY1_IN1_Pin GPIO_PIN_13
#define MY1_IN1_GPIO_Port GPIOE
#define MY1_IN2_Pin GPIO_PIN_14
#define MY1_IN2_GPIO_Port GPIOE
#define DIR_Y_Pin GPIO_PIN_10
#define DIR_Y_GPIO_Port GPIOD
#define STEP_Y_Pin GPIO_PIN_11
#define STEP_Y_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_12
#define LED_2_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOD
#define H_PWM_Pin GPIO_PIN_14
#define H_PWM_GPIO_Port GPIOD
#define N_PWM_Pin GPIO_PIN_15
#define N_PWM_GPIO_Port GPIOD
#define ES_Z1_Pin GPIO_PIN_3
#define ES_Z1_GPIO_Port GPIOG
#define ES_Z2_Pin GPIO_PIN_4
#define ES_Z2_GPIO_Port GPIOG
#define ES_Y2_Pin GPIO_PIN_5
#define ES_Y2_GPIO_Port GPIOG
#define ES_Y1_Pin GPIO_PIN_6
#define ES_Y1_GPIO_Port GPIOG
#define ES_X_Pin GPIO_PIN_7
#define ES_X_GPIO_Port GPIOG
#define MY2_IN1_Pin GPIO_PIN_6
#define MY2_IN1_GPIO_Port GPIOC
#define MY2_IN2_Pin GPIO_PIN_7
#define MY2_IN2_GPIO_Port GPIOC
#define MX_A_Pin GPIO_PIN_15
#define MX_A_GPIO_Port GPIOA
#define STEP_P1_Pin GPIO_PIN_0
#define STEP_P1_GPIO_Port GPIOD
#define DIR_P1_Pin GPIO_PIN_1
#define DIR_P1_GPIO_Port GPIOD
#define STEP_P2_Pin GPIO_PIN_3
#define STEP_P2_GPIO_Port GPIOD
#define DIR_P2_Pin GPIO_PIN_4
#define DIR_P2_GPIO_Port GPIOD
#define SPI1_CSS_Pin GPIO_PIN_10
#define SPI1_CSS_GPIO_Port GPIOG
#define MX_B_Pin GPIO_PIN_3
#define MX_B_GPIO_Port GPIOB
#define STEP_Z1_Pin GPIO_PIN_0
#define STEP_Z1_GPIO_Port GPIOE
#define DIR_Z1_Pin GPIO_PIN_1
#define DIR_Z1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

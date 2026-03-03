/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_STEPPERS_Pin|STEP_Z2_Pin|DIR_Z2_Pin|DIR_F_Pin
                          |STEP_F_Pin|STEP_Z1_Pin|DIR_Z1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI4_CSS_X_Pin|SPI4_CSS_Y1_Pin|SPI4_CSS_Y2_Pin|SPI2_CSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CSS_GPIO_Port, SPI3_CSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR_Y_Pin|STEP_Y_Pin|LED_2_Pin|LED_1_Pin
                          |STEP_P1_Pin|DIR_P1_Pin|STEP_P2_Pin|DIR_P2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_STEPPERS_Pin STEP_Z2_Pin DIR_Z2_Pin DIR_F_Pin
                           STEP_F_Pin STEP_Z1_Pin DIR_Z1_Pin */
  GPIO_InitStruct.Pin = EN_STEPPERS_Pin|STEP_Z2_Pin|DIR_Z2_Pin|DIR_F_Pin
                          |STEP_F_Pin|STEP_Z1_Pin|DIR_Z1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI4_CSS_X_Pin SPI4_CSS_Y1_Pin SPI4_CSS_Y2_Pin SPI2_CSS_Pin */
  GPIO_InitStruct.Pin = SPI4_CSS_X_Pin|SPI4_CSS_Y1_Pin|SPI4_CSS_Y2_Pin|SPI2_CSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CSS_Pin */
  GPIO_InitStruct.Pin = SPI3_CSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Y_Pin STEP_Y_Pin LED_2_Pin LED_1_Pin
                           STEP_P1_Pin DIR_P1_Pin STEP_P2_Pin DIR_P2_Pin */
  GPIO_InitStruct.Pin = DIR_Y_Pin|STEP_Y_Pin|LED_2_Pin|LED_1_Pin
                          |STEP_P1_Pin|DIR_P1_Pin|STEP_P2_Pin|DIR_P2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ES_Z1_Pin ES_Z2_Pin ES_Y2_Pin ES_Y1_Pin
                           ES_X_Pin */
  GPIO_InitStruct.Pin = ES_Z1_Pin|ES_Z2_Pin|ES_Y2_Pin|ES_Y1_Pin
                          |ES_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CSS_Pin */
  GPIO_InitStruct.Pin = SPI1_CSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CSS_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

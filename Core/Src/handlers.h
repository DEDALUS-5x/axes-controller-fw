/*

  _   _                 _ _               
 | | | | __ _ _ __   __| | | ___ _ __ ___ 
 | |_| |/ _` | '_ \ / _` | |/ _ \ '__/ __|
 |  _  | (_| | | | | (_| | |  __/ |  \__ \
 |_| |_|\__,_|_| |_|\__,_|_|\___|_|  |___/
                                          

*/

#ifndef __HANDLERS_H__
#define __HANDLERS_H__

#include "main.h"
#include "types.h"
#include "ctrl.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

extern uint16_t spi1_rx_buf[3];
extern uint16_t spi4_single_buf[1];
extern Axis axis_X, axis_Y1, axis_Y2;
extern float current_values[3];
extern volatile uint8_t current_axis_idx;

extern Encoder enc_rot_X, enc_rot_Y1, enc_rot_Y2;
extern Encoder enc_lin_X, enc_lin_Y1, enc_lin_Y2;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

#endif
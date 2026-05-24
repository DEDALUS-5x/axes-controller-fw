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

#define TIM6_FREQ 10000.0

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

extern uint16_t spi1_rx_buf[1]; // Z1, A, C
extern uint16_t spi2_rx_buf[2]; // X, Y
extern uint16_t spi4_single_buf[1];
extern Axis axis_X, axis_Y;
extern Stepper axis_Z, axis_A, axis_C;
extern float current_values[3];
extern volatile uint8_t current_axis_idx;

extern Encoder enc_rot_X, enc_rot_Y, enc_rot_Z, enc_rot_A, enc_rot_C;
extern Encoder enc_lin_X, enc_lin_Y;

extern uint8_t machine_state;
/*
MACHINE STATE
- 0: init
- 1: homing
- 2: run
- 3: error
*/

extern uint8_t spi3_rx_buf[sizeof(SPIPacket)];
extern uint8_t spi3_tx_buf[sizeof(SPITxPacket)];

void update_rotary_encoder(Encoder *enc, uint16_t raw_spi, float dt);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

#endif
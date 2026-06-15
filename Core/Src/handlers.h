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
#include <string.h>

#define TIM6_FREQ 10000.0

#define INIT 0
#define HOMING 1
#define RUN 2
#define ERROR 3

/*
MACHINE STATE
- 0: INIT
- 1: HOMING
- 2: RUN
- 3: ERROR
*/
extern uint8_t machine_state;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;

extern SPI_HandleTypeDef hspi3;

extern uint16_t spi1_rx_buf[16];  // Y1, X, Z1
extern uint16_t spi1_tx_buf[16];
extern uint16_t spi2_rx_buf[16];  // A1, C, F
extern uint16_t spi2_tx_buf[16];
extern uint16_t spi4_single_buf[16];
extern Axis axis_X, axis_Y;
extern Stepper axis_Z, axis_A, axis_C;
extern float current_values[3];
extern volatile uint8_t current_axis_idx;

extern uint8_t spi3_rx_buf[sizeof(SPIPacket)];
extern uint8_t spi3_tx_buf_active[sizeof(SPIPacket)];
extern uint8_t spi3_tx_buf_staging[sizeof(SPITxPacket)];

extern Encoder enc_rot_X, enc_rot_Y, enc_rot_Z, enc_rot_A, enc_rot_C, enc_rot_F;
extern Encoder enc_lin_X, enc_lin_Y;


/**
 * 
 * @brief Function that takes the raw AS5048a data, directly from the SPI and convert it to the effective angular position of the stepper motor
 * @param enc Pointer to `Encoder` instance
 * @param raw_spi raw data coming from the SPI DMA buffer
 * @param dt time interval
 * 
 */
void update_rotary_encoder(Encoder *enc, uint16_t raw_spi, float dt);

/**
 * 
 * @brief Handler of the real-time timer. It is called as interrupt handler of the same timer and it runs the board operations at the timer frequency
 * @param htim Timer instance, in this case `TIM6`. It runs at 10kHz, so this function is called at the same frequency. PIDs and DMAs request are called at the same freq, but the linear encoders PID is called at 1kHz
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
 * 
 * @brief Handler of the SPI communication. It is called whenever a full-duplex communication is performed
 * @param hspi SPI instance. SPI1 and SPI2 are devoted to AS5048a acquisitino, while SPI3 is devoted to raspberry communication
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

#endif
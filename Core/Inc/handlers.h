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
#define TOL2MM 0.01f
#define IN_PERIOD 0.0001f // 10kHz Period
#define OUT_PERIOD 0.0005f // 1kHz Period
#define MAX_Y 150.0f;
#define MOT_SLEEP_TIMEOUT 20000 // 10 seconds

#define INIT 0
#define HOMING 1
#define RUN 2
#define ERROR 3
#define TUNING 4

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
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi6;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim23;

extern uint16_t spi1_rx_buf[16];  // y
extern uint16_t spi1_tx_buf[16];
extern uint16_t spi2_rx_buf[16];  // x
extern uint16_t spi2_tx_buf[16];
extern uint16_t spi4_rx_buf[16];  // a
extern uint16_t spi4_tx_buf[16];
extern uint16_t spi6_rx_buf[16];  // c
extern uint16_t spi6_tx_buf[16];

// extern uint16_t spi4_single_buf[16];
extern Axis axis_X, axis_Y;
extern Stepper axis_Z1, axis_Z2, axis_A1, axis_A2, axis_C, mot_flow;
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
 * @param raw_spi Raw data coming from the SPI DMA buffer
 * @param dt Time interval
 * 
 */
void update_rotary_encoder(Encoder *enc, uint16_t raw_spi, float dt);

/**
 * 
 * @brief Function that puts the stepper motors in sleep mode if no SPI message has been received for a certain amount of time
 * 
 */
void sleep_motors(void);

void update_pids(SPIPacket pkt, Axis *ax);

/**
 * 
 * @brief Handler of the real-time timer. It is called as interrupt handler of the same timer and it runs the board operations at the timer frequency
 * @param htim Timer instance, in this case `TIM6`. It runs at 10kHz, so this function is called at the same frequency. PIDs and DMAs request are called at the same freq, but the linear encoders PID is called at 1kHz
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
 * 
 * @brief Handler of the SPI communication. It is called whenever a full-duplex communication is performed
 * @param hspi SPI instance. SPI1 and SPI2, SPI4 and SPI6 are devoted to AS5048a acquisitino, while SPI3 is devoted to raspberry communication
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

#endif
/*

  _   _                 _ _               
 | | | | __ _ _ __   __| | | ___ _ __ ___ 
 | |_| |/ _` | '_ \ / _` | |/ _ \ '__/ __|
 |  _  | (_| | | | | (_| | |  __/ |  \__ \
 |_| |_|\__,_|_| |_|\__,_|_|\___|_|  |___/
                                          

*/

#include "handlers.h"
#include <math.h>

static uint8_t pid_counter = 0;

void update_rotary_encoder(Encoder *enc, uint16_t raw_spi, float dt){

  float new_pos = (float)(raw_spi & 0x3FFF) * (360.0f / 16384.0f);
    
    float diff = new_pos - enc->_converted_value;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    float instant_vel = diff / dt;
    enc -> _velocity = (enc -> _velocity * 0.7f) + (instant_vel * 0.3f);
    enc -> _converted_value = new_pos;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim -> Instance == TIM6) {
        const float dt = 0.0001f;

        // spi1 daisy chain + invalidare cahce
        SCB_InvalidateDCache_by_Addr((uint32_t*)spi1_rx_buf, sizeof(spi1_rx_buf));
        
        // DMA buffer: [0]=EncX, [1]=EncY1, [2]=EncY2
        update_rotary_encoder(&enc_rot_X, spi1_rx_buf[0], dt);
        update_rotary_encoder(&enc_lin_Y1, spi1_rx_buf[1], dt);

        if(++pid_counter >= 10){

          pid_counter = 0;
          const float dt_pos = 0.001f;

          enc_lin_X._converted_value = (float)((int32_t)TIM2 -> CNT) * 0.1;
          enc_lin_Y1._converted_value = (float)((int32_t)TIM3 -> CNT) * 0.1;

          axis_X._pid_vel._setpoint = PID_compute_pos(&axis_X._pid_pos, enc_lin_X._converted_value, dt_pos) + axis_X._target_vel;
          axis_Y1._pid_vel._setpoint = PID_compute_pos(&axis_Y1._pid_pos, enc_lin_Y1._converted_value, dt_pos) + axis_Y1._target_vel;

        }

        PID_compute_vel(&axis_X, dt);
        PID_compute_vel(&axis_Y1, dt);

        // spi4 read
        // current_axis_idx = 0;
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // CS x
        // HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);

        // PID_compute(&axis_X, dt);
        // PID_compute(&axis_Y1, dt);
        // PID_compute(&axis_Y2, dt);

        motor_command(&axis_X, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
        motor_command(&axis_Y1, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);
        // motor_command(&axis_Y2, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2);
    }
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

  // gestione raspi, ricordati di scomporre il feedrate su X e Y --> oppure fattelo dare dalla raspi già scomposto



  

  /*
  if (hspi->Instance == SPI4) {

    SCB_InvalidateDCache_by_Addr((uint32_t*)spi4_single_buf, sizeof(spi4_single_buf));
      
    current_values[current_axis_idx] = (float)spi4_single_buf[0] * (5.0f / 4096.0f);

    // CS high of the read one
    if (current_axis_idx == 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    else if (current_axis_idx == 1) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

    else if (current_axis_idx == 2) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

    current_values[current_axis_idx] = (float)spi4_single_buf[0] * (5.0f / 4096.0f);

    current_axis_idx++;

    // next CS
    if (current_axis_idx == 1) {

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // y1
      HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);
    } else if (current_axis_idx == 2) {

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // y2
      HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);
    }
  }

  */
}

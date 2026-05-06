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
static uint8_t led_counter = 0;
uint8_t spi_rx_buffer[sizeof(SPIPacket)] __attribute__((aligned(32)));

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
        
        // DMA buffer: [0]=EncX, [1]=EncY, [2]=EncY2
        update_rotary_encoder(&enc_rot_X, spi1_rx_buf[0], dt);
        update_rotary_encoder(&enc_lin_Y, spi1_rx_buf[1], dt);
        update_rotary_encoder(&enc_rot_Z, spi2_rx_buf[0], dt);
        update_rotary_encoder(&enc_rot_A, spi2_rx_buf[1], dt);
        update_rotary_encoder(&enc_rot_C, spi2_rx_buf[2], dt);

        stepper_loop(&axis_Z, &htim8, TIM_CHANNEL_1, STEP_Z1_GPIO_Port, STEP_Z1_Pin, 30.0f, 5.0f);
        stepper_loop(&axis_A, &htim8, TIM_CHANNEL_2, STEP_P1_GPIO_Port, STEP_P1_Pin, 10.0f, 2.0f);
        stepper_loop(&axis_C, &htim8, TIM_CHANNEL_3, STEP_Y_GPIO_Port, STEP_Y_Pin, 10.0f, 2.0f);

        if(++pid_counter >= 10){

          pid_counter = 0;
          const float dt_pos = 0.001f;

          enc_lin_X._converted_value = (float)((int32_t)TIM2 -> CNT) * 0.1;
          enc_lin_Y._converted_value = (float)((int32_t)TIM3 -> CNT) * 0.1;

          axis_X._pid_vel._setpoint = PID_compute_pos(&axis_X._pid_pos, enc_lin_X._converted_value, dt_pos) + axis_X._target_vel;
          axis_Y._pid_vel._setpoint = PID_compute_pos(&axis_Y._pid_pos, enc_lin_Y._converted_value, dt_pos) + axis_Y._target_vel;

        }

        if(++led_counter >= 10000){
          led_counter = 0;
          HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        }

        PID_compute_vel(&axis_X, dt);
        PID_compute_vel(&axis_Y, dt);

        motor_command(&axis_X, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
        motor_command(&axis_Y, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);

        // steppers

    }
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI4) {
    SPIPacket *packet = (SPIPacket *)spi_rx_buffer;

    // check on cubemx, it should be a circular buffer

    SCB_InvalidateDCache_by_Addr((uint32_t *)spi_rx_buffer, sizeof(spi_rx_buffer));

    if (packet->start == 0xAA) {
        
      axis_X._target_pos = packet -> x;
      axis_Y._target_pos = packet -> y;
      axis_Z._target = packet -> z;
      axis_A._target = packet -> a;
      axis_C._target = packet -> c;
      axis_X._target_vel = packet -> vx;
      axis_Y._target_vel = packet -> vy;

    }

    // 4. Gestione Cache (Cruciale su STM32H7)
    // Poiché il DMA scrive in RAM e la CPU legge, dobbiamo invalidare la cache
    // per forzare la CPU a leggere il dato fresco dalla RAM e non dalla cache L1
    // SCB_InvalidateDCache_by_Addr((uint32_t *)spi_rx_buffer, sizeof(SpiPacket_t));
  }
}


/*
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

  // gestione raspi, ricordati di scomporre il feedrate su X e Y --> oppure fattelo dare dalla raspi già scomposto

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

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // y
      HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);
    } else if (current_axis_idx == 2) {

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // y2
      HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);
    }
  }

  
}*/

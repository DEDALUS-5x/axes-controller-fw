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
static uint32_t led_counter = 0;
static uint8_t homing_counter = 0;


void update_rotary_encoder(Encoder *enc, uint16_t raw_spi, float dt){

  float new_pos = (float)(raw_spi & 0x3FFF) * (360.0f / 16384.0f);
    
  float diff = new_pos - enc->_last_raw_pos;

  if (diff > 180.0f)  enc->_turns--;
  else if (diff < -180.0f) enc->_turns++;
  enc->_last_raw_pos = new_pos;

  float total_pos_deg = (enc->_turns * 360.0f) + new_pos;
  enc->_converted_value = total_pos_deg - enc->_offset;

  float instant_vel = (enc->_converted_value - enc->_last_converted_value) / dt;
  enc->_velocity = (enc->_velocity * 0.8f) + (instant_vel * 0.2f);
  enc->_last_converted_value = enc->_converted_value;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim -> Instance == TIM6) {

        /*
          _  ___  _  ___         
         / |/ _ \| |/ / |__  ____
         | | | | | ' /| '_ \|_  /
         | | |_| | . \| | | |/ / 
         |_|\___/|_|\_\_| |_/___|
                                 
        */

        const float dt = 0.0001f;

        // spi1-2 daisy chain + invalidare cahce
        HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)spi1_tx_buf, (uint8_t*)spi1_rx_buf, 1);
        SCB_InvalidateDCache_by_Addr((uint32_t*)spi2_rx_buf, sizeof(spi2_rx_buf));
        
        // DMA buffer: [0]=EncX, [1]=EncY, [2]=EncY2
        update_rotary_encoder(&enc_rot_X, spi2_rx_buf[0], dt);
        update_rotary_encoder(&enc_rot_Y, spi2_rx_buf[1], dt);
        // update_rotary_encoder(&enc_rot_Z, spi1_rx_buf[0], dt);
        // update_rotary_encoder(&enc_rot_A, spi1_rx_buf[1], dt);
        // update_rotary_encoder(&enc_rot_C, spi1_rx_buf[2], dt);

        if (machine_state == 1 || machine_state == 2) {
            stepper_loop(&axis_Z, &htim4, TIM_CHANNEL_4, DIR_Z1_GPIO_Port, DIR_Z1_Pin, 30.0f, 5.0f);
            stepper_loop_soft(&axis_A, DIR_P1_GPIO_Port, DIR_P1_Pin, 10.0f, 2.0f);
            stepper_loop_soft(&axis_C, DIR_Y_GPIO_Port, DIR_Y_Pin, 10.0f, 2.0f);
            // stepper_loop(&axis_A, &htim8, TIM_CHANNEL_2, DIR_P1_GPIO_Port, DIR_P1_Pin, 10.0f, 2.0f);
            // stepper_loop(&axis_C, &htim8, TIM_CHANNEL_3, DIR_Y_GPIO_Port, DIR_Y_Pin, 10.0f, 2.0f);
        } else {
            axis_Z._target = enc_rot_Z._converted_value;
            axis_A._target = enc_rot_A._converted_value;
            axis_C._target = enc_rot_C._converted_value;
            
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
            axis_A._current_speed_hz = 0.0f;
            axis_C._current_speed_hz = 0.0f;
        }

        // pulses for steppers
        axis_A._accumulator += axis_A._current_speed_hz;
        if (axis_A._accumulator >= TIM6_FREQ) {
            HAL_GPIO_WritePin(STEP_P1_GPIO_Port, STEP_P1_Pin, GPIO_PIN_SET);
            axis_A._accumulator -= TIM6_FREQ; 
        } else {
            HAL_GPIO_WritePin(STEP_P1_GPIO_Port, STEP_P1_Pin, GPIO_PIN_RESET);
        }

        axis_C._accumulator += axis_C._current_speed_hz;
        if (axis_C._accumulator >= TIM6_FREQ) {
            HAL_GPIO_WritePin(STEP_Y_GPIO_Port, STEP_Y_Pin, GPIO_PIN_SET);
            axis_C._accumulator -= TIM6_FREQ;
        } else {
            HAL_GPIO_WritePin(STEP_Y_GPIO_Port, STEP_Y_Pin, GPIO_PIN_RESET);
        }

        if(++pid_counter >= 10){

          /*
            _ _  ___         
           / | |/ / |__  ____
           | | ' /| '_ \|_  /
           | | . \| | | |/ / 
           |_|_|\_\_| |_/___|
                             
          */

          pid_counter = 0;
          const float dt_pos = 0.001f;

          // int16_t in order to avoid overflow in case of negative values
          enc_lin_X._converted_value = (float)((int16_t)TIM2 -> CNT) * 0.1;
          enc_lin_Y._converted_value = (float)((int16_t)TIM3 -> CNT) * 0.1;

          axis_X._pid_vel._setpoint = PID_compute_pos(&axis_X._pid_pos, enc_lin_X._converted_value, dt_pos) + axis_X._target_vel;
          axis_Y._pid_vel._setpoint = PID_compute_pos(&axis_Y._pid_pos, enc_lin_Y._converted_value, dt_pos) + axis_Y._target_vel;

          // Feedback packet to send to the raspi
          static uint32_t current_msg_id = 0;
          SPITxPacket *tx_packet = (SPITxPacket *)spi3_tx_buf;
          
          tx_packet -> start = 0xBB;
          tx_packet -> msg_id = current_msg_id++;
          tx_packet -> x = enc_lin_X._converted_value;
          tx_packet -> y = enc_lin_Y._converted_value;
          tx_packet -> z = enc_rot_Z._converted_value;

          float ex = axis_X._target_pos - tx_packet -> x;
          float ey = axis_Y._target_pos - tx_packet -> y;
          float ez = axis_Z._target - tx_packet -> z;
          tx_packet -> error = sqrtf((ex * ex) + (ey * ey) + (ez * ez));

          SCB_CleanDCache_by_Addr((uint32_t *)spi3_tx_buf, sizeof(spi3_tx_buf));
        }

        if(++led_counter >= 10000){
          led_counter = 0;
          HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        }

        if(machine_state == 2){
          PID_compute_vel(&axis_X, dt);
          PID_compute_vel(&axis_Y, dt);
        }

        if(machine_state == 1 || machine_state == 2){

          motor_command(&axis_X, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
          motor_command(&axis_Y, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);
        } else{

          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        }
        

    }
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

  if (hspi -> Instance == SPI1){

    HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_SET);
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi1_rx_buf, sizeof(spi1_rx_buf));
    update_rotary_encoder(&enc_rot_Z, spi1_rx_buf[0], 0.0001f);
  }

  // triggered when a full duplex communication is provided: full RX message from raspi and TX to raspi
  else if (hspi -> Instance == SPI3) {
    SPIPacket *packet = (SPIPacket *)spi3_rx_buf; 

    // Invalida la cache per leggere i dati freschi della Raspi
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi3_rx_buf, sizeof(spi3_rx_buf));

    if (packet->start == 0xAA) {
      machine_state = 2;
        
      axis_X._target_pos = packet -> x;
      axis_Y._target_pos = packet -> y;
      axis_Z._target = packet -> z;
      axis_A._target = packet -> a;
      axis_C._target = packet -> c;
      axis_X._target_vel = packet -> vx;
      axis_Y._target_vel = packet -> vy;
    }
    // homing procedure
    if (packet -> start == 0xCC) {

      machine_state = 1;
      axis_X._pid_vel._output = -5.0f;
      axis_Y._pid_vel._output = -5.0f;
      axis_Z._target = 0;
      axis_A._target = 0;
      axis_C._target = 0;

      // motors command already embedded in tim6 handler. just keep a constnat pid output (pid disabled)
    }

  // 4. Gestione Cache (Cruciale su STM32H7)
  // Poiché il DMA scrive in RAM e la CPU legge, dobbiamo invalidare la cache
  // per forzare la CPU a leggere il dato fresco dalla RAM e non dalla cache L1
  // SCB_InvalidateDCache_by_Addr((uint32_t *)spi_rx_buffer, sizeof(SpiPacket_t));
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  if(machine_state == 1){

    if (GPIO_Pin == ES_X_Pin) {

      homing_counter++;

      // stop motors
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

      // offset rot encoder
      enc_rot_X._offset = (enc_rot_X._turns * 360.0f) + enc_rot_X._last_raw_pos;
      enc_rot_X._turns = 0;

      // timer linear encoder X
      TIM2 -> CNT = 0; 
      enc_lin_X._converted_value = 0.0f;
      enc_rot_X._converted_value = 0.0f;
      axis_X._pid_pos._setpoint = 0.0f;
      axis_X._pid_vel._setpoint = 0.0f;

      PID_reset(&(axis_X._pid_pos));
      PID_reset(&(axis_X._pid_vel));
    }

    if (GPIO_Pin == ES_Y1_Pin){

      homing_counter++;

      // stop motors
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

      // offset rot encoder
      enc_rot_Y._offset = (enc_rot_Y._turns * 360.0f) + enc_rot_Y._last_raw_pos;
      enc_rot_Y._turns = 0;

      TIM3 -> CNT = 0;
      enc_lin_Y._converted_value = 0.0f;
      axis_Y._pid_pos._setpoint = 0.0f;
      axis_Y._pid_vel._setpoint = 0.0f;

      PID_reset(&(axis_Y._pid_pos));
      PID_reset(&(axis_Y._pid_vel));
    }

    if (GPIO_Pin == ES_Z1_Pin){

      homing_counter++;

      enc_rot_Z._offset = (enc_rot_Z._turns * 360.0f) + enc_rot_Z._last_raw_pos;
      enc_rot_Z._turns = 0;
      enc_rot_Z._converted_value = 0.0f;

      axis_Z._target = 0.0f;
    }

    if(homing_counter == 3){
      machine_state = 2;
    }
  }

  if(machine_state == 2){

    // ERROR! physical violations, stop motors
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

    while(1); // required power cycle. The endstop interrupts have the higher priority
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

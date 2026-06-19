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

static volatile uint8_t spi1_need_clear = 0;
static volatile uint8_t spi2_need_clear = 0;

void update_rotary_encoder(Encoder *enc, uint16_t raw_spi, float dt){

  if (raw_spi & 0x4000) {
    return; 
  }

  float new_pos = (float)(raw_spi & 0x3FFF) * (360.0f / 16384.0f);
  float diff = new_pos - enc->_last_raw_pos;
  
  if (diff > 180.0f) {
    diff -= 360.0f;
    enc->_turns--;
  }
  else if (diff < -180.0f) {
    diff += 360.0f;
    enc->_turns++;
  }

  enc->_last_raw_pos = new_pos;

  float total_pos_deg = (enc->_turns * 360.0f) + new_pos;
  enc->_converted_value = (total_pos_deg - enc->_offset)  / enc -> g_ratio ;

  float instant_vel = diff / dt;
  
  enc -> _velocity = (enc->_velocity * 0.99f) + (instant_vel * 0.01f);
  enc -> _last_converted_value = enc->_converted_value;
  enc -> _acceleration = enc -> _acceleration * 0.95f + ((enc -> _velocity - enc -> _last_velocity) / dt) * 0.05f;
  enc -> _last_velocity = enc -> _velocity;
}



/*
  ____            _   _____ _                
 |  _ \ ___  __ _| | |_   _(_)_ __ ___   ___ 
 | |_) / _ \/ _` | |   | | | | '_ ` _ \ / _ \
 |  _ <  __/ (_| | |   | | | | | | | | |  __/
 |_| \_\___|\__,_|_|   |_| |_|_| |_| |_|\___|
                                          
*/

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

        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
        if (spi1_need_clear) {
            spi1_tx_buf[0] = 0x4001;
            spi1_need_clear = 0;
        } else {
            spi1_tx_buf[0] = 0xFFFF;
        }
        SCB_CleanDCache_by_Addr((uint32_t*)spi1_tx_buf, sizeof(spi1_tx_buf));
        HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_RESET);
        if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)spi1_tx_buf, (uint8_t*)spi1_rx_buf, 1) != HAL_OK) {
            HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_SET);
        }

        __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
        if (spi2_need_clear) {
            spi2_tx_buf[0] = 0x4001;
            spi2_need_clear = 0;
        } else {
            spi2_tx_buf[0] = 0xFFFF;
        }
        SCB_CleanDCache_by_Addr((uint32_t*)spi2_tx_buf, sizeof(spi2_tx_buf));
        HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_RESET);
        if (HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)spi2_tx_buf, (uint8_t*)spi2_rx_buf, 1) != HAL_OK) {
            HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_SET);
        }

        if (machine_state == HOMING || machine_state == RUN) {
            stepper_loop(&axis_Z, &htim4, TIM_CHANNEL_4, DIR_Z1_GPIO_Port, DIR_Z1_Pin, 20.0f, 5.0f);
            stepper_loop(&axis_A, &htim15, TIM_CHANNEL_1, DIR_P1_GPIO_Port, DIR_P1_Pin, 20.0, 5.0f);
            stepper_loop(&axis_C, &htim8, TIM_CHANNEL_2, DIR_Y_GPIO_Port, DIR_Y_Pin, 20.0, 5.0f);

        } else {
            axis_Z._target = enc_rot_Z._converted_value;
            
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
            axis_A._current_speed_hz = 0.0f;
            axis_C._current_speed_hz = 0.0f;
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

          // X
          enc_lin_X._converted_value = (float)((int32_t)TIM2 -> CNT) * 0.01f;
          float inst_vel_x = (enc_lin_X._converted_value - enc_lin_X._last_converted_value) / dt_pos;
          enc_lin_X._velocity = (enc_lin_X._velocity * 0.8f) + (inst_vel_x * 0.2f);
          float inst_acc_x = (enc_lin_X._velocity - enc_lin_X._last_velocity) / dt_pos;
          enc_lin_X._acceleration = (enc_lin_X._acceleration * 0.8f) + (inst_acc_x * 0.2f);

          // Y
          enc_lin_Y._converted_value = (float)((int32_t)TIM5 -> CNT) * 0.01f;        
          float inst_vel_y = (enc_lin_Y._converted_value - enc_lin_Y._last_converted_value) / dt_pos;
          enc_lin_Y._velocity = (enc_lin_Y._velocity * 0.8f) + (inst_vel_y * 0.2f);
          float inst_acc_y = (enc_lin_Y._velocity - enc_lin_Y._last_velocity) / dt_pos;
          enc_lin_Y._acceleration = (enc_lin_Y._acceleration * 0.8f) + (inst_acc_y * 0.2f);

          // update last readings
          enc_lin_X._last_converted_value = enc_lin_X._converted_value;
          enc_lin_X._last_velocity = enc_lin_X._velocity;
          enc_lin_Y._last_converted_value = enc_lin_Y._converted_value;
          enc_lin_Y._last_velocity = enc_lin_Y._velocity;

          // PID pos
          axis_X._pid_pos._setpoint = axis_X._target_pos;
          axis_X._pid_vel._setpoint = -PID_compute_pos(&axis_X._pid_pos, enc_lin_X._converted_value, dt_pos) + axis_X._target_vel;
          // axis_Y._pid_vel._setpoint = PID_compute_pos(&axis_Y._pid_pos, enc_lin_Y._converted_value, dt_pos) + axis_Y._target_vel;
          // axis_X._pid_vel._setpoint = axis_X._target_vel;
          // axis_Y._pid_vel._setpoint = axis_Y._target_vel;

          // Feedback packet to send to the raspi
          static uint32_t current_msg_id = 0;
          SPITxPacket tx_packet;
          
          // FEEDBACK SPI PACKET
          tx_packet.start = 0xBB;
          tx_packet.msg_id = current_msg_id++;
          tx_packet.x = enc_lin_X._converted_value;
          tx_packet.y = enc_lin_Y._converted_value;
          tx_packet.z = enc_rot_Z._converted_value;
          tx_packet.a = enc_rot_A._converted_value;
          tx_packet.c = enc_rot_C._converted_value;
          tx_packet.vx = enc_lin_X._velocity;
          tx_packet.vy = enc_lin_Y._velocity;
          tx_packet.vz = enc_rot_Z._velocity * 8.0f / 360.0f;
          tx_packet.va = enc_rot_A._velocity;
          tx_packet.vc = enc_rot_C._velocity;
          tx_packet.ax = enc_lin_X._acceleration;
          tx_packet.ay = enc_lin_Y._acceleration;
          tx_packet.az = enc_rot_Z._acceleration;
          tx_packet.aa = enc_rot_A._acceleration;
          tx_packet.ac = enc_rot_C._acceleration;

          float ex = axis_X._target_pos - tx_packet.x;
          float ey = axis_Y._target_pos - tx_packet.y;
          float ez = axis_Z._target - tx_packet.z;
          tx_packet.error = sqrtf((ex * ex) + (ey * ey) + (ez * ez));

          uint8_t calc_check = 0;
          uint8_t *ptr = (uint8_t*)&tx_packet;
          for(int i = 0; i < 69; i++){
              calc_check ^= ptr[i];
          }
          tx_packet.check = calc_check;

          // tx_packet as tmp buffer. atomize memcpy
          __disable_irq(); 
          memcpy(spi3_tx_buf_staging, &tx_packet, sizeof(SPITxPacket));
          __enable_irq();

        }

        if(++led_counter >= 10000){
          led_counter = 0;
          HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        }

        if(machine_state == RUN){
          PID_compute_vel(&axis_X, dt);
          PID_compute_vel(&axis_Y, dt);
        }

        if(machine_state == HOMING || machine_state == RUN){

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


/*
  ____  ____ ___ 
 / ___||  _ \_ _|
 \___ \| |_) | | 
  ___) |  __/| | 
 |____/|_|  |___|
                 
*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

  if (hspi -> Instance == SPI1){

    HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_SET);
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi1_rx_buf, sizeof(spi1_rx_buf));

    if (spi1_rx_buf[0] & 0x4000) {
        spi1_need_clear = 1;
    } else {
        update_rotary_encoder(&enc_rot_Y, spi1_rx_buf[0], 0.0001f);
    }
    
  }

  else if(hspi -> Instance == SPI2){

    HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_SET);
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi2_rx_buf, sizeof(spi2_rx_buf));

    if (spi2_rx_buf[0] & 0x4000) {
        spi2_need_clear = 1;
    } else {
        update_rotary_encoder(&enc_rot_X, spi2_rx_buf[0], 0.0001f);
    }
  }

  // triggered when a full duplex communication is provided: full RX message from raspi and TX to raspi
  else if (hspi -> Instance == SPI3) {

    static SPIPacket packet;
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi3_rx_buf, sizeof(spi3_rx_buf));
    memcpy(&packet, spi3_rx_buf, sizeof(SPIPacket));

    memcpy(spi3_tx_buf_active, spi3_tx_buf_staging, sizeof(SPITxPacket));
    SCB_CleanDCache_by_Addr((uint32_t *)spi3_tx_buf_active, sizeof(SPITxPacket));
    HAL_SPI_TransmitReceive_DMA(&hspi3, spi3_tx_buf_active, spi3_rx_buf, sizeof(SPIPacket));

    if (packet.start == 0xAA) {
      machine_state = RUN;
        
      axis_X._target_pos = packet.x;
      axis_Y._target_pos = packet.y;
      axis_Z._target = packet.z;
      axis_A._target = packet.a;
      axis_C._target = packet.c;
      axis_X._target_vel = packet.vx;
      axis_Y._target_vel = packet.vy;
      
    }
    // homing procedure
    if (packet.start == 0xCC) {

      HAL_GPIO_WritePin(EN_STEPPERS_GPIO_Port, EN_STEPPERS_Pin, RESET);

      machine_state = HOMING;
      axis_X._pid_vel._output = -1.0f;
      axis_Y._pid_vel._output = -1.0f;
      axis_Z._target = 0.0f;
      axis_A._target = 0.0f;
      axis_C._target = 0.0f;

      // motors command already embedded in tim6 handler. just keep a constnat pid output (pid disabled)
    }

  }
}

/*
  _____           _   ____  _                  
 | ____|_ __   __| | / ___|| |_ ___  _ __  ___ 
 |  _| | '_ \ / _` | \___ \| __/ _ \| '_ \/ __|
 | |___| | | | (_| |  ___) | || (_) | |_) \__ \
 |_____|_| |_|\__,_| |____/ \__\___/| .__/|___/
                                    |_|        
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  if(machine_state == HOMING){

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

      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

      enc_rot_Z._offset = (enc_rot_Z._turns * 360.0f) + enc_rot_Z._last_raw_pos;
      enc_rot_Z._turns = 0;
      enc_rot_Z._converted_value = 0.0f;

      axis_Z._target = 0.0f;
    }

    if(homing_counter == 3){
      machine_state = RUN;
    }
  }

  if(machine_state == RUN){

    // ERROR! physical violations, stop motors
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

    while(1); // required power cycle. The endstop interrupts have the higher priority
  }

}

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

static volatile uint8_t spi1_need_clear = 0;
static volatile uint8_t spi2_need_clear = 0;
static volatile uint8_t spi4_need_clear = 0;
static volatile uint8_t spi6_need_clear = 0;

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
  
  enc -> _velocity = (enc->_velocity * 0.9) + (instant_vel * 0.1f);
  enc -> _last_converted_value = enc->_converted_value;
  enc -> _acceleration = enc -> _acceleration * 0.95f + ((enc -> _velocity - enc -> _last_velocity) / dt) * 0.05f;
  enc -> _last_velocity = enc -> _velocity;
}

void update_pwm_encoder(Encoder *enc, TIM_HandleTypeDef *htim, float dt) {
    uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t duty   = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (period == 0) return; // no enc or no timer
    float duty_cycle = (float)duty / (float)period;
    float angle_steps = (duty_cycle * 4119.0f) - 16.0f;

    if (angle_steps < 0.0f) angle_steps = 0.0f; // error
    if (angle_steps > 4095.0f) angle_steps = 4095.0f; // error

    // extract angle from 0 to 360
    float new_pos = (angle_steps / 4095.0f) * 360.0f;
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
    enc->_converted_value = (total_pos_deg - enc->_offset);

    float instant_vel = diff / dt;
    enc->_velocity = (enc->_velocity * 0.99f) + (instant_vel * 0.01f);
    enc->_last_converted_value = enc->_converted_value;
    
    float inst_acc = (enc->_velocity - enc->_last_velocity) / dt;
    enc->_acceleration = enc->_acceleration * 0.95f + inst_acc * 0.05f;
    enc->_last_velocity = enc->_velocity;
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

        const float dt = IN_PERIOD;

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

        __HAL_SPI_CLEAR_OVRFLAG(&hspi4);
        if (spi4_need_clear) {
            spi4_tx_buf[0] = 0x4001;
            spi4_need_clear = 0;
        } else {
            spi4_tx_buf[0] = 0xFFFF;
        }
        SCB_CleanDCache_by_Addr((uint32_t*)spi4_tx_buf, sizeof(spi4_tx_buf));
        HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_RESET);
        if (HAL_SPI_TransmitReceive_DMA(&hspi4, (uint8_t*)spi4_tx_buf, (uint8_t*)spi4_rx_buf, 1) != HAL_OK) {
            HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_SET);
        }

        __HAL_SPI_CLEAR_OVRFLAG(&hspi6);
        if (spi6_need_clear) {
            spi6_tx_buf[0] = 0x4001;
            spi6_need_clear = 0;
        } else {
            spi6_tx_buf[0] = 0xFFFF;
        }
        SCB_CleanDCache_by_Addr((uint32_t*)spi6_tx_buf, sizeof(spi6_tx_buf));
        HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_RESET);
        if (HAL_SPI_TransmitReceive_DMA(&hspi6, (uint8_t*)spi6_tx_buf, (uint8_t*)spi6_rx_buf, 1) != HAL_OK) {
            HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_SET);
        }

        if (machine_state == HOMING || machine_state == RUN) {
              // stepper_loop(&axis_Z1, &htim17, TIM_CHANNEL_1, DIR_Z1_GPIO_Port, DIR_Z1_Pin, 20.0f, 2.0f, 0.01f, dt);
              stepper_loop(&axis_Z1, &htim17, TIM_CHANNEL_1, DIR_Z1_GPIO_Port, DIR_Z1_Pin, 3.0f, 1.0f, 0.01f, dt);
              stepper_command(axis_Z1._current_speed_hz, axis_Z2.steps_per_unit, &htim3, TIM_CHANNEL_2, DIR_Z2_GPIO_Port, DIR_Z2_Pin, 0); 
              enc_rot_Z._converted_value += (axis_Z1._current_speed_hz * dt);
              stepper_loop(&axis_A1, &htim15, TIM_CHANNEL_1, DIR_P1_GPIO_Port, DIR_P1_Pin, 10.0f, 10.0f, 0.01f, dt);
              stepper_loop(&axis_A2, &htim16, TIM_CHANNEL_1, DIR_P2_GPIO_Port, DIR_P2_Pin, 10.0f, 10.0f, 0.01f, dt);
              stepper_command(axis_A2._current_speed_hz, axis_A1.steps_per_unit, &htim15, TIM_CHANNEL_1, DIR_P1_GPIO_Port, DIR_P1_Pin, axis_A1._dir);
              stepper_loop(&axis_C, &htim8, TIM_CHANNEL_2, DIR_Y_GPIO_Port, DIR_Y_Pin, 0.15, 0.01f, 0.001f, dt);

              /*
              if(axis_A1._in_position == 0){
                stepper_loop(&axis_A1, &htim15, TIM_CHANNEL_1, DIR_P1_GPIO_Port, DIR_P1_Pin, 10.0f, 10.0f, 0.01f, IN_PERIOD);
              }
              
              if(axis_A2._in_position == 0){
                stepper_loop(&axis_A2, &htim16, TIM_CHANNEL_1, DIR_P2_GPIO_Port, DIR_P2_Pin, 10.0f, 10.0f, 0.01f, IN_PERIOD);
              }
              if(axis_C._in_position == 0){
                stepper_loop(&axis_C, &htim8, TIM_CHANNEL_2, DIR_Y_GPIO_Port, DIR_Y_Pin, 1.0f, 1.0f, 0.01f, IN_PERIOD);
              }
                */
          } else {
              axis_Z1._target = enc_rot_Z._converted_value;
              axis_Z2._target = enc_rot_Z._converted_value;
              __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
              axis_Z1._current_speed_hz = 0.0f;
              axis_Z2._current_speed_hz = 0.0f;

              __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
              axis_A1._current_speed_hz = 0.0f;
              __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
              axis_A2._current_speed_hz = 0.0f;

              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
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
          const float dt_pos = OUT_PERIOD;

          // X
          enc_lin_X._converted_value = - (float)((int32_t)TIM2 -> CNT) * 0.01f;
          float inst_vel_x = (enc_lin_X._converted_value - enc_lin_X._last_converted_value) / dt_pos;
          enc_lin_X._velocity = (enc_lin_X._velocity * 0.9f) + (inst_vel_x * 0.1f);
          float inst_acc_x = (enc_lin_X._velocity - enc_lin_X._last_velocity) / dt_pos;
          enc_lin_X._acceleration = (enc_lin_X._acceleration * 0.8f)+ (inst_acc_x * 0.2f);

          // Y
          enc_lin_Y._converted_value = (float)((int32_t)TIM5 -> CNT) * 0.01f;        
          float inst_vel_y = (enc_lin_Y._converted_value - enc_lin_Y._last_converted_value) / dt_pos;
          enc_lin_Y._velocity = (enc_lin_Y._velocity * 0.9f) + (inst_vel_y * 0.1f);
          float inst_acc_y = (enc_lin_Y._velocity - enc_lin_Y._last_velocity) / dt_pos;
          enc_lin_Y._acceleration = (enc_lin_Y._acceleration * 0.8f) + (inst_acc_y * 0.2f);

          // update last readings
          enc_lin_X._last_converted_value = enc_lin_X._converted_value;
          enc_lin_X._last_velocity = enc_lin_X._velocity;
          enc_lin_Y._last_converted_value = enc_lin_Y._converted_value;
          enc_lin_Y._last_velocity = enc_lin_Y._velocity;

          // PID pos
          axis_X._pid_pos._setpoint = axis_X._target_pos;
          axis_X._pid_vel._setpoint = PID_compute_pos(&axis_X._pid_pos, enc_lin_X._converted_value, dt_pos) + axis_X._target_vel;
          axis_Y._pid_pos._setpoint = axis_Y._target_pos;
          axis_Y._pid_vel._setpoint = PID_compute_pos(&axis_Y._pid_pos, enc_lin_Y._converted_value, dt_pos); // + axis_Y._target_vel;
          // axis_X._pid_vel._setpoint += axis_X._target_vel;
          // axis_Y._pid_vel._setpoint += axis_Y._target_vel;

          // positioning axes
          // update_pwm_encoder(&enc_rot_C, &htim23, dt_pos);
          // update_pwm_encoder(&enc_rot_A, &htim4, dt_pos);

          // enc_rot_A._converted_value += (axis_A1._current_speed_hz * dt_pos);
          // enc_rot_C._converted_value += (axis_C._current_speed_hz * dt_pos);

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
          tx_packet.vz = axis_Z1._current_speed_hz * 8.0f / 360.0f;
          tx_packet.va = enc_rot_A._velocity;
          tx_packet.vc = enc_rot_C._velocity;
          tx_packet.ax = enc_lin_X._acceleration;
          tx_packet.ay = enc_lin_Y._acceleration;
          tx_packet.az = 0.0f; // enc_rot_Z._acceleration;
          tx_packet.aa = enc_rot_A._acceleration;
          tx_packet.ac = enc_rot_C._acceleration;

          float ex = axis_X._target_pos - tx_packet.x;
          float ey = axis_Y._target_pos - tx_packet.y;
          float ez = axis_Z1._target - tx_packet.z;
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

        if(machine_state == HOMING || machine_state == RUN){

          // continuity
          axis_X._target_pos += axis_X._target_vel * dt;
          axis_Y._target_pos += axis_Y._target_vel * dt;
          PID_compute_vel(&axis_X, dt);
          PID_compute_vel(&axis_Y, dt);
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
        update_rotary_encoder(&enc_rot_Y, spi1_rx_buf[0], IN_PERIOD);
    }
    
  }

  else if(hspi -> Instance == SPI2){

    HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_SET);
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi2_rx_buf, sizeof(spi2_rx_buf));

    if (spi2_rx_buf[0] & 0x4000) {
        spi2_need_clear = 1;
    } else {
        update_rotary_encoder(&enc_rot_X, spi2_rx_buf[0], IN_PERIOD);
    }
  }

  else if(hspi -> Instance == SPI4){

    HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_SET);
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi4_rx_buf, sizeof(spi4_rx_buf));

    if (spi4_rx_buf[0] & 0x4000) {
        spi4_need_clear = 1;
    } else {
        update_rotary_encoder(&enc_rot_A, spi4_rx_buf[0], IN_PERIOD);
    }
  }

  else if(hspi -> Instance == SPI6){

    HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_SET);
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi6_rx_buf, sizeof(spi6_rx_buf));

    if (spi6_rx_buf[0] & 0x4000) {
        spi6_need_clear = 1;
    } else {
        update_rotary_encoder(&enc_rot_C, spi6_rx_buf[0], IN_PERIOD);
    }
  }

  // triggered when a full duplex communication is provided: full RX message from raspi and TX to raspi
  else if (hspi -> Instance == SPI3) {

    static SPIPacket packet;
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi3_rx_buf, sizeof(spi3_rx_buf));
    memcpy(&packet, spi3_rx_buf, sizeof(SPIPacket));

    // memcpy(spi3_tx_buf_active, spi3_tx_buf_staging, sizeof(SPITxPacket));
    // SCB_CleanDCache_by_Addr((uint32_t *)spi3_tx_buf_active, sizeof(SPITxPacket));
    // HAL_SPI_TransmitReceive_DMA(&hspi3, spi3_tx_buf_active, spi3_rx_buf, sizeof(SPIPacket));

    if (packet.start == 0xAA && machine_state == RUN) {
        
      axis_X._target_pos = (axis_X._target_pos * 0.3f) + (packet.x * 0.7f);
      axis_Y._target_pos = (axis_Y._target_pos * 0.5f) + (packet.y * 0.5f);
      axis_Z1._target = packet.z;
      axis_Z2._target = packet.z;
      axis_X._target_vel = packet.vx;
      axis_Y._target_vel = packet.vy;

      // steppers positioning management
      if(axis_A1._target != packet.a){
        axis_A1._in_position = 0;
        axis_A1._target = packet.a;
      }
      if(axis_A2._target != packet.a){
        axis_A2._in_position = 0;
        axis_A2._target = packet.a;
      }
      if(axis_C._target != packet.c){
        axis_C._in_position = 0;
        axis_C._target = packet.c;
      }
      
    }
    // homing procedure
    if (packet.start == 0xCC) {

      HAL_GPIO_WritePin(EN_STEPPERS_GPIO_Port, EN_STEPPERS_Pin, RESET);

      machine_state = HOMING;
      axis_X._pid_vel._output = -1.0f;
      axis_Y._pid_vel._output = -1.0f;
      axis_Z1._target = 0.0f;
      axis_Z2._target = 0.0f;
      axis_A1._target = 0.0f;
      axis_A2._target = 0.0f;
      axis_C._target = 0.0f;

      axis_A1._in_position = 0;
      axis_A2._in_position = 0;
      axis_C._in_position = 0;

      // motors command already embedded in tim6 handler. just keep a constnat pid output (pid disabled)
    }

    memcpy(spi3_tx_buf_active, spi3_tx_buf_staging, sizeof(SPITxPacket));
    SCB_CleanDCache_by_Addr((uint32_t *)spi3_tx_buf_active, sizeof(SPITxPacket));
    // clena errors and restart DMA
    __HAL_SPI_CLEAR_OVRFLAG(&hspi3); 
    __HAL_SPI_CLEAR_FREFLAG(&hspi3);
    HAL_SPI_TransmitReceive_DMA(&hspi3, spi3_tx_buf_active, spi3_rx_buf, sizeof(SPIPacket));

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

  // debouncing shit
  static uint32_t last_interrupt_time = 0;
  uint32_t current_time = HAL_GetTick();
  if (current_time - last_interrupt_time < 100) {
      return; 
  }
  last_interrupt_time = current_time;

  static uint8_t x_homed = 0;
  static uint8_t y_homed = 0;
  // static uint8_t z_homed = 0;

  if(machine_state == HOMING) {

    if (GPIO_Pin == ES_X_Pin && x_homed == 0) {
      
      x_homed = 1;
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

    if (GPIO_Pin == ES_Y1_Pin && y_homed == 0) {

      y_homed = 1;
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

    if (GPIO_Pin == ES_Z1_Pin) {
      // z_homed = 1;
      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);

      enc_rot_Z._offset = 0.0f;
      enc_rot_Z._turns = 0;
      enc_rot_Z._converted_value = 0.0f;

      axis_Z1._target = 0.0f;
      axis_Z2._target = 0.0f;
      axis_Z1._current_speed_hz = 0.0f;
      axis_Z2._current_speed_hz = 0.0f;
    }

    if(x_homed == 1 && y_homed == 1) {
      machine_state = RUN;
      
      // for the next homing
      x_homed = 0;
      y_homed = 0;
      // z_homed = 0;
    }
  
  } else if(machine_state == RUN) {

    // ERROR! physical violations, stop motors
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

    machine_state = INIT;
  }
}
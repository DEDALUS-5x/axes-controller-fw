/*

   ____            _             _ 
  / ___|___  _ __ | |_ _ __ ___ | |
 | |   / _ \| '_ \| __| '__/ _ \| |
 | |__| (_) | | | | |_| | | (_) | |
  \____\___/|_| |_|\__|_|  \___/|_|
                                   

*/

#include "ctrl.h"

void PID_reset(PID *pid){

  pid -> _integral = 0.0f;
  pid -> _last_error = 0.0f;
  pid -> _output = 0.0f;
  pid -> _setpoint = 0.0f;
}

void PID_init(PID *pid, float kp, float ki, float kd, float limit, float w_limit){

  pid -> _kp = kp;
  pid -> _ki = ki;
  pid -> _kd = kd;
  pid -> _output_limit = limit;
  pid -> _anti_windup_limit = w_limit;
  
  PID_reset(pid);
}

float PID_compute_pos(PID *pid, float current_pos, float dt){

  float error = pid -> _setpoint - current_pos;

  float P = pid -> _kp * error;

  // integrale solo se non siamo già saturi
  pid -> _integral += pid -> _ki * error * dt;
  if (pid->_anti_windup_limit > 0.0f) {
      if (pid->_integral > pid->_anti_windup_limit) pid->_integral = pid->_anti_windup_limit;
      if (pid->_integral < -pid->_anti_windup_limit) pid->_integral = -pid->_anti_windup_limit;
  } else {
      pid->_integral = 0.0f; 
  }

  float D = pid -> _kd * (error - pid -> _last_error) / dt;
  pid -> _last_error = error;

  float out = P + pid -> _integral + D;

  // saturazione velocità
  if (out > pid -> _output_limit) out = pid -> _output_limit;
  if (out < -pid -> _output_limit) out = -pid -> _output_limit;

  return out;
}

void PID_compute_vel(Axis *axis, float dt) {

  float current_linear_vel = axis -> _enc_rot -> _velocity * (DEG_TO_MM / axis -> _enc_rot -> g_ratio);

  float error = axis -> _pid_vel._setpoint - current_linear_vel;
  float accel = (axis -> _target_vel - axis -> _last_vel) / dt;
  axis -> _last_vel = axis -> _target_vel;

  float P = axis -> _pid_vel._kp * error;

  axis -> _pid_vel._integral += axis -> _pid_vel._ki * error * dt;
  if (axis -> _pid_vel._anti_windup_limit > 0.0f) {
      if (axis -> _pid_vel._integral > axis -> _pid_vel._anti_windup_limit) 
          axis -> _pid_vel._integral = axis -> _pid_vel._anti_windup_limit;
      if (axis -> _pid_vel._integral < -axis -> _pid_vel._anti_windup_limit) 
          axis -> _pid_vel._integral = -axis -> _pid_vel._anti_windup_limit;
  } else {
      axis -> _pid_vel._integral = 0.0f;
  }
  // derivative D and filterin N
  float raw_D = axis -> _pid_vel._kd * (error - axis -> _pid_vel._last_error) / dt;
  float filtered_D = (axis -> _pid_vel._last_D * 0.9f) + (raw_D * 0.1f); 
  axis -> _pid_vel._last_D = filtered_D;
  axis -> _pid_vel._last_error = error;

  float stiction_pwm = 0.0f;
  
  if (axis->_target_vel > 0.02f) {
      stiction_pwm = axis -> _kickstart;
  } else if (axis->_target_vel < -0.02f) {
      stiction_pwm = -(axis -> _kickstart);
  }

  float out = P + axis->_pid_vel._integral + filtered_D + (accel * axis->_ka) + stiction_pwm;

  if (out > axis -> _pid_vel._output_limit) out = axis -> _pid_vel._output_limit;
  if (out < -axis -> _pid_vel._output_limit) out = -axis -> _pid_vel._output_limit;

  axis -> _pid_vel._output = out;
}

void motor_command(Axis *axis, TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2) {
    if (axis -> _pid_vel._output > 0) {
        __HAL_TIM_SET_COMPARE(htim, channel1, (uint32_t)fabsf(axis -> _pid_vel._output));
        __HAL_TIM_SET_COMPARE(htim, channel2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(htim, channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, channel2, (uint32_t)fabsf(axis -> _pid_vel._output));
    }
}


/*

  ____  _                                 
 / ___|| |_ ___ _ __  _ __   ___ _ __ ___ 
 \___ \| __/ _ \ '_ \| '_ \ / _ \ '__/ __|
  ___) | ||  __/ |_) | |_) |  __/ |  \__ \
 |____/ \__\___| .__/| .__/ \___|_|  |___/
               |_|   |_|                  

*/

void stepper_command(float speed,  float steps_per_unit, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, uint8_t dir) {

    if(dir == 0) {
        HAL_GPIO_WritePin(dir_port, dir_pin, (speed <= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dir_port, dir_pin, (speed <= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    float freq = fabsf(speed) * steps_per_unit;
    if (freq < 1.6f) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0); 
        return;
    }


    uint32_t timer_base_clock;
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM15 || 
        htim->Instance == TIM16 || htim->Instance == TIM17) {
        timer_base_clock = HAL_RCC_GetPCLK2Freq() * 2; // APB2 x2 on H7
    } else {
        timer_base_clock = HAL_RCC_GetPCLK1Freq() * 2; // APB1 x2 on H7
    }

    uint32_t psc = htim->Instance->PSC + 1; // PSC è (valore - 1)

    uint32_t arr_val = (uint32_t)((float)timer_base_clock / (float)(psc * freq)) - 1;

    if (arr_val < 10) arr_val = 10; // keep low freqs
    if (arr_val > 0xFFFF && (htim->Instance != TIM2 && htim->Instance != TIM5)) {
        arr_val = 0xFFFF; // hard limit for 16bit timer
    }

    htim->Instance->CR1 &= ~TIM_CR1_ARPE;
    __HAL_TIM_SET_AUTORELOAD(htim, arr_val);
    __HAL_TIM_SET_COMPARE(htim, channel, arr_val / 2); // 50% duty cycle 

    // check anti glitch
    if (htim->Instance->CNT > arr_val) {
        htim->Instance->CNT = 0;
    }

    if (!(htim->Instance->CR1 & TIM_CR1_CEN)) {
        if (htim->Instance == TIM16) {
            HAL_TIMEx_PWMN_Start(htim, channel);
        } else {
            HAL_TIM_PWM_Start(htim, channel);
        }
    }
}


void stepper_loop(Stepper *stepper, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, float max_speed, float kp, float dt) {

    float raw_error = stepper->_target - stepper->_enc_rot->_converted_value;
    
    float alpha_err = 0.2f; 
    float error = (stepper->_last_error * (1.0f - alpha_err)) + (raw_error * alpha_err);
    stepper->_last_error = error;

    float step_deg = 1.0f / stepper->steps_per_unit; 
    float encoder_noise_floor = 0.1f;

    // noise on the encoder is about 0.05 deg, so we add it to the tolerance
    float inner_tol = (step_deg * 0.6f) + (encoder_noise_floor * 0.5f); 
    float outer_tol = (step_deg * 1.5f) + encoder_noise_floor; 

    if (stepper->_in_position == 1) {
        if (fabsf(error) > outer_tol) {
            stepper->_in_position = 0; 
        } else {
            stepper_command(0.0f, stepper->steps_per_unit, htim, channel, dir_port, dir_pin, stepper->_dir);
            return;
        }
    } else {
        if (fabsf(error) < inner_tol) {
            stepper_command(0.0f, stepper->steps_per_unit, htim, channel, dir_port, dir_pin, stepper->_dir);
            stepper->_current_speed_hz = 0.0f;
            stepper->_target_speed = 0.0f;
            stepper->_in_position = 1;
            return;
        }
    }

    float req_v = error * kp;
    float max_accel = 30.0f; 
    float safe_v = sqrtf(2.0f * max_accel * fabsf(error));

    if (req_v > safe_v) req_v = safe_v;
    if (req_v < -safe_v) req_v = -safe_v;
    if (req_v > max_speed) req_v = max_speed;
    if (req_v < -max_speed) req_v = -max_speed;
    
    float max_vel = max_accel * dt;
    if (req_v > stepper->_target_speed + max_vel) {
        stepper->_target_speed += max_vel;
    } else if (req_v < stepper->_target_speed - max_vel) {
        stepper->_target_speed -= max_vel;
    } else {
        stepper->_target_speed = req_v;
    }

    float min_hz = 1.7f;
    float current_hz = fabsf(stepper->_target_speed) * stepper->steps_per_unit;
    
    if (current_hz > 0.0f && current_hz < min_hz) {
        if (error > 0.0f) {
            stepper->_target_speed = min_hz / stepper->steps_per_unit;
        } else {
            stepper->_target_speed = -min_hz / stepper->steps_per_unit;
        }
    }

    stepper->_current_speed_hz = stepper->_target_speed;

    stepper_command(stepper->_target_speed, stepper->steps_per_unit, htim, channel, dir_port, dir_pin, stepper->_dir);
}

void heat_command(GPIO_TypeDef *drive_port, uint16_t drive_pin, GPIO_TypeDef *ntc_port, uint16_t ntc_pin, float temp){

    TempState *current_state;
    uint32_t  *current_timer;
    
    if (drive_pin == H_PWM_Pin) {
        current_state = &bed_state;
        current_timer = &bed_timer;
    } else if (drive_pin == H_PWM_Pin) {
        current_state = &ext_state;
        current_timer = &ext_timer;
    } else {
        return;
    }
    
    if (temp < 10.0f || machine_state == 0 || machine_state == ERROR) {
        HAL_GPIO_WritePin(drive_port, drive_pin, GPIO_PIN_RESET);
        *current_state = STATE_OFF;
        return;
    }
    uint8_t is_cold = (HAL_GPIO_ReadPin(ntc_port, ntc_pin) == GPIO_PIN_SET);

    switch (*current_state) {
        
        case STATE_OFF: // start to heat when cold
            if (is_cold) {
                *current_state = STATE_HEATING;
                *current_timer = HAL_GetTick(); 
                HAL_GPIO_WritePin(drive_port, drive_pin, GPIO_PIN_SET);
            }
            break;

        case STATE_HEATING: // heating and no more cold

            if (!is_cold) {
                HAL_GPIO_WritePin(drive_port, drive_pin, GPIO_PIN_RESET);
                *current_state = STATE_COOLDOWN; 
                *current_timer = HAL_GetTick();      
            } 
            else { // heating, still cold
                HAL_GPIO_WritePin(drive_port, drive_pin, GPIO_PIN_SET); 
                if ((HAL_GetTick() - *current_timer) > WATCHDOG_TIME_MS) {
                    HAL_GPIO_WritePin(drive_port, drive_pin, GPIO_PIN_RESET);
                    machine_state = ERROR;
                }
            }
            break;

        case STATE_COOLDOWN:
            HAL_GPIO_WritePin(drive_port, drive_pin, GPIO_PIN_RESET); 
            if ((HAL_GetTick() - *current_timer) > COOLDOWN_TIME_MS) {
                if (is_cold) {
                    *current_state = STATE_HEATING;
                    *current_timer = HAL_GetTick();
                }
            }
            break;
    }
}
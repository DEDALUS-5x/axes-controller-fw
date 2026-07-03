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

void PID_init(PID *pid, float kp, float ki, float kd, float limit){

  pid -> _kp = kp;
  pid -> _ki = ki;
  pid -> _kd = kd;
  pid -> _output_limit = limit;
  
  PID_reset(pid);
}

float PID_compute_pos(PID *pid, float current_pos, float dt){

  float error = pid -> _setpoint - current_pos;

  float P = pid -> _kp * error;

  // integrale solo se non siamo già saturi
  pid -> _integral += pid -> _ki * error * dt;
  if (pid -> _integral > pid -> _output_limit) pid -> _integral = pid -> _output_limit;
  if (pid -> _integral < -pid -> _output_limit) pid -> _integral = -pid -> _output_limit;

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
  // Clamping integrale basato sul limite PWM (es. 1000 per il timer)
  if (axis -> _pid_vel._integral > axis -> _pid_vel._output_limit) 
      axis -> _pid_vel._integral = axis -> _pid_vel._output_limit;
  if (axis -> _pid_vel._integral < -axis -> _pid_vel._output_limit) 
      axis -> _pid_vel._integral = -axis -> _pid_vel._output_limit;

  // Derivativa con Filtro Passa-Basso (N)
  // Fondamentale per non amplificare il rumore della derivazione numerica
  float raw_D = axis -> _pid_vel._kd * (error - axis -> _pid_vel._last_error) / dt;
  // Filtro alpha (N = 100 in Simulink corrisponde a circa 0.1 qui)
  float filtered_D = (axis -> _pid_vel._last_D * 0.9f) + (raw_D * 0.1f); 
  axis -> _pid_vel._last_D = filtered_D;
  axis -> _pid_vel._last_error = error;

  float stiction_pwm = 0.0f;
  
  float deadband = 400.0f; 
    if (axis->_target_vel > 0.02f) {
      stiction_pwm = deadband;
  } else if (axis->_target_vel < -0.02f) {
      stiction_pwm = -deadband;
  }
  float out = P + axis->_pid_vel._integral + filtered_D + (accel * axis->_ka) + stiction_pwm;

  if (out > axis -> _pid_vel._output_limit) out = axis -> _pid_vel._output_limit;
  if (out < -axis -> _pid_vel._output_limit) out = -axis -> _pid_vel._output_limit;

  axis -> _pid_vel._output = out;
  
  // *(axis -> _pwm_register) = (uint32_t)fabsf(out);
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

    uint32_t psc = htim->Instance->PSC + 1; // Il registro PSC è (valore - 1)

    uint32_t arr_val = (uint32_t)((float)timer_base_clock / (float)(psc * freq)) - 1;

    if (arr_val < 10) arr_val = 10; // Evita frequenze troppo alte per il driver Moons'
    if (arr_val > 0xFFFF && (htim->Instance != TIM2 && htim->Instance != TIM5)) {
        arr_val = 0xFFFF; // Limite per timer a 16 bit
    }

    htim->Instance->CR1 &= ~TIM_CR1_ARPE;
    __HAL_TIM_SET_AUTORELOAD(htim, arr_val);
    __HAL_TIM_SET_COMPARE(htim, channel, arr_val / 2); // Duty cycle al 50% 

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

void stepper_loop(Stepper *stepper, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, float max_speed, float kp, float kd, float dt) {

    float current_pos = stepper->_enc_rot->_converted_value;
    float error = stepper->_target - current_pos;

    float step_deg = 1.0f / stepper->steps_per_unit; 
    float inner_tol = 0.05f;
    float outer_tol = 0.15f;
    
    if (htim == &htim8) { 
        inner_tol = 0.15f; outer_tol = 0.30f; 
    } else if (htim == &htim15 || htim == &htim16) { 
        inner_tol = 0.15f; 
        outer_tol = 0.30f; 
    }

    if (stepper->_in_position == 1) {
        if (fabsf(error) > outer_tol) {
            stepper->_in_position = 0; 
        } else {
            stepper_command(0.0f, stepper->steps_per_unit, htim, channel, dir_port, dir_pin, stepper->_dir);
            return;
        }
    } else {
        if (fabsf(error) < inner_tol) {
            stepper->_in_position = 1;
            stepper_command(0.0f, stepper->steps_per_unit, htim, channel, dir_port, dir_pin, stepper->_dir);
            stepper->_current_speed_hz = 0.0f;
            stepper->_target_speed = 0.0f; 
            stepper->_last_error = error;
            return; 
        }
    }

    float derivative = -stepper->_enc_rot->_velocity; 
    float raw_speed = (error * kp) + (derivative * kd);

    float alpha = 0.1f; 
    if (htim == &htim8) {
        alpha = 0.005f; 
    }
    float required_speed = (stepper->_target_speed * (1.0f - alpha)) + (raw_speed * alpha);

    float approach_zone = 5.0f; 
    float dynamic_max_speed = max_speed;

    if (fabsf(error) < approach_zone) {
        dynamic_max_speed = max_speed * (fabsf(error) / approach_zone);
        float min_speed = 1.6f / stepper->steps_per_unit;
        if(dynamic_max_speed < min_speed) dynamic_max_speed = min_speed;
    }

    if (required_speed > dynamic_max_speed) required_speed = dynamic_max_speed;
    if (required_speed < -dynamic_max_speed) required_speed = -dynamic_max_speed;

    float max_accel = 5000.0f; // Default
    if (htim == &htim17 || htim == &htim3) {
        max_accel = 15.0f;
    } else if (htim == &htim8) {
        max_accel = 80.0f; 
    } else if (htim == &htim15 || htim == &htim16) {
        max_accel = 40.0f; 
    }
    
    float max_delta_v = max_accel * dt; 

    if (required_speed > stepper->_target_speed + max_delta_v) {
        required_speed = stepper->_target_speed + max_delta_v;
    } else if (required_speed < stepper->_target_speed - max_delta_v) {
        required_speed = stepper->_target_speed - max_delta_v;
    }

    stepper->_target_speed = required_speed; 
    stepper->_last_error = error;
    stepper->_current_speed_hz = required_speed;

    stepper_command(required_speed, stepper->steps_per_unit, htim, channel, dir_port, dir_pin, stepper->_dir);
}
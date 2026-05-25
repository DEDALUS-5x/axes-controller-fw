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
  float error = axis -> _pid_vel._setpoint - axis -> _enc_rot -> _velocity;

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

  float out = P + axis -> _pid_vel._integral + filtered_D + (accel * axis -> _ka);

  if (out > axis -> _pid_vel._output_limit) out = axis -> _pid_vel._output_limit;
  if (out < -axis -> _pid_vel._output_limit) out = -axis -> _pid_vel._output_limit;

  axis -> _pid_vel._output = out;
  
  *(axis -> _pwm_register) = (uint32_t)fabsf(out);
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


void stepper_command(float speed, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin) {
    
    HAL_GPIO_WritePin(dir_port, dir_pin, (speed >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    float freq = fabsf(speed) * STEPS_MM;

    if (freq < 10.0f) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0); 
        return;
    }


    uint32_t timer_base_clock;
    

    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM15 || 
        htim->Instance == TIM16 || htim->Instance == TIM17) {
        timer_base_clock = HAL_RCC_GetPCLK2Freq() * 2; // Solitamente APB2 x2 su H7
    } else {
        timer_base_clock = HAL_RCC_GetPCLK1Freq() * 2; // Solitamente APB1 x2 su H7
    }

    uint32_t psc = htim->Instance->PSC + 1; // Il registro PSC è (valore - 1)


    uint32_t arr_val = (uint32_t)((float)timer_base_clock / (float)(psc * freq)) - 1;

    if (arr_val < 10) arr_val = 10; // Evita frequenze troppo alte per il driver Moons'
    if (arr_val > 0xFFFF && (htim->Instance != TIM2 && htim->Instance != TIM5)) {
        arr_val = 0xFFFF; // Limite per timer a 16 bit
    }

    __HAL_TIM_SET_AUTORELOAD(htim, arr_val);
    __HAL_TIM_SET_COMPARE(htim, channel, arr_val / 2); // Duty cycle al 50% 

    // check anti glitch
    if (htim->Instance->CNT > arr_val) {
        htim->Instance->CNT = 0;
    }

    if (!(htim->Instance->CR1 & TIM_CR1_CEN)) {
        HAL_TIM_PWM_Start(htim, channel);
    }
}


void stepper_loop(Stepper *stepper, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, float max_speed, float kp) {

  float current_pos = stepper->_enc_rot->_converted_value;

  float error = stepper->_target - current_pos;

  float required_speed = error * kp;

  if (required_speed > max_speed) required_speed = max_speed;
  if (required_speed < -max_speed) required_speed = -max_speed;

  float tolerance = (stepper->_current_speed_hz == 0.0f) ? 0.6f : 0.15f; 
  
  if (fabsf(error) < tolerance) {
      stepper_command(0.0f, htim, channel, dir_port, dir_pin);
      stepper->_current_speed_hz = 0.0f; // Registriamo lo stato: FERMO
      return; 
  }

  stepper_command(required_speed, htim, channel, dir_port, dir_pin);
}

void stepper_loop_soft(Stepper *stepper, GPIO_TypeDef *dir_port, uint16_t dir_pin, float max_speed, float kp) {
    float current_pos = stepper->_enc_rot->_converted_value;
    float error = stepper->_target - current_pos;
    float required_speed = error * kp;
    
    if (required_speed > max_speed) required_speed = max_speed;
    if (required_speed < -max_speed) required_speed = -max_speed;
    float tolerance = 0.01f; 
    if (fabsf(error) < tolerance) {
        required_speed = 0.0f;
    }

    HAL_GPIO_WritePin(dir_port, dir_pin, (required_speed >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET); // direction
    stepper->_current_speed_hz = fabsf(required_speed) * STEPS_MM;
}
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

/*
void PID_Compute(Axis *axis, float dt){

  float error = axis->_pid._setpoint - axis->_feedback->_converted_value;

  float P = axis->_pid._kp * error;
  axis->_pid._integral += axis->_pid._ki * error * dt; // sommatoria da un momento passato
  float D = axis->_pid._kd * (error - axis->_pid._last_error) / dt;

  float out = P + axis->_pid._integral + D;

  if (out > axis->_pid._output_limit) {
      out = axis->_pid._output_limit;

  } else if (out < -axis->_pid._output_limit) {
      out = -axis->_pid._output_limit;

  }

  axis->_pid._output = out;
  axis->_pid._last_error = error;

  *(axis->_pwm_register) = (uint32_t)fabsf(out);
}
  */

void motor_command(Axis *axis, TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2) {
    if (axis -> _pid_vel._output > 0) {
        __HAL_TIM_SET_COMPARE(htim, channel1, (uint32_t)fabsf(axis -> _pid_vel._output));
        __HAL_TIM_SET_COMPARE(htim, channel2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(htim, channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, channel2, (uint32_t)fabsf(axis -> _pid_vel._output));
    }
}

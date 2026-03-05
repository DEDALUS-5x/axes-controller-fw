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

/*

   ____            _             _ 
  / ___|___  _ __ | |_ _ __ ___ | |
 | |   / _ \| '_ \| __| '__/ _ \| |
 | |__| (_) | | | | |_| | | (_) | |
  \____\___/|_| |_|\__|_|  \___/|_|
                                   

*/
#ifndef __CTRL_H__
#define __CTRL_H__

#include "main.h"
#include "types.h"

void PID_init(PID *pid, float kp, float ki, float kd, float limit);

void PID_Compute(Axis *axis, float dt);

void PID_reset(PID *pid);

void motor_command(Axis *axis, TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2);

#endif

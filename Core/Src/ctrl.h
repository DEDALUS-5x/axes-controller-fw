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

// void PID_compute(Axis *axis, float dt);

/**
 * 
 * @brief PID function of the position loop. It is called by 1kHz. Its output is the velocity inner loop input
 * @param pid Pointer to `PID` instance
 * @param current_pos Current position
 * @param dt time interval
 */
float PID_compute_pos(PID *pid, float current_pos, float dt);

/**
 * 
 * @brief PID function of the velocity loop. It is called by 10kHz. Its output feeds the motor control
 * @param axis pointer to `Axis` instance
 * @param dt time interval
 */
void PID_compute_vel(Axis *axis, float dt);

/**
 * 
 * @brief PID reset function. To be called when the machine is disabled/rebooted or has reached the zero homing position
 * @param pid Pointer to `PID` instance
 */
void PID_reset(PID *pid);

/**
 * 
 * @brief Motor command function that directly command the PWM that drive the DC motor. The PWM is given by the control PID loop
 * @param axis Pointer to `Axis` instance
 * @param htim Pointer to Timer instance
 * @param channel1 First Timer channel: for CW motor rotation
 * @param channel2 Second Timer channel for CCW motor rotation
 */
void motor_command(Axis *axis, TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2);

/**
 * 
 * @brief Stepper command function that outputs and modulate the pwm for the driver
 * @param speed Required speed
 * @param htim Pointer to Timer instance
 * @param channel Timer channel
 * @param dir GPIO port for the DIR pin
 * @param dir_pin GPIO pin for the DIR pin
 */
void stepper_command(float speed, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin);

/**
 * 
 * @brief Stepper loop function
 */
void stepper_loop(Stepper *stepper, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, float max_speed, float kp);

#endif

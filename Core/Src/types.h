/*

  ____  _                   _       
 / ___|| |_ _ __ _   _  ___| |_ ___ 
 \___ \| __| '__| | | |/ __| __/ __|
  ___) | |_| |  | |_| | (__| |_\__ \
 |____/ \__|_|   \__,_|\___|\__|___/
                                    

*/

#ifndef __TYPES_H__
#define __TYPES_H__

// PID struct
typedef struct {
    float _kp, _ki, _kd;
    float _setpoint;
    float _integral;
    float _last_error;
    float _output_limit;
    float _output;
    float _last_D;
} PID;

// Encoder struct
typedef struct {
    int32_t _raw_value;
    float _converted_value;
    float _velocity;
    // reference to DMA

} Encoder;

typedef struct {
    Encoder *_enc_rot;
    Encoder *_enc_lin;
    PID _pid_pos;
    PID _pid_vel;

    // from raspi
    float _target_vel; // from feedforward
    float _last_vel;
    float _ka;

    volatile uint32_t *_pwm_register; // * al registro CCR del timer configurato in pwm
} Axis;

typedef struct {
    float target_pos[3];    // X, Y1, Y2
    float target_rot[2];
    float velocity[3];
    float acceleration[3];
} ProfileCMD;

#endif
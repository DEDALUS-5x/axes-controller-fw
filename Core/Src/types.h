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
    float _offset;
    float g_ratio;

    float _last_raw_pos;
    float _last_converted_value;
    float _last_velocity;
    float _acceleration;
    int32_t _turns;

} Encoder;

typedef struct {
    Encoder *_enc_rot;
    Encoder *_enc_lin;
    PID _pid_pos;
    PID _pid_vel;

    // from raspi
    float _target_pos;
    float _target_vel; // from feedforward
    float _last_vel;
    float _ka;

    volatile uint32_t *_pwm_register; //registro CCR del timer pwm
} Axis;

typedef struct {

    Encoder *_enc_rot;
    float _target;
    float steps_per_unit;
    float _current_speed_hz;
    float _target_speed;
    float _last_error;
    uint8_t _dir;
    uint8_t _a;

} Stepper;

#pragma pack(push, 1)

// Full duplex communication with raspberry: 96 bytes
typedef struct __attribute__((packed)){
    uint8_t start;
    float x;
    float y;
    float z;
    float a;
    float c;
    float vx;
    float vy;
    uint8_t check;
    uint8_t padding[66];
} SPIPacket;

typedef struct __attribute__((packed)) {
    uint8_t  start;
    uint32_t msg_id;
    float    x;
    float    y;
    float    z;
    float    a;
    float    c;
    float    vx;
    float    vy;
    float    va;
    float    vc;
    float    vz;
    float    ax;
    float    ay;
    float    az;
    float    aa;
    float    ac;
    float    error;
    uint8_t  check;
    uint8_t padding[26];
} SPITxPacket;

#pragma pack(pop)

#endif
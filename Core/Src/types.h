/*

  ____  _                   _       
 / ___|| |_ _ __ _   _  ___| |_ ___ 
 \___ \| __| '__| | | |/ __| __/ __|
  ___) | |_| |  | |_| | (__| |_\__ \
 |____/ \__|_|   \__,_|\___|\__|___/
                                    

*/

// PID struct
typedef struct {
    float _kp, _ki, _kd;
    float _setpoint;
    float _integral;
    float _last_error;
    float _output_limit;
    float _output;
} PID;

// Encoder struct
typedef struct {
    int32_t _raw_value;
    float _converted_value;
    float _velocity;
    // reference to DMA

} Encoder;

typedef struct {
    Encoder *_feedback;
    PID _pid;
    uint32_t *_pwm_register; // * al registro CCR del timer configurato in pwm
} Axis;
#pragma once

#include <stdint.h>
#include "motor_messages/motor_messages.h"

typedef MotorCommand ReceiveData;
typedef MotorStatus SendData;
typedef MotorMode MainControlMode;
typedef uint32_t mcu_time;  // a timestamp in cpu cycles

typedef struct {
    float kp;               // proportional gain, units of V/A for current control
    float ki;               // integral gain, same units as proportional gain but per cycle, note this may change to per second
    float ki_limit;         // integrator saturation, same units as output saturation
    float command_max;      // Output saturation, units of V for current control, A for position control
} PIParam;

typedef struct {
    float kp;               // proportional gain, units of A/rad for position control
    float ki;               // \sa PIParam.ki 
    float ki_limit;         // \sa PIParam.ki_limit
    float kd;               // derivative gain, implemented on error units same as kp * seconds
    float command_max;      // \sa PIParam.command_max
    float velocity_filter_frequency_hz; // First order filter on velocity feedback
    float output_filter_frequency_hz; // First order filter on output
} PIDParam;

typedef struct {
    PIParam pi_d;           // PIParam for d axis current - often make the same as pi_q
    PIParam pi_q;           // PIParam for q axis current
    float current_filter_frequency_hz;  // First order filter on current measurements
    float num_poles;        // number of motor pole pairs - i.e. number of motor magnets/2
} FOCParam;

#define COGGING_TABLE_SIZE 8192  // must be multiple of 2
typedef struct {
    float adc1_offset, adc2_offset, adc3_offset;    // initial guess at current sensor bias in counts - default 2048
    float adc1_gain, adc2_gain, adc3_gain;          // current sensor linear gain units A/count
    FOCParam foc_param;
    uint8_t phase_mode;     // two possible motor wiring states: 0: standard, 1: reverse (i.e. two motor leads flipped)
    struct {
        float index_electrical_offset_pos;          // index offset electrial zero in encoder counts
                                                    // can obtain this value from  motor_index_pos_ - motor_electrical_zero_pos_ 
                                                    // in fast_loop if use_index_electrical_offset_pos == 0 
        uint8_t use_index_electrical_offset_pos;    // Set to 1 to enable using the index_electrical_offset_pos above, 0 to disable
                                                    // Allows for more repeatable commutation from a quadrature encoder with index or absolute encoder
        uint32_t cpr;                               // Counts/revolution for encoder, for quadrature encoders 4x lines per revolution
        float dir;                                  // Set to 1 for positive output, -1 for negative
        int32_t rollover;                           // Encoder counts will rollover from +rollover to -rollover when it reaches this value. 
                                                    // Position control will take the shortest route. Velocity control is continuous.
                                                    // Set to 0 to disable.
                                                    // Ideally set to an even multiple of cpr smaller than 8388608 for no resolution loss 
    } motor_encoder;
    struct {
        float table[COGGING_TABLE_SIZE];            // cogging table in A
        float gain;                                 // cogging table multiplier - 0 to disable, 1 for 1:1 ratio
    } cogging;
    float vbus_gain;                                // vbus sensor gain units V/count
} FastLoopParam;

typedef struct {
    float gain;
    float bias;
    float k_temp;
    float filter_frequency_hz;
} TorqueSensorParam;

typedef struct {
    PIDParam position;
} PositionControllerParam;

typedef struct {
    PIDParam torque;
} TorqueControllerParam;

typedef struct {
    PIDParam impedance;
    PIDParam torque;
} ImpedanceControllerParam;

typedef struct {
    PIDParam position;
} VelocityControllerParam;

typedef struct {
    PositionControllerParam position_controller_param;
    TorqueControllerParam torque_controller_param;
    ImpedanceControllerParam impedance_controller_param;
    VelocityControllerParam velocity_controller_param;
    struct {
        float cpr;                                  // output encoder cpr \sa FastLoopParam.motor_encoder.cpr
    } output_encoder;
    TorqueSensorParam torque_sensor;
    int16_t host_timeout;                             // 0 to disable, if no commands received before host timeout, go to safe_mode
    MainControlMode safe_mode;                 // goes to this mode and freeze command if error
                                                    // need to send reset from host to exit
    float torque_correction;
    float torque_gain, torque_bias;                 // not currently used
    float kt;                                       // not currently used
    float gear_ratio;                               // not currently used
} MainLoopParam;

typedef struct {
    uint8_t do_phase_lock;          // 1: yes, 0: no
    float phase_lock_current;       // current in A
    float phase_lock_duration;      // duration in seconds
    MainControlMode startup_mode;
} StartupParam;

typedef struct {
    struct { float i_d, i_q, v_q; } desired;         // desired current in A, i_d typically 0, i_q creates torque, v_q in V is a feedforward
    struct { float i_a, i_b, i_c, motor_encoder; } measured;    // sensor currents in A, motor_encoder in rad referenced to electrical zero
} FOCCommand;

typedef struct {
    struct {
        float i_d, i_q;                         // \sa FOCCommand.desired
    } desired;
    struct {
        float position;                         // motor electrical position, rad filtered by 1/10
        float i_d, i_q, i_0;                    // measured processed currents, A filtered
        float sin, cos;                         // sin and cos of the electrical position
    } measured;
    struct { float v_a, v_b, v_c, v_d, v_q; } command;  // command in V to PWM
} FOCStatus;

typedef struct {
    mcu_time timestamp;                 // timestamp in microcontroller clock cycles
    FOCStatus foc_status;
    struct {
        int32_t raw;                    // raw counts since startup
        float position;                 // position in radians, 0 at startup or absolute value
        float velocity;                 // velocity in rad/s some filter        
    } motor_position;
    float motor_mechanical_position;    // counts referenced to index
    FOCCommand foc_command;
    float t_seconds, dt;                // time since startup in seconds, will lose resolution, dt is measured time in seconds
    float vbus;                         // bus voltage V
} FastLoopStatus;

typedef struct {
    FastLoopStatus fast_loop;
    float torque_filtered;
} MainLoopStatus;

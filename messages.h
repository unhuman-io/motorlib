#ifndef UNHUMAN_MOTORLIB_MESSAGES_H_
#define UNHUMAN_MOTORLIB_MESSAGES_H_

#include <stdint.h>
#include "motor_messages/motor_messages.h"

#ifdef __cplusplus
using namespace obot;
#endif

typedef MotorCommand ReceiveData;
#ifndef CUSTOM_SENDDATA
typedef MotorStatus SendData;
#endif
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
                            // for linear encoders set to poles per mm * 2 * pi
} FOCParam;

#define COGGING_TABLE_SIZE 512  // must be multiple of 2
#define MOTOR_ENCODER_TABLE_LENGTH  512
#define OUTPUT_ENCODER_TABLE_LENGTH  128
#define TORQUE_TABLE_LENGTH  512

typedef struct {
    float ia_bias, ib_bias, ic_bias;                // initial guess at current sensor bias in amps
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
                                                    // For linear encoders set to counts per mm * (2*pi) or similar
        float dir;                                  // Set to 1 for positive output, -1 for negative
        int32_t rollover;                           // Encoder counts will rollover from +rollover to -rollover when it reaches this value. 
                                                    // Position control will take the shortest route. Velocity control is continuous.
                                                    // Set to 0 to disable.
                                                    // Ideally set to an even multiple of cpr smaller than 8388608 for no resolution loss
        float table[MOTOR_ENCODER_TABLE_LENGTH][4]; // Additive pchip correction table in motor radians. motor_position = measured_motor_position + table 
    } motor_encoder;
    struct {
        float table[COGGING_TABLE_SIZE][4];            // cogging table in A, pchip
        float gain;                                 // cogging table multiplier - 0 to disable, 1 for 1:1 ratio
    } cogging;
    float vbus_gain;                                // vbus sensor gain units V/count
    float beep_frequency;                           // frequency hz for audio feedback
    float beep_amplitude;                           // beep amplitude A
} FastLoopParam;

typedef struct {
    float gain;
    float bias;
    float k_temp;
    float table[TORQUE_TABLE_LENGTH][4];
    float table_gain;
    float dir;
} TorqueSensorParam;

typedef struct {
    PIDParam position;
    float velocity_limit;
    float desired_filter_hz;
} PositionControllerParam;

typedef struct {
    PIDParam torque;
} TorqueControllerParam;

typedef struct {
    PIDParam impedance;
    PIDParam torque;
} ImpedanceControllerParam;

typedef struct {
    float velocity_filter_frequency_hz;
    float torque_filter_frequency_hz;
    float torque_dot_filter_frequency_hz;
    float output_filter_frequency_hz;
    float ff_tau;
    float command_max;
} StateControllerParam;

typedef struct {
    PIDParam velocity;
    float acceleration_limit;
} VelocityControllerParam;

typedef struct {
    VelocityControllerParam velocity;       // inner velocity controller
    float kpj;                              // outer loop gain on joint position (motor rad/s)/(joint rad)
} JointPositionControllerParam;

typedef struct {

} FindLimitsControllerParam;                // note uses the velocity_controller_param of the velocity controller
                                            // and position_controller_param of the position controller

typedef struct {
    PositionControllerParam position_controller_param;
    TorqueControllerParam torque_controller_param;
    ImpedanceControllerParam impedance_controller_param;
    VelocityControllerParam velocity_controller_param;
    StateControllerParam state_controller_param;
    JointPositionControllerParam joint_position_controller_param;
    struct {
        float table[OUTPUT_ENCODER_TABLE_LENGTH][4];
        float cpr;                                  // output encoder cpr \sa FastLoopParam.motor_encoder.cpr
        float bias;
        float dir;                                  // -1 or 1
    } output_encoder;
    struct {
        float motor_hard_max;         // will switch to safe mode if going past these limits
        float motor_hard_min;         // ignored if both are set to the same value
        float output_hard_max;
        float output_hard_min;
        float motor_controlled_max;   // will attempt to use position control to stay in these limits
        float motor_controlled_min;
    } encoder_limits;
    TorqueSensorParam torque_sensor;
    int16_t host_timeout;                             // 0 to disable, if no commands received before host timeout, go to safe_mode
    MainControlMode safe_mode;                 // goes to this mode and freeze command if error
                                                    // need to send reset from host to exit
    float torque_correction;
    float vbus_min;
    float vbus_max;
    MotorError error_mask;              // can set to ERROR_MASK_ALL or ERROR_MASK_NONE or others
    uint8_t safe_mode_driver_disable;   // driver is disabled in safe mode
    uint8_t no_latch_driver_fault;      // 1 allows for the driver_fault to be reset by software
} MainLoopParam;

typedef struct {
    uint8_t do_phase_lock;          // 1: yes, 0: no
    float phase_lock_current;       // current in A
    float phase_lock_duration;      // duration in seconds
    enum {
        ENCODER_ZERO, // motor encoder is 0 at startup plus any absolute offset
        ENCODER_BIAS, // motor encoder is set to bias at startup
        ENCODER_BIAS_FROM_OUTPUT, // motor encoder bias is set to 
                        // (output_encoder/cpr - output_encoder.bias)*gear_ratio+motor_encoder_bias
                        // Note: this requires that output encoder and motor encoder both increment in the 
                        // same direction, that is positive motor encoder is also positive output encoder
        ENCODER_BIAS_FROM_OUTPUT_WITH_MOTOR_CORRECTION,  // use motor encoder absolute
                        // reading to jump only in integer numbers of the gear ratio*motor_cpr. It will likely 
                        // need to use calibrated output and motor encoder values to do this. 
        ENCODER_BIAS_FROM_OUTPUT_WITH_TORQUE_AND_MOTOR_CORRECTION // as above but also use 
                        // transmission stiffness to correct the output encoder measurement
                        // 
    } motor_encoder_startup;
    float gear_ratio;   // gear ratio from input to output
    float motor_encoder_bias;   // for ENCODER_BIAS and ENCODER_BIAS_FROM_OUTPUT: extra bias to add to motor encoder
                                // for ENCODER_BIAS_*_WITH_MOTOR_CORRECTION: motor bias to give a desired motor zero position
                                    // for example when output position = 0 if motor position is -1, then motor_encoder_bias = 1
    float num_encoder_poles;    // if motor encoder is only absolute per revolution % num_encoder_poles
    float transmission_stiffness; // also use transmission stiffness to help with motor bias setting
    float output_encoder_rollover; // if the output encoder+bias is greater than this then output_encoder -= 2*pi
    MainControlMode startup_mode;
    uint8_t no_zero_current_sensors;   // default of 0 will zero current sensors for 2 seconds on startup
                                       // 1 to disable and use fast_loop_param.i*_bias
    uint8_t no_driver_enable;       // 1 to require sending driver_enable mode and clear faults to enable the driver
} StartupParam;

typedef struct {
    struct { float i_d, i_q, v_q; } desired;         // desired current in A, i_d typically 0, i_q creates torque, v_q in V is a feedforward
    struct { float i_a, i_b, i_c, motor_encoder; } measured;    // sensor currents in A, motor_encoder in mechanical rad referenced to electrical zero
} FOCCommand;

typedef struct {
    struct {
        float i_d, i_q;                         // \sa FOCCommand.desired
    } desired;
    struct {
        float i_d, i_q, i_0;                    // measured processed currents, A filtered
    } measured;
    struct { float v_a, v_b, v_c, v_d, v_q; } command;  // command in V to PWM
} FOCStatus;

typedef struct {
    mcu_time timestamp;                 // timestamp in microcontroller clock cycles
    FOCStatus foc_status;
    struct {
        int32_t raw;                    // raw counts since startup
        float position;                 // position in radians, 0 at startup or absolute value    
    } motor_position;
    float motor_mechanical_position;    // counts referenced to index
    FOCCommand foc_command;
    float power;                        // estimated power in W
    uint32_t energy_uJ;                  // rolling over sum of micro Joules (rollover at 4000J/1.1 Wh)
    float vbus;                         // bus voltage V
} FastLoopStatus; // 22*4 bytes

typedef struct {
    FastLoopStatus fast_loop;
    float torque;
    float output_position;
    float motor_position;
    MotorMode mode;
    MotorError error;
    RoundRobinData rr_data;
    float power;
} MainLoopStatus;

#endif  // UNHUMAN_MOTORLIB_MESSAGES_H_

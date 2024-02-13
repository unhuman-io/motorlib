#include "math.h"
#include "param.h"

// Can be written by external methods, e.g. bootloader
const volatile Param __attribute__((section("calibration_data"))) initial_param = {
    .startup_param.motor_encoder_bias = 0,
    .main_loop_param.output_encoder.bias = 0,
    .fast_loop_param.motor_encoder.index_electrical_offset_pos = -1868,
    .main_loop_param.output_encoder.bias = -25,
    .main_loop_param.torque_sensor.bias = 0,
    .joint_encoder_bias = 0,
    .name = "J1",
};

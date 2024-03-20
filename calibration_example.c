#include "math.h"
#include "messages.h"

// Can be written by external methods, e.g. bootloader
Calibration __attribute__((section("calibration_data"))) initial_calibration = {
    .schema_version = 2,
    .motor_encoder_bias = 0,
    .motor_encoder_index_electrical_offset_pos = 0,
    .output_encoder_bias = 0,
    .torque_sensor_bias = 0,
    .joint_encoder_bias = 0,
    .error_mask.all = ERROR_MASK_ALL
};

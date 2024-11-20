#include <cstdint>
#include "../motor_messages/motor_messages.h"

using namespace obot;

int main() {
    assert(MotorError{.sequence=1}.all == ERROR_MASK_SEQUENCE);
    assert(MotorError{.bus_voltage_low=1}.all == ERROR_MASK_BUS_VOLTAGE_LOW);
    assert(MotorError{.bus_voltage_high=1}.all == ERROR_MASK_BUS_VOLTAGE_HIGH);
    assert(MotorError{.bus_current=1}.all == ERROR_MASK_BUS_CURRENT);
    assert(MotorError{.microcontroller_temperature=1}.all == ERROR_MASK_MICROCONTROLLER_TEMPERATURE);
    assert(MotorError{.board_temperature=1}.all == ERROR_MASK_BOARD_TEMPERATURE);
    assert(MotorError{.motor_temperature=1}.all == ERROR_MASK_MOTOR_TEMPERATURE);
    assert(MotorError{.driver_fault=1}.all == ERROR_MASK_DRIVER_FAULT);
    assert(MotorError{.motor_overcurrent=1}.all == ERROR_MASK_MOTOR_OVERCURRENT);
    assert(MotorError{.motor_phase_open=1}.all == ERROR_MASK_MOTOR_PHASE_OPEN);
    assert(MotorError{.motor_encoder=1}.all == ERROR_MASK_MOTOR_ENCODER);
    assert(MotorError{.motor_encoder_limit=1}.all == ERROR_MASK_MOTOR_ENCODER_LIMIT);
    assert(MotorError{.output_encoder=1}.all == ERROR_MASK_OUTPUT_ENCODER);
    assert(MotorError{.output_encoder_limit=1}.all == ERROR_MASK_OUTPUT_ENCODER_LIMIT);
    assert(MotorError{.torque_sensor=1}.all == ERROR_MASK_TORQUE_SENSOR);
    assert(MotorError{.controller_tracking=1}.all == ERROR_MASK_CONTROLLER_TRACKING);
    assert(MotorError{.host_fault=1}.all == ERROR_MASK_HOST_FAULT);
    assert(MotorError{.driver_not_enabled=1}.all == ERROR_MASK_DRIVER_NOT_ENABLED);
    assert(MotorError{.encoder_disagreement=1}.all == ERROR_MASK_ENCODER_DISAGREEMENT);
    assert(MotorError{.torque_sensor_disagreement=1}.all == ERROR_MASK_TORQUE_SENSOR_DISAGREEMENT);
    assert(MotorError{.init_failure=1}.all == ERROR_MASK_INIT_FAILURE);
    assert(MotorError{.invalid_command=1}.all == ERROR_MASK_INVALID_COMMAND);
    assert(MotorError{.motor_encoder_warning=1}.all == ERROR_MASK_MOTOR_ENCODER_WARNING);
    assert(MotorError{.output_encoder_warning=1}.all == ERROR_MASK_OUTPUT_ENCODER_WARNING);
    assert(MotorError{.torque_sensor_warning=1}.all == ERROR_MASK_TORQUE_SENSOR_WARNING);
    assert(MotorError{.motor_current_limit=1}.all == ERROR_MASK_MOTOR_CURRENT_LIMIT);
    assert(MotorError{.motor_voltage_limit=1}.all == ERROR_MASK_MOTOR_VOLTAGE_LIMIT);
    assert(MotorError{.motor_soft_limit=1}.all == ERROR_MASK_MOTOR_SOFT_LIMIT);
    assert(MotorError{.fault=1}.all == ERROR_MASK_FAULT);
    assert(MotorError{}.all == 0);
    return 0;
}
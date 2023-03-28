#ifndef UNHUMAN_MOTORLIB_SYSTEM_TYPES_H_
#define UNHUMAN_MOTORLIB_SYSTEM_TYPES_H_

#ifdef INSTANCE_SYSTEM_TYPES_HEADER

#include INSTANCE_SYSTEM_TYPES_HEADER

#else
// === Default type definitions. ===
#include "communication.h"
#include "controller/impedance_controller.h"
#include "controller/joint_position_controller.h"
#include "controller/position_controller.h"
#include "controller/state_controller.h"
#include "controller/torque_controller.h"
#include "controller/velocity_controller.h"
#include "driver.h"
#include "encoder.h"
#include "hardware_brake.h"
#include "peripheral/pwm.h"
#include "torque_sensor.h"

using CommunicationType = CommunicationBase;
using DriverType = DriverBase;
using PWMType = PWMBase;
using MotorEncoderType = EncoderBase;
using OutputEncoderType = EncoderBase;
using TorqueSensorType = TorqueSensorBase;
using HardwareBrakeType = HardwareBrakeBase;

using PositionControllerType = PositionController;
using TorqueControllerType = TorqueController;
using ImpedanceControllerType = ImpedanceController;
using VelocityControllerType = VelocityController;
using StateControllerType = StateController;
using JointPositionControllerType = JointPositionController;
#endif  // INSTANCE_SYSTEM_TYPES_HEADER

#endif  // UNHUMAN_MOTORLIB_SYSTEM_TYPES_H_
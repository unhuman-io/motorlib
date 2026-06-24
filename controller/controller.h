#pragma once

#include <concepts>
#include "../messages.h"

//typedef ReceiveData MotorCommand;

template <typename T>
concept IsController = requires(T controller, const MotorCommand& cmd, const MainLoopStatus& status) {
  { controller.step(cmd, status) } -> std::same_as<float>;
  { controller.validate_command(cmd) } -> std::same_as<bool>;
  { controller.init(status) } -> std::same_as<void>;
};

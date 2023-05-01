# State controller

The controller diagram for the state controller is below. This controller covers many possible use cases without needing to switch modes. Sending 0 commands for inputs and gains will effectively result in those feedback loops being disabled. The final input after saturation to the "System" in the diagram is a current. This controller could simply be used as a current controller using the `current_desired` input. It can be a PD position controller by using position in velocity inputs and gains. It can be a PD torque controller by using the torque inputs and gains. And it can control the state defined by position and torque and their derivatives. This selection of state was chosen based on the work of Christian Ott, published in "Cartesian Impedance Control of Redundant and Flexible-Joint Robots" in 2008. In all the cases, complete gain scheduling is available to deal with variable system configurations.

![state controller diagram](state_controller.svg#gh-light-mode-only)
![state controller diagram](state_controller_dark.svg#gh-dark-mode-only)

As per the convention of `motorlib`, filtering is taken care of by the controller. The current implementation of the state controller uses first order low pass filtering of various signals. Derivative terms are computed by a backward difference. Finally a first order low pass filter is used on the output as well. This serves as a catch-all for various noise sources, such as discretization of the command inputs, which include the gains. The output filter can also be used as a means for analyzing the system performance in a lead or lag configuration. By no means are these filters or this controller configuration a complete or best solution for all possible use cases but rather just a useful starting point for system controls development. The recommended approach to modifying filter implementations is to either derive from, or make a copy of the state controller for a custom system specific implementation.

The actual realtime command interface is the [`StateControllerCommand`](https://github.com/unhuman-io/motor_messages/blob/ccfa577f92dc7b82534664f639baf24b8acd76f7/motor_messages.h#L135) in [`motor_messages.h`](../motor_messages/motor_messages.h). Parameters are definined in [`StateControllerParam`](https://github.com/unhuman-io/motorlib/blob/4dcba4bdafbc64327b806b476517bcbbcb4a0b4d/messages.h#L95) in [`messages.h'](../messages.h). The tuning interface to the parameters is:
  - `state_command_max`
  - `state_output_filter`
  - `state_velocity_error_filter`
  - `state_torque_error_filter`
  - `state_torque_error_dot_filter`

Tuning parameters also may be used as a means for overriding stored parameters. For example in c++ it is sometimes convenient to have startup code of the form:
```c++
motor["state_command_max"] = "value"; // value is a string, for example "10" would set the current limit to 10 A
...
```

Finally, the implementation of the state controller is in [`state_controller.h`](../controller/state_controller.h).

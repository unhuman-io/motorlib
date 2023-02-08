# Basic bring up

This document gives the prescriptive steps necessary to obtain and build firmware for a specific actuator. The steps include:
1. Obtaining this firmware repo
2. Installing `gcc` for the firmware build
3. Installing `motor_util` and the `usb_rt` driver using the obot install script
4. Building and loading the firmware for a specific actuator configuration
5. Steps to determine commutation and encoder directions

## Obtaining the firmware repo
```console
git config --global submodule.recurse true
git clone --recursive https://github.com/unhuman-io/obot-controller.git
```

## Installing gcc
```console
cd obot-controller
./scripts/motorlib/install_gcc.sh
```

## Installing `motor_util` and the `usb_rt` driver using the obot install script
See the install steps at github.com/unhuman-io/obot, or for most cases just run:
```console
sudo apt install -y linux-headers-$(uname -r)
curl https://raw.githubusercontent.com/unhuman-io/obot/main/install-obot.sh > install-obot.sh
chmod +x install-obot.sh
./install-obot.sh
```

## Building the firmware for a specific actuator configuration
A param and config file must be selected during the build process. The files are located in the `config/` and `param/` directories respectively and are selected by their suffix. For example for the osa14 configuration and parameters:
```console
make CONFIG=osa14 PARAM=osa14 -j
./build/osa14/load_osa14.sh
```

## Steps to determine commutation and encoder directions
The key parameters specified in the param file that are necessary to commutate a motor are `num_poles`, `motor_encoder_cpr`, and `phase_mode`. `num poles` is actually the number of pole pairs. It and the encoder cpr (counts per revolution) should be determined from the physical motor parameters. `phase_mode` is most easily determined experimentally and refers to the order in which motor leads are connected. It can have a value of either 0 or 1, which correspond to reversing two wires on the motor. If motor wiring remains consistent between different motors, then `phase_mode` will only need to be determined once for that type of motor.
```console
motor_util set --mode phase_lock --current CONTINUOUS_MOTOR_CURRENT_IN_A
sleep 2
motor_util set --mode current --current ABOVE_STARTING_MOTOR_CURRENT_IN_A
```
If the motor spins with the above commands then commutation is correct. If it does not spin and instead tends to lock to specific positions, then `phase_mode` is incorrect. Just toggle the value in the param file (0 becomes 1 or 1 becomes 0), recompile and reload firmware, then try the above steps again (or set `phase_mode` via the debug api). If the motor still does not spin then the problem is more fundamental and `num_poles`, `motor_encoder_cpr`, the correct param and config files, and sensor signals should be checked.

Next the current direction, motor/output encoder directions, and the torque sensor direction should be determined. It is important that positive current creates positive motion in the preferred direction and positive signals on the motor and output encoders. It the output is held fixed then positive current should also produce a positive torque signal as measured by the torque sensor if equipped. By running the above current, and observing the sensors signals the following variables will change the sensor signal or current directions.
- `motor_util set --mode phase_lock --current -CONTINUOUS_MOTOR_CURRENT_IN_A`: Note the negative current. Direction of rotation effected by a positive current command
- `motor_encoder_dir`: Positive direction of the motor encoder that affects position, velocity, and all other controllers
- `output_encoder_dir`: Positive direction of the output encoder that affects the output reading and encoder bias zeroing at startup
- `torque_sensor_gain`: Lumped parameter may be positive or negative that sets the gain and direction of the torque sensor
These values may be changed in the parameters file. Recompile and reload to test their affect.

An additional parameter that is necessary if continuous output rotation is desired is `rollover`. Since the internal and communicated variable `motor_position` is a float value in radians, it will eventually lose precision when the motor rotates past 2^23 motor encoder noise free counts. It should be set to an integer multiple of the encoder cpr and will cause the the motor position to wrap at this value when converted to radians. For example for a 100,000 cpr effective encoder a good choice for rollover is `100*100,000`, which would cause the motor position to wrap at `2*pi*100` radians to `-2*pi*100`.

After proper commutation is determined, if the motor encoder is absolute or has an index pulse, then commutation can be determined at startup without the additonal `phase_lock` step. To determine the correct offset parameter, first ensure that the encoder has passed the index pulse if applicable then enter:
```console
motor_util set --mode phase_lock --current (+/-)CONTINUOUS_MOTOR_CURRENT_IN_A
sleep 2
motor_util --set-api index_offset_measured
```
This will return the `fast_loop_param.motor_encoder.index_electrical_offset_pos` that can be entered into the parameter file. To make `index_offset_measured` active, also set `fast_loop_param.motor_encoder.use_index_electrical_offset_pos = 1` in the parameter file as well.
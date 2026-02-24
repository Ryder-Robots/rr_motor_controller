# rr_motor_controller

Lifecycle-managed ROS 2 ECU node for a differential-drive robot. `RrECU` accepts `geometry_msgs/msg/Twist` velocity commands and distributes them to two `RrMotorController` instances (left and right), each driving a DC motor and encoder pair via a shared GPIO plugin.

## Installation

```bash
# One-time setup (if not already done)
curl -fsSL https://ryder-robots.github.io/rr-apt/public.gpg | sudo gpg --dearmor -o /usr/share/keyrings/rr-apt.gpg
echo "deb [signed-by=/usr/share/keyrings/rr-apt.gpg] https://ryder-robots.github.io/rr-apt noble main" | sudo tee /etc/apt/sources.list.d/rr-apt.list

# Install
sudo apt update
sudo apt install ros-kilted-rr-motor-controller
```

## Control Loop

1. **subscribe_callback_** receives `Twist` messages on `/cmd_vel` and converts them to per-motor velocity and direction commands using a differential-drive kinematic model.
2. **encoder_cb_** runs in GPIO interrupt context per motor, accumulating pulse timing over one full revolution and computing measured velocity via EMA smoothing.
3. **pid_cb_** runs in a dedicated `SCHED_FIFO` thread per motor, converting target velocity to a PWM duty cycle (currently via linear regression) and applying it to the motor at a fixed 100 ms interval.
4. **publish_callback_** fires on a 250 ms wall timer, publishing estimated pose and velocity to `/odom`.

## Lifecycle Transitions

| Transition | What happens |
|---|---|
| **configure** | Loads parameters, loads GPIO plugin, configures left and right motor controllers |
| **activate** | Activates GPIO plugin, sets pin modes, attaches encoder ISRs, creates `/cmd_vel` subscription and `/odom` publisher, starts control loops |
| **deactivate** | Cancels timers, tears down ROS interfaces, stops PID threads, detaches ISRs, sets PWM to 0 |
| **cleanup** | Releases plugin loader, resets all shared pointers |

## Parameters

All parameters are declared in the `RrECU` constructor.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `motor_count` | int | `0` | Number of motors to manage (must be 2 for differential drive) |
| `encoder_pins` | int[] | `[]` | GPIO pins connected to each encoder — **one entry per motor**, ordered by motor index (index 0 = left, index 1 = right) |
| `pwm_pins` | int[] | `[]` | GPIO pins used for PWM output — **one entry per motor**, same index order as `encoder_pins` |
| `dir_pins` | int[] | `[]` | GPIO pins used for direction control — **one entry per motor**, same index order as `encoder_pins` |
| `ppr` | int | `8` | Pulses per revolution (shared across motors) |
| `wheel_radius` | int | `20` | Wheel radius in mm (assumed uniform). Default of 20 mm corresponds to wheels with a 40 mm diameter |
| `wheel_base` | int | `70` | Distance in mm between the contact points of the left and right wheels |
| `pwm_freq` | int | `2000` | PWM frequency in Hz |
| `encoder_timeout` | int | `0` | ISR timeout in µs; 0 disables timeout callbacks |
| `ttl_ns` | int | `200000000` | Command time-to-live in nanoseconds; commands older than this are discarded |
| `covariance` | double[] | `[]` | 36-element row-major 6×6 pose covariance matrix written to `odom.pose.covariance`. Diagonal elements represent variance for [x, y, z, roll, pitch, yaw]. z, roll, and pitch are not tracked (flat floor), so their diagonal entries should be set to a large value (e.g. `1e6`) to signal high uncertainty to the navigation stack |
| `transport_plugin` | string | — | Pluginlib class name for the GPIO transport layer. For Raspberry Pi 4B use `rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin` (see [rr_gpio_pi4b_pigpio_plugin](https://github.com/Ryder-Robots/rr_gpio_pi4b_pigpio_plugin)) |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscribe | Linear and angular velocity commands from the navigation stack |
| `/odom` | `nav_msgs/msg/Odometry` | Publish | Estimated robot pose and velocity, published at 250 ms |

## Example Usage

### Launch file (Python)

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='motor_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='rr_motor_controller',
                    plugin='rr_motor_controller::RrECU',
                    name='motor_ecu',
                    parameters=[{
                        'motor_count': 2,
                        'ppr': 8,
                        'wheel_radius': 20,   # 20 mm radius = 40 mm diameter wheels
                        'wheel_base':   70,   # 70 mm between left and right wheel contact points
                        'pwm_freq': 1000,
                        # One pin per motor: index 0 = left, index 1 = right
                        'encoder_pins': [17, 27],
                        'pwm_pins':     [12, 13],
                        'dir_pins':     [24, 25],
                        'encoder_timeout': 500000,
                        # Raspberry Pi 4B GPIO plugin — see https://github.com/Ryder-Robots/rr_gpio_pi4b_pigpio_plugin
                        'transport_plugin': 'rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin',
                        # 6×6 row-major pose covariance [x, y, z, roll, pitch, yaw].
                        # Diagonal: x=0.01, y=0.01, z=1e6, roll=1e6, pitch=1e6, yaw=0.01
                        # z, roll, pitch set to 1e6 — not tracked on a flat floor.
                        'covariance': [
                            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
                            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
                            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
                            0.0,  0.0,  0.0,  0.0,  0.0,  0.01,
                        ],
                    }],
                ),
            ],
        ),
    ])
```

### Lifecycle transitions via CLI

```bash
# Configure the node (loads parameters, initialises hardware)
ros2 lifecycle set /motor_ecu configure

# Activate the node (starts control loops)
ros2 lifecycle set /motor_ecu activate

# Deactivate the node (stops motors, tears down)
ros2 lifecycle set /motor_ecu deactivate

# Cleanup (releases all resources)
ros2 lifecycle set /motor_ecu cleanup
```

### Checking node state

```bash
ros2 lifecycle get /motor_ecu
```

### Sending commands with send_motors_command.py

A helper script in `scripts/send_motors_command.py` publishes `Twist` commands to `/cmd_vel` and times each maneuver automatically.

```bash
# List all available maneuvers
python3 scripts/send_motors_command.py

# Run a specific maneuver by number
python3 scripts/send_motors_command.py --maneuver 1
```

| # | Description | `linear.x` (m/s) | `angular.z` (rad/s) | Duration |
| --- | --- | --- | --- | --- |
| 1 | Move 300 mm forward  @ 0.70 m/s | +0.70 | 0.0 | 0.429 s |
| 2 | Move 300 mm backward @ 0.70 m/s | −0.70 | 0.0 | 0.429 s |
| 3 | Move 300 mm forward  @ 0.86 m/s | +0.86 | 0.0 | 0.349 s |
| 4 | Move 300 mm backward @ 0.86 m/s | −0.86 | 0.0 | 0.349 s |
| 5 | Move 300 mm forward  @ 0.95 m/s | +0.95 | 0.0 | 0.316 s |
| 6 | Move 300 mm backward @ 0.95 m/s | −0.95 | 0.0 | 0.316 s |
| 7 | Rotate CW  45°  @ 0.75 rad/s | 0.0 | −0.75 | 1.047 s |
| 8 | Rotate CCW 45°  @ 0.75 rad/s | 0.0 | +0.75 | 1.047 s |
| 9 | Rotate CW  360° @ 0.75 rad/s | 0.0 | −0.75 | 8.378 s |
| 10 | Rotate CCW 360° @ 0.75 rad/s | 0.0 | +0.75 | 8.378 s |
| 11 | Arc CW  300 mm / 45° @ 0.86 m/s | +0.86 | −2.251 | 0.349 s |
| 12 | Arc CCW 300 mm / 45° @ 0.86 m/s | +0.86 | +2.251 | 0.349 s |

Arc maneuvers (11, 12) derive angular velocity as `ω = θ · v / s` so the robot turns exactly 45° while travelling 300 mm along the arc.

### Monitoring odometry

```bash
ros2 topic echo /odom
```

## Dependencies

- `rclcpp` / `rclcpp_lifecycle` / `rclcpp_components`
- `pluginlib`
- `nav_msgs`
- `tf2` / `tf2_geometry_msgs`
- `rr_common_base` (GPIO plugin interface and constants)
- `rr_interfaces` (shared message definitions)

### Recommended: GPIO plugin for Raspberry Pi 4B

For hardware deployment on a Raspberry Pi 4B, install `rr_gpio_pi4b_pigpio_plugin`. This plugin implements `rrobots::interfaces::RRGPIOInterface` using the `pigpio` library and is the value to supply for the `transport_plugin` parameter.

It requires `pigpio` (`rrpigpio`), available from the Ryder Robots apt repository. If you have not already added the repository, follow the installation instructions above, then:

```bash
sudo apt install ros-kilted-rr-gpio-pi4b-pigpio-plugin
```

Source: [github.com/Ryder-Robots/rr_gpio_pi4b_pigpio_plugin](https://github.com/Ryder-Robots/rr_gpio_pi4b_pigpio_plugin)

## Roadmap

### PID Duty Conversion

Replace the current linear regression duty convertor with a closed-loop PID algorithm implementing the `DutyConversion` interface. The PID controller will use encoder feedback (measured velocity) and the target velocity to compute duty cycle adjustments each control period. Kp, Ki, and Kd coefficients will be exposed as ROS 2 parameters per motor, allowing runtime tuning to align individual motors to consistent velocity targets and compensate for per-motor mechanical variation.

### Multi-Motor Support

The current implementation is hardcoded for exactly two motors (`std::array<RrMotorController, 2>`). Multi-motor support will generalise this to a `std::vector` sized from the `motor_count` parameter, allowing the ECU to manage three or more driven wheels. The `MotorCmdProc` interface will be updated accordingly, and the `motorcmdproc.hpp` caveat note will be resolved.

### Mecanum Wheel Kinematics

Add a `MecanumCmdProc` implementation of the `MotorCmdProc` interface to support four-wheel mecanum drive. Mecanum kinematics decompose a `Twist` command into independent per-wheel velocities that enable full holonomic motion (forward, lateral, and rotational simultaneously). The drive layout will be selectable at launch time via a parameter, with differential drive remaining the default.

### Raspberry Pi 5 GPIO Plugin (lgpio)

The `pigpio` library used by `rr_gpio_pi4b_pigpio_plugin` is not compatible with the Raspberry Pi 5. A new `rr_gpio_pi5_lgpio_plugin` implementing `rrobots::interfaces::RRGPIOInterface` via `lgpio` (the recommended GPIO library for Pi 5) will provide equivalent functionality for that platform. Once available, the correct plugin is selected at launch time via the `transport_plugin` parameter, keeping the ECU node itself hardware-agnostic.

## License

MIT License - Copyright (c) 2026 Ryder Robots

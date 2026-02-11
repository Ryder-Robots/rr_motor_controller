# rr_motor_controller

Lifecycle-managed ROS 2 node that drives a single DC motor and encoder pair. Each instance controls one motor identified by its position in the `rr_interfaces/msg/Motors` message array.

## Control Loop

1. **subscribe_callback_** receives `Motors` messages on `/motors_command` and stores the target velocity and direction for this motor.
2. **encoder_cb_** runs in GPIO interrupt context, accumulating pulse timing over one full revolution and computing measured velocity via EMA smoothing.
3. **pid_cb_** fires on a 100 ms wall timer, converts target velocity to a PWM duty cycle (currently via linear regression), and applies it to the motor.
4. **publish_callback_** fires on a 200 ms wall timer, publishing velocity and diagnostic counters to `/motors_command<motor_pos>/stats`.

## Lifecycle Transitions

| Transition | What happens |
|---|---|
| **configure** | Loads parameters, computes distance-per-pulse, initialises GPIO plugin, configures motor and encoder hardware |
| **activate** | Sets pin modes, attaches encoder ISR, creates subscription/publisher/timers, starts control loop |
| **deactivate** | Stops timers, tears down ROS interfaces, detaches ISR, sets PWM to 0, resets direction |
| **cleanup** | Releases encoder tick callback |

## Parameters

All parameters are declared in the `RrMotorController` constructor.

### Controller Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `motor_pos` | int | `0` | Index into the `Motors` message array identifying which motor this node controls |
| `ppr` | int | `8` | Pulses per revolution from the encoder |
| `wheel_radius` | int | `20` | Wheel radius in mm, used to compute distance per pulse |
| `transport_plugin` | string | `"rrobots::interfaces::RRGPIOInterface"` | Pluginlib class name for the GPIO transport layer |

### Motor Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `pwm_pin` | int | `-1` | GPIO pin used for hardware PWM output |
| `dir_pin` | int | `-1` | GPIO pin used for motor direction control |
| `pwm_freq` | int | `2000` | PWM frequency in Hz (TC78H660FTG accepts DC to 400 kHz; 500 Hz - 1 kHz typical for small motors) |

### Encoder Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `encoder_pin` | int | `0` | GPIO pin connected to the encoder (hall sensor) output |
| `encoder_timeout` | int | `0` | Interrupt timeout in microseconds; 0 disables timeout callbacks |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/motors_command` | `rr_interfaces/msg/Motors` | Subscribe | Receives target velocity and direction commands |
| `/motors_command<N>/stats` | `rr_interfaces/msg/MotorResponse` | Publish | Publishes velocity, total pulses, healthy pulses, and boundary trigger counts (where `<N>` is `motor_pos`) |

## Example Usage

### Launch file (Python)

```python
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='rr_motor_controller',
            executable='rr_motor_controller_node',
            name='motor_left',
            namespace='',
            parameters=[{
                'motor_pos': 0,
                'ppr': 8,
                'wheel_radius': 33,
                'transport_plugin': 'rrobots::interfaces::RRGPIOInterface',
                'pwm_pin': 12,
                'dir_pin': 24,
                'pwm_freq': 1000,
                'encoder_pin': 17,
                'encoder_timeout': 500000,
            }],
        ),
    ])
```

### Lifecycle transitions via CLI

```bash
# Configure the node (loads parameters, initialises hardware)
ros2 lifecycle set /motor_left configure

# Activate the node (starts control loop)
ros2 lifecycle set /motor_left activate

# Deactivate the node (stops motor, tears down)
ros2 lifecycle set /motor_left deactivate

# Cleanup (releases callbacks)
ros2 lifecycle set /motor_left cleanup
```

### Checking node state

```bash
ros2 lifecycle get /motor_left
```

### Monitoring output

```bash
# Watch velocity and diagnostics
ros2 topic echo /motors_command0/stats
```

## Dependencies

- `rclcpp` / `rclcpp_lifecycle`
- `pluginlib`
- `rr_common_base` (GPIO plugin interface and constants)
- `rr_interfaces` (Motors and MotorResponse message definitions)

## Roadmap

### ECU Node (Twist to Motors)

Add an ECU (Electronic Control Unit) class that subscribes to `geometry_msgs/msg/Twist` commands from the ROS 2 navigation stack and converts them into `rr_interfaces/msg/Motors` command messages for 1 to N motor controllers. The ECU will decompose linear and angular velocity into per-motor target velocities using a differential drive model. Angular velocity is strictly subtractive: turning is achieved by reducing velocity on motors on the inside of the turn, never by increasing velocity on the opposite side. This ensures the vehicle's maximum speed is bounded by the linear velocity command alone.

### PID Duty Conversion

Replace the current linear regression duty convertor with a closed-loop PID algorithm implementing the `DutyConversion` interface. The PID controller will use encoder feedback (measured velocity) and the target velocity to compute duty cycle adjustments each control period. Kp, Ki, and Kd coefficients will be exposed as ROS 2 parameters per motor, allowing runtime tuning to align individual motors to consistent velocity targets and compensate for per-motor mechanical variation.

## License

MIT License - Copyright (c) 2026 Ryder Robots
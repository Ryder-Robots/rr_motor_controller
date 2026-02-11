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

### Controller Parameters

Declared in the `RrMotorController` constructor.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `motor_pos` | int | `0` | Index into the `Motors` message array identifying which motor this node controls |
| `ppr` | int | `8` | Pulses per revolution from the encoder |
| `wheel_radius` | int | `20` | Wheel radius in mm, used to compute distance per pulse |
| `transport_plugin` | string | `"rrobots::interfaces::RRGPIOInterface"` | Pluginlib class name for the GPIO transport layer |

### Motor Parameters

Declared during motor configuration.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `pwm_pin` | int | `-1` | GPIO pin used for hardware PWM output |
| `dir_pin` | int | `-1` | GPIO pin used for motor direction control |
| `pwm_freq` | int | `2000` | PWM frequency in Hz (TC78H660FTG accepts DC to 400 kHz; 500 Hz - 1 kHz typical for small motors) |

### Encoder Parameters

Declared during encoder configuration.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `encoder_pin` | int | `0` | GPIO pin connected to the encoder (hall sensor) output |
| `encoder_timeout` | int | `0` | Interrupt timeout in microseconds; 0 disables timeout callbacks |
| `encoder_min_interval_us` | int | `0` | Minimum interval between valid pulses in microseconds |

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
                'encoder_min_interval_us': 200,
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

## License

MIT License - Copyright (c) 2026 Ryder Robots
// Copyright (c) 2026 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file motor.hpp
 * @author Aaron Spiteri
 * @brief Allows controller to access a specific motor.
 */
#pragma once

#include <algorithm>
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rr_motor_controller
{

/**
 * @class Motor
 * @brief Represents a motor that can be controlled.
 *
 * Note that this class is not a node on its own, it separates responsibilities from the motor controller
 * into specific motor instances.
 */
class Motor
{
public:
  constexpr static int BACKWARD = rrobots::interfaces::RRGPIOInterface::PI_LOW;
  constexpr static int FORWARD = rrobots::interfaces::RRGPIOInterface::PI_HIGH;

  // Duty cycles are a range that are usually offset, thus 50% would be 500,000 the offset corrects for that.
  constexpr static int DUTY_OFFSET = 10000;
  constexpr static int OK = 0;

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Motor() = default;
  ~Motor() = default;

  /**
   * @fn configure
   * @brief Configures the motor, this sets up variables that define the pins, but does not interact with the hardware.
   * This is called during the configure lifecycle transition of the motor controller node.
   *
   * @param previous_state the previous state of the node when this method is called, this is used to determine if the
   * motor should be configured or not, for example if the previous state was unconfigured, then the motor should be
   * configured, but if the previous state was active, then the motor should not be configured.
   * @param node the lifecycle node that owns this motor, this is used to create publishers and subscribers for the
   * motor, and to access the logger for the motor.
   * @param gpio_plugin the GPIO plugin that is used to interact with the hardware, this is used to set up the GPIO pins
   * for the motor, and to set up any interrupts that are required for the motor. Note
   */
  CallbackReturn configure(
    const rclcpp_lifecycle::State & previous_state,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin);

  /**
   * @fn on_activate
   * @brief Activates the motor, this engages the hardware and allows the motor to be
   * controlled. This is called during the activate lifecycle transition of the motor controller node.
   * @param previous_state the previous state of the node when this method is called, this is used to
   * determine if the motor should be activated or not, for example if the previous state was unconfigured, then the
   * motor should not be activated, but if the previous state was configured, then the motor should be activated.
   * @param node the lifecycle node that owns this motor, this is used to create publishers
   * and subscribers for the motor, and to access the logger for the motor.
   * @return CallbackReturn returns status result of method.
   * Note that the node is passed in this method, as some motors may require access to the node during activation, for
   * example to create publishers or subscribers, or to access the logger. This is not ideal, but it is necessary for
   * some motors, and it is the responsibility of the implementer to ensure that the motor does not use the node for any
   * other purpose than what is required for activation.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

  /**
   * @fn on_deactivate
   * @brief Deactivates the motor, this disengages the hardware and prevents the motor
   * from being controlled. This is called during the deactivate lifecycle transition of the motor controller node.
   * @param previous_state the previous state of the node when this method is called, this
   * is used to determine if the motor should be deactivated or not, for example if the previous state was unconfigured,
   * then the motor should not be deactivated, but if the previous state was active, then the motor should be
   * deactivated.
   * @return CallbackReturn returns status result of method.
   * Note that the node is not passed in this method, as the motor should not require access to the node during
   * deactivation, if it does, then it is the responsibility of the implementer to ensure that the motor does not use
   * the node for any other purpose than what is required for deactivation.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  /**
   * @fn set_pwm
   * @brief Sets the PWM duty cycle for the motor, this is used to control the
   * speed of the motor, and the direction of the motor is determined by the sign of the duty cycle, for example if the
   * duty cycle is positive, then the motor will move forward, and if the duty cycle is negative, then the motor will
   * move backward. This is called by the motor controller node when it receives a command to set the PWM duty cycle for
   * the motor.
   * @param duty the duty cycle to set for the motor, this is a value between -100 and 100, where negative values
   * indicate backward movement, positive values indicate forward movement, and 0 indicates that the motor should be
   * stopped.
   * @return int returns status result of method, 0 for success, -1 for failure.
   */
  int set_pwm(int duty);

  /**
   * @fn set_direction
   * @brief Sets the direction of the motor, this is used to control the direction of
   * the motor, and the speed of the motor is determined by the PWM duty cycle, for example if the direction is forward,
   * then the motor will move forward, and if the direction is backward, then the motor
   * will move backward. This is called by the motor controller node when it receives a command to set the direction for
   * the motor.
   * @param dir the direction to set for the motor, this is a value of either
   * Motor::FORWARD or Motor::BACKWARD, where Motor::FORWARD indicates forward movement, and Motor::BACKWARD indicates
   * backward movement.
   * @return int returns status result of method, 0 for success, -1 for
   *    failure.
   * Note that the direction is set separately from the PWM duty cycle, this is because some motor controllers require
   * the direction to be set separately from the PWM duty cycle, for example some motor controllers require a separate
   * pin to be set for the direction, while others require the direction to be determined by the sign of the PWM duty
   * cycle, and it is the responsibility of the implementer to ensure that the motor is configured correctly for the
   * specific motor controller being used.
   *
   */
  int set_direction(int dir);

  /**
   * @fn get_pwm
   * @brief Gets the current PWM duty cycle for the motor, this is used to determine the current speed of the motor, and
   * the direction of the motor is determined by the sign of the duty cycle, for example if the duty cycle is positive,
   * then the motor is moving forward, and if the duty cycle is negative, then the motor is moving backward. This is
   * called by the motor controller node when it needs to determine the current PWM duty cycle for the motor, for
   * example to publish the current state of the motor.
   *
   * @return int returns the current PWM duty cycle for the motor, this is a value between -100 and 100, where
   * negative values indicate backward movement, positive values indicate forward movement, and 0 indicates that the
   * motor is stopped. Note that the direction is determined by the sign of the PWM duty cycle, this is because some
   * motor controllers determine the direction based on the sign of the PWM duty cycle, while others require the
   * direction to be set separately from the PWM duty cycle, and it is the responsibility of the implementer to ensure
   * that the motor is configured correctly for the specific motor controller being used.
   *
   */
  int get_pwm() const noexcept;

  /**
   * @fn get_direction
   * @brief Gets the current direction of the motor, this is used to determine the current direction of the motor, and
   * the speed of the motor is determined by the PWM duty cycle, for example if the direction is forward, then the motor
   * is moving forward, and if the direction is backward, then the motor is moving backward. This is called by the motor
   * controller node when it needs to determine the current direction for the motor, for example to publish the current
   * state of the motor.
   * @return int returns the current direction of the motor, this is a value of either Motor::FORWARD or
   * Motor::BACKWARD, where Motor::FORWARD indicates forward movement, and Motor::BACKWARD indicates backward movement.
   * Note that the direction is determined separately from the PWM duty cycle, this is because some motor controllers
   * determine the direction based on the sign of the PWM duty cycle, while others require the direction to be set
   * separately from the PWM duty cycle, and it is the responsibility of the implementer to ensure that the motor is
   * configured correctly for the specific motor controller being used.
   *
   */
  int get_direction() const noexcept;

private:
  int pwm_pin_;
  int dir_pin_;

  // TC78H660FTG have an adjustable OSCM which means that the frequency can be anything, and since
  //
  // The TC78H660FTG brushed DC motor driver accepts input PWM frequencies from DC up to 400 kHz max. For optimal
  // performance with small DC motors, use 500 Hz to 1 kHz. So going with a frequency of 1000Hz (1kHz), but want to
  // make this adjustable, it could be something that PID algorithm adjusts on the fly.
  // int freq_ = 1000; // hardware real range for PWM pin
  int freq_ = 700;

  // these are stored here for use in the on_activate method, as some motors may require access to the node and gpio
  // plugin during activation, for example to set up publishers or subscribers, or to set up any interrupts that are
  // required for the motor. this is not ideal, but it is necessary for some motors, and it is the responsibility of the
  // implementer to ensure that the motor does not use the node or gpio plugin for any other purpose than what is
  // required for activation. note that these are not shared pointers, as the motor does not own the node or gpio
  // plugin, it is the responsibility of the motor controller node to ensure that the node and gpio plugin are valid for
  // the lifetime of the motor, and it is the responsibility of the implementer to ensure that the motor does not use
  // the node or gpio plugin after they have been destroyed. note that these are not references, as the motor may need
  // to store them for use in the on_activate method, and it is not guaranteed that the node and gpio plugin will be
  // valid for the lifetime of the motor, so it is the responsibility of the implementer to ensure that the motor does
  // not use the node or gpio plugin after they have been destroyed. note that these are not raw pointers, as the motor
  // does not own the node or gpio plugin, and it is the responsibility of the motor controller node to ensure that the
  // node and gpio plugin are valid for the lifetime of the motor, so it is the responsibility of the implementer to
  // ensure that the motor does not use the node or gpio plugin after they have been destroyed. note that these are not
  // weak pointers, as the motor does not own the node or gpio plugin, and it is the responsibility of the motor
  // controller node to ensure that the node and gpio plugin are valid for the lifetime of the motor, so it is the
  // responsibility of the implementer to ensure that the motor does not use the node or gpio plugin after they have
  // been destroyed. note that these are not unique pointers, as the motor does not own the node or gpio plugin, and it
  // is the responsibility of the motor controller node to ensure that the node and gpio plugin are valid for the
  // lifetime of the motor, so it is the responsibility of the implementer to ensure that the motor does not use the
  // node or gpio plugin after they have been destroyed.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin_;
};
}  // namespace rr_motor_controller

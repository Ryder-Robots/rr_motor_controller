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
  constexpr static int BACKWARD = rrobots::interfaces::RRGPIOInterface::RRGPIO_LOW;
  constexpr static int FORWARD = rrobots::interfaces::RRGPIOInterface::RRGPIO_HIGH;

  // Duty cycles are a range that are usually offset, thus 50% would be 500,000 the offset corrects for that.
  constexpr static int DUTY_OFFSET = 10000;
  constexpr static int OK = 0;

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Motor() = default;
  ~Motor() = default;

  /**
   * @fn configure
   * @brief Configures the motor by reading pin parameters. Does not interact with hardware.
   *
   * Called during the configure lifecycle transition of the motor controller node.
   *
   * @param previous_state Lifecycle state prior to this transition.
   * @param node The lifecycle node that owns this motor (used for parameter access and logging).
   * @param gpio_plugin GPIO transport used for hardware pin validation and later activation.
   */
  CallbackReturn configure(const rclcpp_lifecycle::State& previous_state,
                           rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                           std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin);

  /**
   * @fn on_activate
   * @brief Activates the motor hardware (pin modes, initial direction and PWM).
   *
   * Called during the activate lifecycle transition.
   *
   * @param previous_state Lifecycle state prior to this transition.
   * @return CallbackReturn SUCCESS or FAILURE.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  /**
   * @fn on_deactivate
   * @brief Deactivates the motor by zeroing PWM and resetting direction.
   *
   * Called during the deactivate lifecycle transition.
   *
   * @param previous_state Lifecycle state prior to this transition.
   * @return CallbackReturn SUCCESS or FAILURE.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

  /**
   * @fn set_pwm
   * @brief Set the PWM duty cycle (motor speed).
   * @param duty Duty percentage (0-100). Scaled by DUTY_OFFSET before writing to hardware.
   * @return 0 on success, non-zero on failure.
   */
  int set_pwm(int duty);

  /**
   * @fn set_direction
   * @brief Set the motor direction via the direction GPIO pin.
   * @param dir Motor::FORWARD or Motor::BACKWARD.
   * @return 0 on success, non-zero on failure.
   */
  int set_direction(int dir);

  /**
   * @fn get_pwm
   * @brief Read the current PWM duty cycle from hardware.
   * @return Duty percentage (0-100).
   */
  int get_pwm() const noexcept;

  /**
   * @fn get_direction
   * @brief Read the current direction from the direction GPIO pin.
   * @return Motor::FORWARD or Motor::BACKWARD.
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

  // Stored during configure() for use in on_activate(). Motor does not own these —
  // the parent motor controller node must keep them valid for the motor's lifetime.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin_;
};
}  // namespace rr_motor_controller

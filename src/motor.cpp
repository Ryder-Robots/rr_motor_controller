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

#include "rr_motor_controller/motor.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace rr_motor_controller
{
CallbackReturn Motor::configure(
  const rclcpp_lifecycle::State & previous_state,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin)
{
  (void)previous_state;
  RCLCPP_INFO(rclcpp::get_logger("Motor"), "Configuring motor...");

  if (!(gpio_plugin && node)) {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "missing node or GPIO plugin");
    return CallbackReturn::FAILURE;
  }

  if (!(node->get_parameter("pwm_pin", pwm_pin_) && node->get_parameter("dir_pin", dir_pin_) &&
    node->get_parameter("pwm_freq", freq_)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Failed to get parameters for motor configuration");
    return CallbackReturn::FAILURE;
  }

  gpio_plugin_ = gpio_plugin;
  node_ = node;

  // verify parameters are within acceptable ranges.
  const auto & pwm_pins = gpio_plugin->get_pwm_pins();
  if (std::find(pwm_pins.begin(), pwm_pins.end(), pwm_pin_) == pwm_pins.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Invalid PWM pin %d", pwm_pin_);
    return CallbackReturn::FAILURE;
  }

  if (dir_pin_ == pwm_pin_) {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "direction pin and PWM can not be both %d", pwm_pin_);
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Motor::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(rclcpp::get_logger("Motor"), "Activating motor...");

  if (gpio_plugin_->set_pin_mode(dir_pin_, rrobots::interfaces::RRGPIOInterface::PI_OUTPUT) != OK) {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Failed to set direction pin mode");
    return CallbackReturn::FAILURE;
  }

  if (gpio_plugin_->set_pin_mode(pwm_pin_, rrobots::interfaces::RRGPIOInterface::PI_ALT5) != OK) {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Failed to set pwm pin mode");
    return CallbackReturn::FAILURE;
  }

  if (set_direction(FORWARD) != OK) {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Failed to set initial direction");
    return CallbackReturn::FAILURE;
  }

  if (set_pwm(0) != OK) {
    RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Failed to set initial pwm");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

int Motor::set_direction(int dir)
{
  return gpio_plugin_->digital_write(dir_pin_, dir);
}

int Motor::set_pwm(int duty)
{
  return gpio_plugin_->gpio_hardware_pwm(pwm_pin_, freq_, duty + DUTY_OFFSET);
}

int Motor::get_pwm() const noexcept
{
  return gpio_plugin_->gpio_hardware_get_pwm(pwm_pin_) - DUTY_OFFSET;
}

int Motor::get_direction() const noexcept
{
  return gpio_plugin_->digital_read(dir_pin_);
}

CallbackReturn Motor::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(rclcpp::get_logger("Motor"), "Deactivating motor...");
  CallbackReturn exit_res = CallbackReturn::SUCCESS;
  if (set_pwm(0) != OK) {
    exit_res = CallbackReturn::FAILURE;
  }

  if (set_direction(FORWARD) != OK) {
    exit_res = CallbackReturn::FAILURE;
  }

  return exit_res;
}

}  // namespace rr_motor_controller
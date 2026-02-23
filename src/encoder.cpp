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

#include "rr_motor_controller/encoder.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace rr_motor_controller
{
CallbackReturn MotorEncoder::configure(const rclcpp_lifecycle::State& previous_state,
                                       rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                       std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin,
                                       EncoderTickCallback tick_cb, int mpos)
{
  (void)previous_state;
  RCLCPP_INFO(rclcpp::get_logger("MotorEncoder"), "Configuring encoder...");

  {
    // int64_t pin{ -1 };
    // int64_t timeout{ 0 };

    if (!(node->has_parameter("encoder_pins") && node->has_parameter("encoder_timeout")))
    {
      RCLCPP_ERROR(rclcpp::get_logger("MotorEncoder"), "pin, timeout, and min_interval are required parameters...");
      return CallbackReturn::FAILURE;
    }

    pin_ = static_cast<int>(node->get_parameter("encoder_pins").as_integer_array().at(mpos));
    timeout_ = static_cast<int>(node->get_parameter("encoder_timeout").as_integer_array().at(mpos));
  }

  if (!tick_cb)
  {
    return CallbackReturn::FAILURE;
  }
  tick_cb_ = tick_cb;
  gpio_plugin_ = gpio_plugin;
  node_ = node;
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorEncoder::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;

  if (tick_cb_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotorEncoder"), "pin encoder callback was not defined...");
    return CallbackReturn::FAILURE;
  }

  if (pin_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotorEncoder"), "pin can't be a negative value...");
    return CallbackReturn::FAILURE;
  }

  if (gpio_plugin_->set_pin_mode(pin_, rrobots::interfaces::RRGPIOInterface::RRGPIO_INPUT) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotorEncoder"), "could not set pin %d to input...", pin_);
    return CallbackReturn::FAILURE;
  }

  if (gpio_plugin_->set_pull_up_down(pin_, rrobots::interfaces::RRGPIOInterface::RRGPIO_PUD_DOWN) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotorEncoder"), "could not pull pin %d to PUD_DOWN...", pin_);
    return CallbackReturn::FAILURE;
  }
  last_tick_ = gpio_plugin_->tick();
  if (gpio_plugin_->set_isr_func_ex(pin_, rrobots::interfaces::RRGPIOInterface::RRGPIO_RISING_EDGE, timeout_,
                                    &MotorEncoder::gpio_isr_func, this) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotorEncoder"), "could not attach ISR callback to pin %d...", pin_);
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorEncoder::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  CallbackReturn rv = CallbackReturn::SUCCESS;
  if (gpio_plugin_->clear_isr_func(pin_))
  {
    RCLCPP_WARN(rclcpp::get_logger("MotorEncoder"), "unable to clear pin %d ISR callback function...", pin_);
    rv = CallbackReturn::FAILURE;
  }
  if (gpio_plugin_->set_pull_up_down(pin_, rrobots::interfaces::RRGPIOInterface::RRGPIO_PUD_OFF) != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("MotorEncoder"), "could not pull pin %d to PUD_OFF...", pin_);
    rv = CallbackReturn::FAILURE;
  }
  if (gpio_plugin_->set_pin_mode(pin_, rrobots::interfaces::RRGPIOInterface::RRGPIO_INPUT))
  {
    RCLCPP_WARN(rclcpp::get_logger("MotorEncoder"), "could not set the pin %d back to INPUT...", pin_);
    rv = CallbackReturn::FAILURE;
  }

  return rv;
}

// Static wrapper - required for C function pointer compatibility
void MotorEncoder::gpio_isr_func(int gpio, int level, uint32_t tick, void* userdata)
{
  auto* self = static_cast<MotorEncoder*>(userdata);
  self->handle_interrupt(gpio, level, tick);
}

void MotorEncoder::handle_interrupt(int gpio, int level, uint32_t tick)
{
  /*
   0 = change to low (a falling edge)
   1 = change to high (a rising edge)
   2 = no level change (interrupt timeout)
  */

  TickStatus status = TickStatus::HEALTHY;
  if (level != expected_level_)
  {
    status = TickStatus::NOISE_REJECTED;
    if (level == 2)
    {
      status = TickStatus::TIMEOUT;
    }
  }
  uint32_t delta_us = tick - last_tick_;
  last_tick_ = tick;
  tick_cb_(gpio, delta_us, tick, status);
}

}  // namespace rr_motor_controller
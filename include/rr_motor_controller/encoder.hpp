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

#pragma once

#include <functional>
#include <cstdint>
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rr_motor_controller
{

/**
 * Encoder event status codes
 */
enum class TickStatus : uint8_t
{
  HEALTHY = 0,         // Valid rising edge detected
  TIMEOUT = 1,         // No edge within configured timeout period
  NOISE_REJECTED = 2,  // Edge rejected (interval too short, likely electrical noise)
  UNEXPECTED = 3,      // Condition occurred that was unexpected, this should be treated immeidate termination.
};

/**
 * Callback invoked by MotorEncoder on each hardware event (pulse or timeout).
 *
 * Executed in interrupt context - keep processing minimal and avoid blocking operations.
 *
 * @param gpio_pin  GPIO pin that sample is taken from.
 * @param delta_us Time elapsed since the last valid pulse in microseconds.
 *                 For OK status: time between valid pulses (use for velocity calculation)
 *                 For TIMEOUT status: time since last valid pulse to timeout
 *                 For NOISE_REJECTED status: the rejected (too-short) interval
 *
 * @param tick   Current tick.
 *
 * @param tick_status Event status indicating the nature of this callback:
 *                    - TickStatus::HEALTHY: Valid rising edge detected on encoder phase
 *                    - TickStatus::TIMEOUT: No pulse received within configured timeout period
 *                    - TickStatus::NOISE_REJECTED: Pulse rejected (interval shorter than physical limits)
 *
 * Note: The encoder reports all events neutrally. Application logic must interpret
 * whether a timeout represents a fault condition based on expected motion state.
 */
using EncoderTickCallback = std::function<void(int gpio_pin, uint32_t delta_us, uint32_t tick, TickStatus tick_status)>;

/**
 * Controls motor encoder, this will uses a sensor such as a hall sensor to measure rotations of
 * motor.
 */
class MotorEncoder
{
public:
  MotorEncoder() = default;
  ~MotorEncoder() = default;

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn configure(const rclcpp_lifecycle::State& previous_state,
                           rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                           std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin,
                           EncoderTickCallback tick_cb);

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

private:
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

  /**
   * Called after each pulse. This method will trigger tick_cb which will trigger handler.
   */
  static void gpio_isr_func(int gpio, int level, uint32_t tick, void* userdata);

  void handle_interrupt(int gpio, int level, uint32_t tick);

  // last tick, this should be set during configuration for initial tick.
  uint32_t last_tick_{ 0 };
  int pin_{ -1 };
  int timeout_{ 0 };
  EncoderTickCallback tick_cb_{ nullptr };
  uint32_t min_interval_us_{ 0 };

  int expected_level_ = rrobots::interfaces::RRGPIOInterface::RISING_EDGE;
};
}  // namespace rr_motor_controller
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
  UNEXPECTED = 3,      // Condition occurred that was unexpected, this should be treated as immediate termination.
};

/**
 * Callback invoked by MotorEncoder on each hardware event (pulse or timeout).
 *
 * Executed in interrupt context - keep processing minimal and avoid blocking operations.
 *
 * @param gpio_pin  GPIO pin that sample is taken from.
 * @param delta_us Time elapsed since the last valid pulse in microseconds.
 *                 For HEALTHY status: time between valid pulses (use for velocity calculation)
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
 * Controls motor encoder, uses a sensor such as a hall sensor to measure rotations of motor.
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
  // Shared pointers to the node and GPIO plugin, stored during configure() for use in on_activate()
  // and on_deactivate(). The encoder does not own these — the motor controller node is responsible
  // for ensuring they remain valid for the encoder's lifetime.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin_;

  /**
   * Static ISR wrapper for C function pointer compatibility. Delegates to handle_interrupt(),
   * which computes delta_us and invokes tick_cb_.
   */
  static void gpio_isr_func(int gpio, int level, uint32_t tick, void* userdata);

  void handle_interrupt(int gpio, int level, uint32_t tick);

  // last tick, set during on_activate() to seed the initial delta_us calculation.
  uint32_t last_tick_{ 0 };
  int pin_{ -1 };
  int timeout_{ 0 };
  EncoderTickCallback tick_cb_{ nullptr };

  int expected_level_ = rrobots::interfaces::RRGPIOInterface::RISING_EDGE;
};
}  // namespace rr_motor_controller
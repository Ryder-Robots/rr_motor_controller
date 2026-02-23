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

// Lifecycle-managed motor controller node. Each instance drives a single motor/encoder pair
// identified by motor_pos_ within the Motors message array.
//
// Control loop:
//   encoder_cb_ measures actual velocity_ from encoder pulse timing (called per interrupt).
//   pid_cb_ runs on a wall timer, computes duty from target vs actual, and sets PWM.
//
// Currently uses linear regression for duty conversion; a PID implementation can be
// swapped in via the DutyConversion interface once empirical tuning data is available.
#pragma once
#include <string>
#include <cmath>
#include <pthread.h>
#include <sys/timerfd.h>
#include <pluginlib/class_loader.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rr_motor_controller/visibility_control.h"
#include "rr_motor_controller/motor.hpp"
#include "rr_motor_controller/encoder.hpp"
#include "rr_motor_controller/dutyconv.hpp"
#include "rr_motor_controller/dc_linear.hpp"
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rr_interfaces/msg/motor_response.hpp"
#include "rr_interfaces/msg/motors.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rr_motor_controller
{

class RrMotorController
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;
  using RRGPIOInterface = rrobots::interfaces::RRGPIOInterface;

public:
  RrMotorController()
  {
  }

  ~RrMotorController() = default;

  /**
   * @brief Loads parameters (motor_pos, ppr, wheel_radius), computes dpp_,
   * associates GPIO plugin to encoder and motor, and configures motor and encoder hardware.
   */
  CallbackReturn on_configure(const State& state, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int mpos,
                              std::shared_ptr<RRGPIOInterface> gpio_plugin);

  /**
   * @brief Activates motor and encoder.
   */
  CallbackReturn on_activate(const State& state);

  /**
   * @brief Stops the control loop, releases ROS interfaces, and deactivates
   * encoder and motor hardware (in that order).
   */
  CallbackReturn on_deactivate(const State& state);

  CallbackReturn on_cleanup(const State& state);

protected:
  /**
   * @brief Encoder interrupt handler. Accumulates pulse timing over one full
   * revolution (ppr_ pulses), then computes velocity via EMA smoothing.
   * Called from the GPIO interrupt context — must be lock-free.
   */
  void encoder_cb_(const int gpio_pin, const uint32_t delta_us, const uint32_t tick, const TickStatus tick_status);

  /**
   * @brief Subscription handler for Motors messages. Stores the requested
   * target velocity and direction for the motor at motor_pos_.
   */
  void process_cmd(const rr_interfaces::msg::Motors& req);

  /**
   * @brief Wall timer callback. Computes duty cycle from target_velocity_
   * and velocity_ via the DutyConversion strategy, then applies it to the motor.
   */
  void pid_cb_();

private:
  // -- Configuration (set during on_configure, immutable after) --
  int motor_pos_ = -1;         // index into Motors message array for this controller
  int64_t wheel_radius_{ 0 };  // wheel radius in mm
  double dpp_{ 0 };            // distance per pulse in mm: (2 * pi * wheel_radius_) / ppr_
  int ppr_{ 8 };               // pulses per revolution from encoder

  // -- Hardware --
  Motor motor_;
  MotorEncoder encoder_;
  std::shared_ptr<rr_motor_controller::DutyConversion> duty_conv_;
  std::shared_ptr<RRGPIOInterface> gpio_plugin_;
  EncoderTickCallback tick_cb_{ nullptr };

  // -- Control loop state (atomic: written by encoder ISR, read by pid_cb_) --
  std::atomic<double> velocity_{ 0 };         // measured velocity in m/s (from encoder)
  std::atomic<double> target_velocity_{ 0 };  // desired velocity in m/s (from subscriber)
  std::atomic<int> direction_{ Motor::FORWARD };

  // -- Diagnostics (published to ECU via publish_callback_) --
  std::atomic<int> total_pulses_{ 0 };       // all encoder edges received
  std::atomic<int> healthy_pulses_{ 0 };     // edges within valid timing window
  std::atomic<int> boundary_triggers_{ 0 };  // full-revolution boundaries hit

  std::atomic<bool> running_{ false };  // gate for encoder_cb_ processing

  // -- Encoder accumulation (reset each full revolution) --
  std::atomic<int> delta_ct_{ 0 };             // pulse count toward next revolution boundary
  std::atomic<int> delta_us_ct_{ 0 };          // healthy pulse count within current revolution
  std::atomic<uint64_t> delta_us_accum_{ 0 };  // sum of healthy delta_us in current revolution

  // -- Constants --
  constexpr static int64_t PID_TIMER_DELTA{ 100 };  // PID timer period in ms
  constexpr static uint32_t MIN_DELTA_US{ 300 };    // encoder timing lower bound (us)
  constexpr static uint32_t MAX_DELTA_US{ 3000 };   // encoder timing upper bound (us)

  // -- ROS interfaces (created in on_activate, released in on_deactivate) --
  rclcpp_lifecycle::LifecyclePublisher<rr_interfaces::msg::MotorResponse>::SharedPtr publisher_{ nullptr };
  rclcpp::Subscription<rr_interfaces::msg::Motors>::SharedPtr subscription_{ nullptr };
  std::thread pid_timer_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

}  // namespace rr_motor_controller

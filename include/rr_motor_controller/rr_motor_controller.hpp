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

// CAVEAT: At this stage PID algorithm is not included.  This is because more imperical evidence
// is needed to know how this algorithm shoudl work in real world environments.
#pragma once
#include <mutex>
#include <thread>
#include <string>
#include <cmath>
#include <pluginlib/class_loader.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rr_motor_controller/visibility_control.h"
#include "rr_motor_controller/motor.hpp"
#include "rr_motor_controller/encoder.hpp"
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rr_interfaces/msg/motor_response.hpp"
#include "rr_interfaces/msg/motors.hpp"
#include "rr_common_base/rr_constants.hpp"

namespace rr_motor_controller
{

class RrMotorController : rclcpp_lifecycle::LifecycleNode
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;

public:
  RrMotorController() = default;

  virtual ~RrMotorController() = default;

  CallbackReturn on_configure(const State& state) override;

  CallbackReturn on_activate(const State& state) override;

  CallbackReturn on_deactivate(const State& state) override;

  CallbackReturn on_cleanup(const State& state) override;

protected:
  void encoder_cb_(const int gpio_pin, const uint32_t delta_us, const uint32_t tick, const TickStatus tick_status);

  void publish_callback_();
  void subscribe_callback_(const rr_interfaces::msg::Motors & req);

private:
  int motor_pos_ = -1;  // position of motor within Motors message that this controller must listen for.
  uint8_t wheel_radius_{0}; // radius in mm.
  double dpp_ {0}; // distance per pulse, how much distance the each pulse moved in mm
  Motor motor_;
  MotorEncoder encoder_;
  std::shared_ptr<rrobots::interfaces::RRGPIOInterface> gpio_plugin_;
  EncoderTickCallback tick_cb_{ nullptr };
  int ppr_{ 8 };

  // output and diagnostic variables are publsihed to ECU.
  // output variables
  std::atomic<double> velocity_{ 0 };  // velocity per rotation.

  // diagnoses variables
  std::atomic<int> total_pulses_{ 0 };
  std::atomic<int> healthy_pulses_{ 0 };
  std::atomic<int> boundary_triggers_{ 0 };

  // true if running.
  std::atomic<bool> running_{ false };

  // state variables
  std::atomic<int> delta_ct_{ 0 };     // count for each delta that has arrive (regardless of its in range or not)
  std::atomic<int> delta_us_ct_{ 0 };  // count of healthy delta ticks.
  std::atomic<uint64_t> delta_us_accum_{ 0 };  // accumulate deltas

  constexpr static double MIN_DELTA_US{ 300 };
  constexpr static double MAX_DELTA_US{ 3000 };
  const std::string CMD_TOPIC = rr_constants::TOPIC_MOTOR;

  rclcpp_lifecycle::LifecyclePublisher<rr_interfaces::msg::MotorResponse>::SharedPtr publisher_ {nullptr};
  rclcpp::Subscription<rr_interfaces::msg::Motors>::SharedPtr subscriber_ {nullptr};
};

}  // namespace rr_motor_controller

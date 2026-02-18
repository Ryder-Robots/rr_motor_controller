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
#include "rr_motor_controller/visibility_control.h"
#include <pluginlib/class_loader.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rr_motor_controller/rr_motor_controller.hpp"
#include "rr_motor_controller/rr_motor_controller_common.hpp"
#include "rr_motor_controller/motorcmdproc.hpp"
#include <vector>
#include <deque>

namespace rr_motor_controller
{
/**
 * @class RrECU
 * @brief Electronic Control Unit for Ryder Robots motor subsystem (NOT YET IMPLEMENTED).
 *
 * Intended to coordinate multiple RrMotorController instances by accepting
 * high-level velocity commands and dispatching them to individual motors
 * via per-motor command queues.
 *
 * Motor index convention (2-motor differential drive):
 *   0 — LEFT
 *   1 — RIGHT
 *
 * @note Currently only a 2-motor differential drive layout is planned.
 * @note This class is a draft header — no implementation (.cpp) exists yet.
 */
class RrECU : public rclcpp_lifecycle::LifecycleNode
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;
  using RRGPIOInterface = rrobots::interfaces::RRGPIOInterface;

public:

  /**
   * @brief Construct the ECU node and declare parameters.
   *
   * Pin arrays must be ordered by motor index — encoder_pins[i], pwm_pins[i],
   * and dir_pins[i] must all refer to the same physical motor.
   *
   * @param options Node options forwarded from the composable node container.
   */
  explicit RrECU(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("RrECU", options)
  {
    declare_parameter("encoder_pins", std::vector<int64_t>{});  // one encoder pin per motor
    declare_parameter("pwm_pins",  std::vector<int64_t>{});     // one PWM pin per motor
    declare_parameter("dir_pins", std::vector<int64_t>{});      // one direction pin per motor

    declare_parameter("motor_count", 0);        // number of motors to manage
    declare_parameter("ppr", 8);                // pulses per revolution (shared across motors)
    declare_parameter("wheel_radius", 20);      // wheel radius in mm (assumed uniform)
    declare_parameter("wheel_base", 74);        // distance between left and right side wheels in mm

    // Board-specific GPIO transport plugin (pluginlib class name).
    declare_parameter("transport_plugin", "rrobots::interfaces::RRGPIOInterface");
  }

  ~RrECU() = default;

  CallbackReturn on_configure(const State& state) override;
  CallbackReturn on_activate(const State& state) override;
  CallbackReturn on_deactivate(const State& state) override;
  CallbackReturn on_cleanup(const State& state) override;

protected:
  /**
   * @brief Subscription callback for geometry_msgs::msg::Twist.
   *
   * Converts linear/angular velocity into per-motor commands and
   * enqueues them into command_queue_ for processing.
   */
  void subscribe_callback_(const geometry_msgs::msg::Twist& req);
  void publish_callback_();

private:
  // Motor controllers, one per physical motor. Indexed by motor position.
  std::vector<RrMotorController> motors_;

  std::unique_ptr<MotorCmdProc> mt_cmd_proc_ {nullptr};

  // Per-motor command queues. command_queue_[i] feeds motors_[i].
  // Commands whose ttl_ns has expired are discarded before processing.
  std::vector<std::deque<MotorCommand>> command_queue_;

  std::unique_ptr<pluginlib::ClassLoader<RRGPIOInterface>> poly_loader_{ nullptr };  ///< GPIO plugin loader.
  std::shared_ptr<RRGPIOInterface> gpio_plugin_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_{ nullptr };  ///< Twist command input.
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr publisher_{ nullptr };  ///< Aggregated motor status output.
};

}  // namespace rr_motor_controller
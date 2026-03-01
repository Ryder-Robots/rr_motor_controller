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
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rr_motor_controller/rr_motor_controller.hpp"
#include "rr_motor_controller/rr_motor_controller_common.hpp"
#include "rr_motor_controller/motorcmdproc.hpp"
#include <algorithm>

// TODO: this can be deprecated if different robots become a plugin.
#include "rr_motor_controller/differential.hpp"

#include <vector>
#include <array>
#include <mutex> 

namespace rr_motor_controller
{
/**
 * @class RrECU
 * @brief Electronic Control Unit for Ryder Robots motor subsystem.
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
 */
class RrECU : public nav2_util::LifecycleNode
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
  explicit RrECU(const rclcpp::NodeOptions& options) : nav2_util::LifecycleNode("RrECU", "driver", options)
  {
    declare_parameter("encoder_pins", std::vector<int64_t>{});  // one encoder pin per motor
    declare_parameter("pwm_pins", std::vector<int64_t>{});      // one PWM pin per motor
    declare_parameter("dir_pins", std::vector<int64_t>{});      // one direction pin per motor

    declare_parameter("motor_count", 0);    // number of motors to manage
    declare_parameter("ppr", 8);            // pulses per revolution (shared across motors)
    declare_parameter("wheel_radius", 20);  // wheel radius in mm (assumed uniform)
    declare_parameter("wheel_base", 74);    // distance between left and right side wheels in mm
    declare_parameter("ttl_ns", 200'000'000);      // time to live, the expiry of each command recieved in subscription.
    declare_parameter("covariance", std::vector<double>());  // adjustments for noise with six degrees of freedom (x, y,
                                                             // z, roll, pitch, and yaw)
    declare_parameter("pwm_freq", 2000);

    // Board-specific GPIO transport plugin (pluginlib class name).
    declare_parameter("transport_plugin", "rrobots::interfaces::RRGPIOInterface");
    declare_parameter("encoder_timeout",
                      0);  //  timeout in milliseconds for the ISR to be called after the edge is detected, if the edge
                           //  is not detected within the timeout period, the ISR will be called with level = -1. If
                           //  timeout is 0, the ISR will be called immediately after the edge is detected.
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
   * Converts linear/angular velocity into per-motor commands.
   */
  void subscribe_callback_(const geometry_msgs::msg::Twist& req);

  /**
   * @brief Publish callback for nav_msgs::msg::Odometry.
   * 
   * Publishes linear/angular velocity, postulate x,y, z and 
   * covariance.
   */
  void publish_callback_();

private:
  // Motor controllers, one per physical motor. Indexed by motor position.
  // allocate memory to stack, since these will not change.
  std::array<RrMotorController, 2> motors_;
  int motor_count_ = 0;

  std::unique_ptr<MotorCmdProc> mt_cmd_proc_{ nullptr };

  std::unique_ptr<pluginlib::ClassLoader<RRGPIOInterface>> poly_loader_{ nullptr };  ///< GPIO plugin loader.
  std::shared_ptr<RRGPIOInterface> gpio_plugin_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_{ nullptr };  ///< Twist command input.
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr publisher_{ nullptr };  ///< Aggregated motor
                                                                                                   ///< status output.
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex motor_mutex_;
};

}  // namespace rr_motor_controller
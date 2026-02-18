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

#include "rr_motor_controller/differential.hpp"

namespace rr_motor_controller
{

void DifferentalCmdProc::on_configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
  wheel_base_ = node->get_parameter_or("wheel_base", 0);
  ttl_ns_ = node->get_parameter_or("ttl_ns", 0);
  node_ = node;
}

MotorCommand DifferentalCmdProc::make_cmd(double velocity)
{
  MotorCommand cmd;
  cmd.velocity = static_cast<float>(std::abs(velocity));
  cmd.direction = (velocity >= 0.0) ? 1 : -1;  // Motor::FORWARD / BACKWARD
  cmd.ttl_ns = static_cast<uint64_t>(node_->now().nanoseconds()) + ttl_ns_;
  return cmd;
}

std::vector<MotorCommand> DifferentalCmdProc::proc_twist(geometry_msgs::msg::Twist twist)
{
  std::vector<MotorCommand> cmd = {};
  double v_left = twist.linear.x - (twist.angular.z * wheel_base_ / 2.0);
  double v_right = twist.linear.x + (twist.angular.z * wheel_base_ / 2.0);

  cmd.push_back(make_cmd(v_left));
  cmd.push_back(make_cmd(v_right));
  return cmd;
}

nav_msgs::msg::Odometry DifferentalCmdProc::proc_odom(const std::vector<MotorCommand>)
{
  rclcpp::Time stamp(static_cast<int64_t>(static_cast<uint64_t>(node_->now().nanoseconds())), RCL_STEADY_TIME);
  tf2::Quaternion q;
}
}  // namespace rr_motor_controller
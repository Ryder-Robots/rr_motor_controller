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

void DifferentialCmdProc::on_configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
  wheel_base_ = node->get_parameter_or("wheel_base", 0);
  ttl_ns_ = node->get_parameter_or("ttl_ns", 0);

  // Diagonal covariance (default it to 0)
  covariance_.fill(0);
  if (node->has_parameter("covariance"))
  {
    auto v = node->get_parameter("covariance").as_double_array();
    std::copy_n(v.begin(), std::min(v.size(), covariance_.size()), covariance_.begin());
  }

  // initlize command_history_
  command_history_.resize(2);
  node_ = node;
  is_configured = true;
}

double DifferentialCmdProc::compute_distance(const std::vector<rr_motor_controller::MotorCommand>& history,
                                             uint64_t now_ns)
{
  if (history.empty())
    return 0.0;

  double distance = 0.0;

  for (size_t i = 0; i < history.size(); ++i)
  {
    const auto& cmd = history[i];

    // Discard commands that had already expired before they were issued
    // (shouldn't happen in practice, but guard anyway)
    if (i > 0 && cmd.ttl_ns <= history[i - 1].ttl_ns)
      continue;

    // Start time of this command segment
    uint64_t t_start = (i == 0) ? last_update_ns_ : history[i - 1].ttl_ns;

    // End time: either the next command's TTL, or now if this is the last
    uint64_t t_end = (i + 1 < history.size()) ? cmd.ttl_ns : now_ns;

    // Clamp: don't integrate beyond now
    if (t_start >= now_ns)
      break;
    t_end = std::min(t_end, now_ns);

    double dt = static_cast<double>(t_end - t_start) * 1e-9;  // ns -> s
    double signed_velocity = cmd.velocity * ((cmd.direction == 0) ? -1.0 : 1.0);
    distance += signed_velocity * dt;
  }

  return distance;
}

uint64_t DifferentialCmdProc::update()
{
  uint64_t now_ns = static_cast<uint64_t>(node_->now().nanoseconds());

  if (last_update_ns_ == 0)
  {
    last_update_ns_ = now_ns;
    return now_ns;
  }

  // --- Integrate wheel distances since last update ---
  double d_left = compute_distance(command_history_[DD_LEFT], now_ns);
  double d_right = compute_distance(command_history_[DD_RIGHT], now_ns);

  // --- Differential drive kinematics ---
  double d_centre = (d_left + d_right) / 2.0;
  double d_theta = (d_right - d_left) / wheel_base_;

  // Update postulate
  x_ += d_centre * std::cos(theta_);
  y_ += d_centre * std::sin(theta_);
  theta_ += d_theta;

  // --- Velocities (for twist) ---
  dt_ = static_cast<double>(now_ns - last_update_ns_) * 1e-9;
  v_linear_ = (dt_ > 0.0) ? d_centre / dt_ : 0.0;
  v_angular_ = (dt_ > 0.0) ? d_theta / dt_ : 0.0;

  last_update_ns_ = now_ns;
  prune_history(now_ns);
  return now_ns;
}

MotorCommand DifferentialCmdProc::make_cmd(double velocity)
{
  MotorCommand cmd;
  cmd.velocity = static_cast<float>(std::abs(velocity));
  cmd.direction = (velocity >= 0.0) ? true : false;  // Motor::FORWARD / BACKWARD
  cmd.ttl_ns = static_cast<uint64_t>(node_->now().nanoseconds()) + ttl_ns_;
  return cmd;
}

std::array<MotorCommand,2 > DifferentialCmdProc::proc_twist(geometry_msgs::msg::Twist twist)
{
  std::array<MotorCommand, 2> cmd = {};
  if (!is_configured)
    return cmd;
  double v_left = twist.linear.x - (twist.angular.z * wheel_base_ / 2.0);
  double v_right = twist.linear.x + (twist.angular.z * wheel_base_ / 2.0);

  cmd[DD_LEFT] = make_cmd(v_left);
  cmd[DD_RIGHT] = make_cmd(v_right);

  command_history_[DD_LEFT].push_back(cmd[0]);
  command_history_[DD_RIGHT].push_back(cmd[1]);

  return cmd;
}

nav_msgs::msg::Odometry DifferentialCmdProc::proc_odom()
{
  nav_msgs::msg::Odometry odom;
  if (!is_configured)
    return odom;
  uint64_t now_ns = update();
  rclcpp::Time stamp(static_cast<int64_t>(now_ns), RCL_STEADY_TIME);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);

  // Odometry message
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  // set posulate
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf2::toMsg(q);

  // Diagonal covariance — tune to your encoder noise
  std::copy(covariance_.begin(), covariance_.end(), odom.pose.covariance.begin());

  odom.twist.twist.linear.x = v_linear_;
  odom.twist.twist.angular.z = v_angular_;

  return odom;
}

// This may could a performance hit, on ARM. If performance is affected, run perf, and see
// there is a way this can be optimised.
void DifferentialCmdProc::prune_history(uint64_t now_ns)
{
  for (auto& history : command_history_)
  {
    // Keep at least one command (the last active one)
    while (history.size() > 1 && history[1].ttl_ns <= now_ns)
    {
      history.erase(history.begin());
    }
  }
}
}  // namespace rr_motor_controller
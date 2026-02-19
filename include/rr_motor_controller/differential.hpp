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
#include "rr_motor_controller/motorcmdproc.hpp"
#include "rr_motor_controller/visibility_control.h"
#include <vector>
#include <array>
#include "rr_motor_controller/rr_motor_controller_common.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @file differential.hpp
 * @brief Command processor for two-motor differential (skid-steer) robots.
 */
namespace rr_motor_controller
{

/**
 * @class DifferentialCmdProc
 * @brief Converts Twist commands to per-motor commands for a differential drive.
 *
 * Processes @c geometry_msgs::msg::Twist messages whose fields are:
 *  - @b linear  (x: forward/backward m/s, y: lateral m/s, z: vertical m/s)
 *  - @b angular (x: roll rad/s, y: pitch rad/s, z: yaw rad/s)
 *
 * Processing rules:
 *  -# The z component of both linear is always ignored.
 *  -# If angular z is non-zero rotation is processed and linear is ignored
 *
 * The returned vector contains exactly two MotorCommand entries:
 *  - Index 0: left motor
 *  - Index 1: right motor
 */
class RR_MOTOR_CONTROLLER_PUBLIC DifferentialCmdProc : public MotorCmdProc
{
public:
  DifferentialCmdProc() = default;
  ~DifferentialCmdProc() = default;

  /**
   * @brief Initialise parameters from the lifecycle node.
   *
   * Reads the following ROS parameters:
   *  - @b wheel_base  (double) Distance in metres between the left and right wheels.
   *  - @b ttl_ns      (uint64_t) Time-to-live for each MotorCommand in nanoseconds.
   *                   Commands older than this are considered stale.
   *  - @b covariance  (double[]) Optional flat 36-element array that populates
   *                   the pose covariance matrix (6x6, row-major, order:
   *                   x, y, z, roll, pitch, yaw).  Defaults to all zeros.
   *
   * @param node Shared pointer to the owning lifecycle node.
   */
  void on_configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) override;

  /** @copydoc MotorCmdProc::proc_twist */
  std::vector<MotorCommand> proc_twist(geometry_msgs::msg::Twist msg) override;

  /** @copydoc MotorCmdProc::proc_odom */
  nav_msgs::msg::Odometry proc_odom(const std::vector<MotorCommand>) override;

  /** Index of the left motor within command_history_ and integrated_distance_. */
  static constexpr int DD_LEFT = 0;
  /** Index of the right motor within command_history_ and integrated_distance_. */
  static constexpr int DD_RIGHT = 1;

protected:

  /**
   * @brief Integrate wheel motion and update the dead-reckoned pose.
   *
   * Called internally by @c proc_odom() on every odometry request.  On each
   * call it:
   *  1. Computes the distance each wheel has travelled since the last update
   *     by integrating @c command_history_ up to the current time.
   *  2. Applies differential-drive kinematics to advance @c x_, @c y_, and
   *     @c theta_.
   *  3. Computes instantaneous linear and angular velocities (@c v_linear_,
   *     @c v_angular_) from the displacement and elapsed time.
   *  4. Prunes expired commands from @c command_history_.
   *
   * @return The timestamp (nanoseconds, RCL_STEADY_TIME) used for this update,
   *         which is also applied to the odometry message header stamp.
   */
  uint64_t update();

  /**
   * @brief Build a MotorCommand from a signed velocity.
   *
   * Decomposes the signed value into an absolute speed and a direction
   * flag, and stamps the command with the current time plus ttl_ns_.
   *
   * @param velocity Signed velocity in m/s (negative = backward).
   * @return MotorCommand with magnitude, direction, and expiry set.
   */
  MotorCommand make_cmd(double velocity);

  /**
   * @brief Integrate a command history to compute total distance travelled.
   *
   * Treats each consecutive pair of commands as a constant-velocity segment
   * and sums (velocity × duration) across all segments up to @p now_ns.
   * The last command in the history is assumed to remain active until
   * @p now_ns.
   *
   * @param history Ordered list of MotorCommands for one wheel.
   * @param now_ns  Current time in nanoseconds (RCL_STEADY_TIME).
   * @return Signed distance in metres (negative = backward).
   */
  double compute_distance(const std::vector<rr_motor_controller::MotorCommand>& history, uint64_t now_ns);

  /**
   * @brief Remove commands from history that are no longer needed.
   *
   * Keeps at least the most-recent active command so that @c compute_distance
   * always has a valid baseline.  Called at the end of every @c update().
   *
   * @param now_ns Current time in nanoseconds (RCL_STEADY_TIME).
   */
  void prune_history(uint64_t now_ns);

private:

  bool is_configured = false;

  /** Time-to-live applied to each issued MotorCommand (nanoseconds). */
  uint64_t ttl_ns_ = 0;

  /** Distance between left and right wheel contact patches (metres). */
  double wheel_base_ = 0;

  /**
   * @brief Per-wheel command history used by compute_distance().
   *
   * Indexed by DD_LEFT and DD_RIGHT.  Each inner vector is ordered
   * chronologically and pruned by prune_history() after every update.
   */
  std::vector<std::vector<rr_motor_controller::MotorCommand>> command_history_;

  /** Dead-reckoned x position in the odom frame (metres). */
  double x_ = 0.0;
  /** Dead-reckoned y position in the odom frame (metres). */
  double y_ = 0.0;
  /** Dead-reckoned heading angle from the odom frame x-axis (radians). */
  double theta_ = 0.0;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_{ nullptr };

  /** Timestamp of the previous update() call (nanoseconds, RCL_STEADY_TIME). */
  uint64_t last_update_ns_ = 0;

  /** Duration of the last update cycle (seconds). */
  double dt_ = 0.0;

  /** Instantaneous linear velocity of the robot centre (m/s), updated by update(). */
  double v_linear_ = 0.0;
  /** Instantaneous angular velocity about the z-axis (rad/s), updated by update(). */
  double v_angular_ = 0.0;

  /**
   * @brief Pose covariance matrix stored as a flat 36-element row-major array.
   *
   * Represents a 6x6 symmetric matrix over the state vector [x, y, z, roll, pitch, yaw].
   * Diagonal elements (variances) are at indices: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw.
   * Off-diagonal elements express correlation between axes (set to 0 = uncorrelated).
   * Loaded from the @b covariance ROS parameter in on_configure(); defaults to all zeros.
   */
  std::array<double, 36> covariance_{};

  /**
   * @brief Cumulative distance integrated per wheel since construction (metres).
   *
   * Used to compute per-cycle deltas in update() without re-integrating the
   * full command history from the beginning each time.
   * Indexed by DD_LEFT and DD_RIGHT.
   */
  std::array<double, 2> integrated_distance_ = {0.0, 0.0};
};
}  // namespace rr_motor_controller
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
#include <vector>
#include "rr_motor_controller/rr_motor_controller_common.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @file differential.hpp
 * @brief Command processor for two-motor differential (skid-steer) robots.
 */
namespace rr_motor_controller {

/**
 * @class DifferentalCmdProc
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
    class DifferentalCmdProc : public MotorCmdProc {
        public:

        DifferentalCmdProc() = default;
        ~DifferentalCmdProc() = default;

        void on_configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) override;

        /** @copydoc MotorCmdProc::proc_twist */
        std::vector<MotorCommand> proc_twist(geometry_msgs::msg::Twist  msg) override;

        /** @copydoc MotorCmdProc::proc_odom */
        nav_msgs::msg::Odometry proc_odom(const std::vector<MotorCommand>) override;

        static constexpr int DD_LEFT = 0;
        static constexpr int DD_RIGHT = 1;

        protected:
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

        private:
        uint64_t ttl_ns_ = 0;
        double wheel_base_ = 0;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ {nullptr};
    };
}
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
#include <array>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rr_motor_controller/rr_motor_controller_common.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "nav_msgs/msg/odometry.hpp"

/**
 * @file motorcmdproc.hpp
 * @brief Abstract interface for motor command processing.
 * 
 * CAVEAT: motor commands are currently use array of size 2, which is fine for
 * differential motors, but may not work for different robot setups, and may need
 * to be changed to vector. It definately will not work for more than two motors.
 */
namespace rr_motor_controller {

/**
 * @class MotorCmdProc
 * @brief Interface for converting between ROS messages and motor commands.
 *
 * Implementations handle the kinematics for a specific drive layout
 * (e.g. differential, mecanum). Two operations are provided:
 *  - Convert a Twist velocity command into per-motor MotorCommand values.
 *  - Convert per-motor MotorCommand feedback back into an Odometry message.
 */
    class MotorCmdProc  {
        public:

        virtual ~MotorCmdProc() = default;

        /**
         * @brief Lifecycle configure callback — acquire parameters from the node.
         * @param node The lifecycle node that owns this processor.
         */
        virtual void on_configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) = 0;

        /**
         * @brief Convert a Twist message into motor commands.
         * @param msg Desired linear and angular velocity.
         * @return Vector of MotorCommand, one per driven motor.
         */
        virtual std::array<MotorCommand, 2> proc_twist(geometry_msgs::msg::Twist  msg) = 0;

        /**
         * @brief Compute odometry from motor command feedback.
         * @param commands Current motor command state for each motor.
         * @return Odometry message representing the robot's estimated pose and velocity.
         */
        virtual nav_msgs::msg::Odometry proc_odom(const std::array<MotorCommand, 2>) = 0;
    };
}
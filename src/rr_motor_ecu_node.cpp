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

#include "rr_motor_controller/rr_motor_ecu_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
using RRGPIOInterface = rrobots::interfaces::RRGPIOInterface;
using namespace rr_motor_controller;

CallbackReturn RrECU::on_configure(const State& state)
{
  RCLCPP_INFO(this->get_logger(), "Configuring motor ECU...");

  // create and configure gpio first, if this fails, dont want to have
  // to roll back everything else.
  std::string plugin_param = get_parameter("transport_plugin").as_string();
  RCLCPP_DEBUG(get_logger(), "transport plugin is '%s'", plugin_param.c_str());
  try
  {
    poly_loader_ = std::make_unique<pluginlib::ClassLoader<RRGPIOInterface>>("rr_common_base",
                                                                             "rrobots::interfaces::"
                                                                             "RRGPIOInterface");
    gpio_plugin_ = poly_loader_->createUniqueInstance(plugin_param);
    if (gpio_plugin_->configure(state, this->shared_from_this()) != CallbackReturn::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "could not configure gpio_plugin!!");
      return CallbackReturn::FAILURE;
    }
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(get_logger(), "could not load transport plugin: %s - reported: %s", plugin_param.c_str(), ex.what());
    return CallbackReturn::ERROR;
  }

  // in future versions this may be selected for different robot types
  // and possibly a plugin but for now, just create a shared instance here.
  mt_cmd_proc_ = std::make_unique<DifferentialCmdProc>();
  mt_cmd_proc_->on_configure(this->shared_from_this());

  // Configure motors
  motors_[DifferentialCmdProc::DD_LEFT].on_configure(state, this->shared_from_this(), DifferentialCmdProc::DD_LEFT);
  motors_[DifferentialCmdProc::DD_RIGHT].on_configure(state, this->shared_from_this(), DifferentialCmdProc::DD_RIGHT);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RrECU::on_activate(const State& state)
{
  if (gpio_plugin_->on_activate(state) != CallbackReturn::SUCCESS)
  {
    RCLCPP_FATAL(get_logger(), "failed to activate gpio plugin");
    return CallbackReturn::FAILURE;
  }
  for (size_t i = 0; i < motors_.size(); i++) {
    // TODO: activate callbacks and timers for motor_controllers
    if (motors_[i].on_activate(state) != CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(get_logger(), "failed to activate motor(s)");
        for (; i > -1; i--) {
            motors_[i].on_deactivate(state);
        }
        return CallbackReturn::FAILURE;
    }
  }

  // TODO: create wall timers for internal subscription, these do not have to
  // be deterministic, so wall_timer is fine.
  return CallbackReturn::SUCCESS;
}

CallbackReturn RrECU::on_deactivate(const State& state)
{
}

CallbackReturn RrECU::on_cleanup(const State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Cleaning up motor ECU...");

  gpio_plugin_.reset();
  mt_cmd_proc_.reset();

  return CallbackReturn::SUCCESS;
}

void RrECU::subscribe_callback_(const geometry_msgs::msg::Twist& req)
{
}

void RrECU::publish_callback_()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(rr_motor_controller::RrECU)
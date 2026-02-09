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


// TODO: 
// * Critical
// ** Fix Duty cycle it needs to be offset.
// ** Craete subscriber
// ** Create publisher
// * Next Release
// ** Duty from PID.

#include "rr_motor_controller/rr_motor_controller.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
using RRGPIOInterface = rrobots::interfaces::RRGPIOInterface;

namespace rr_motor_controller
{

CallbackReturn RrMotorController::on_configure(const State& state)
{
  RCLCPP_INFO(get_logger(), "Configuring motor controller...");

  // set the element that should be listed too, this is the least dangerous thing, it is first because
  // it will not leave dangling pointers, or bad memory if it fails at this point.
  if (!(get_parameter("motor_pos", motor_pos_) && get_parameter("ppr", ppr_) &&
        get_parameter("wheel_radius", wheel_radius_)))
  {
    RCLCPP_ERROR(get_logger(), "Failed to get parameters for motor configuration");
    return CallbackReturn::FAILURE;
  }

  // compute dpp here, to avoid calculations in callbacks.
  dpp_ = (2 * M_PI * wheel_radius_)/ (ppr_ * 1000.0);

  // attempt to load the plugin.
  declare_parameter("transport_plugin", "rrobots::interfaces::RRGPIOInterface");
  std::string plugin_param = get_parameter("transport_plugin").as_string();
  RCLCPP_DEBUG(get_logger(), "transport plugin is '%s'", plugin_param.c_str());

  try
  {
    std::unique_ptr<pluginlib::ClassLoader<RRGPIOInterface>> poly_loader_ =
        std::make_unique<pluginlib::ClassLoader<RRGPIOInterface>>("rr_common_base",
                                                                  "rrobots::interfaces::"
                                                                  "RRGPIOInterface");
    gpio_plugin_ = poly_loader_->createUniqueInstance(plugin_param);
    if (gpio_plugin_->initialise() != 0)
    {
      RCLCPP_ERROR(get_logger(), "could not initlize gpio_plugin!!");
      return CallbackReturn::FAILURE;
    }
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(get_logger(), "could not load transport plugin: %s - reported: %s", plugin_param.c_str(), ex.what());
    return CallbackReturn::ERROR;
  }

  // CAVEAT: configure motor, note that plugin is not rolled back. It is expected that this is done during
  // deactive routine within this method. Therefore deactivate must always be called.
  if (motor_.configure(state, this->shared_from_this(), gpio_plugin_) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Motor configuration failed!!");
    return CallbackReturn::FAILURE;
  }

  // setup encoder
  tick_cb_ = [this](int gpio_pin, uint32_t delta_us, uint32_t tick, TickStatus tick_status) {
    this->encoder_cb_(gpio_pin, delta_us, tick, tick_status);
  };

  if (encoder_.configure(state, this->shared_from_this(), gpio_plugin_, tick_cb_) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Encoder configuration failed!!");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

// TODO: create subscription and publsiher.
CallbackReturn RrMotorController::on_activate(const State& state)
{
  if (motor_.on_activate(state) != CallbackReturn::SUCCESS)
  {
    // this->create_subscription();
    // this->create_publisher();
    return CallbackReturn::FAILURE;
  }

  if (encoder_.on_activate(state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::FAILURE;
  }

  // setup subscription and publisher.

  return CallbackReturn::SUCCESS;
}

void RrMotorController::publish_callback_()
{
  // Do not log, this could significantly slow processing.
  rr_interfaces::msg::MotorResponse response;
  response.header.stamp = now();
  response.header.frame_id = rr_constants::LINK_MOTOR;
  response.velocity = velocity_.load();
  response.total_pulses = total_pulses_.load();
  response.healthy_pulses = healthy_pulses_.load();
  response.boundary_triggers = boundary_triggers_.load();
  publisher_->publish(response);
}

// TODO: This method should use a PID algorithm to translate the velocity to a duty cycle.
// however this will work for now. This will be done in next release.
void RrMotorController::subscribe_callback_(const rr_interfaces::msg::Motors& req)
{
  // Do not log, this could significantly slow processing.

  // 1,000,000 / vel

  if (req.motors.size() > static_cast<std::size_t>(motor_pos_))
  {
    double duty = 1'000'000.0 / (static_cast<double>(req.motors.at(motor_pos_).velocity) / dpp_);
    motor_.set_pwm(duty);
    motor_.set_direction(req.motors.at(motor_pos_).direction);
  }
}

CallbackReturn RrMotorController::on_deactivate(const State& state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RrMotorController::on_cleanup(const State& state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

void RrMotorController::encoder_cb_(const int gpio_pin, const uint32_t delta_us, const uint32_t tick,
                                    const TickStatus tick_status)
{
  (void)gpio_pin;
  (void)tick;

  if (!running_.load(std::memory_order_acquire))
  {
    return;
  }

  total_pulses_.fetch_add(1, std::memory_order_relaxed);

  // Accumulate timing (ONCE!)
  if (tick_status == TickStatus::HEALTHY && delta_us > MIN_DELTA_US && delta_us < MAX_DELTA_US)
  {
    healthy_pulses_.fetch_add(1, std::memory_order_relaxed);
    delta_us_ct_.fetch_add(1, std::memory_order_acq_rel);
    delta_us_accum_.fetch_add(static_cast<uint64_t>(delta_us), std::memory_order_acq_rel);
  }

  // Atomic increment with boundary check
  int old_count = delta_ct_.fetch_add(1, std::memory_order_acq_rel);

  // Check if THIS interrupt brought us to exactly PPR
  if (old_count == ppr_ - 1)
  {
    boundary_triggers_.fetch_add(1, std::memory_order_relaxed);

    int expected = ppr_;
    if (delta_ct_.compare_exchange_strong(expected, 0, std::memory_order_acq_rel, std::memory_order_acquire))
    {
      uint64_t accum = delta_us_accum_.load(std::memory_order_acquire);
      int ct = delta_us_ct_.load(std::memory_order_acquire);

      delta_us_ct_.store(0, std::memory_order_release);
      delta_us_accum_.store(0, std::memory_order_release);

      if (accum > 0 && ct > 0)
      {
        double avg_us = static_cast<double>(accum) / static_cast<double>(ct);
        double new_vel = (dpp_ / 1000.0) / avg_us;
        double current_vel = velocity_.load(std::memory_order_acquire);

        // Smooth with EMA (alpha = 0.3)
        double smoothed_vel = 0.7 * current_vel + 0.3 * new_vel;
        velocity_.store(smoothed_vel, std::memory_order_release);
      }
    }
  }
}

}  // namespace rr_motor_controller

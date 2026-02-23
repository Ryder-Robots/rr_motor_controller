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

#include "rr_motor_controller/rr_motor_controller.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
using RRGPIOInterface = rrobots::interfaces::RRGPIOInterface;

namespace rr_motor_controller
{

CallbackReturn RrMotorController::on_configure(const State& state,
                                               std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int mpos,
                                               std::shared_ptr<RRGPIOInterface> gpio_plugin)
{
  RCLCPP_INFO(node->get_logger(), "Configuring motor controller...");
  gpio_plugin_ = gpio_plugin;
  // Parameters are loaded first — a failure here leaves no hardware state to unwind.
  if (!(node->get_parameter("motor_pos", motor_pos_) && node->get_parameter("ppr", ppr_) &&
        node->get_parameter("wheel_radius", wheel_radius_)))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameters for motor configuration");
    return CallbackReturn::FAILURE;
  }

  // Pre-compute distance per pulse (mm). Used by encoder_cb_ to derive velocity:
  dpp_ = (2 * M_PI * wheel_radius_) / ppr_;

  // Configure motor hardware. On failure the GPIO plugin is not rolled back here;
  // on_deactivate is expected to handle full teardown.
  if (motor_.configure(state, node->shared_from_this(), gpio_plugin_, mpos) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Motor configuration failed!!");
    return CallbackReturn::FAILURE;
  }

  // Bind encoder interrupt to encoder_cb_ via lambda (captures 'this').
  tick_cb_ = [this](int gpio_pin, uint32_t delta_us, uint32_t tick, TickStatus tick_status) {
    this->encoder_cb_(gpio_pin, delta_us, tick, tick_status);
  };

  if (encoder_.configure(state, node->shared_from_this(), gpio_plugin_, tick_cb_, mpos) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Encoder configuration failed!!");
    return CallbackReturn::FAILURE;
  }
  node_ = node;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RrMotorController::on_activate(const State& state)
{
  // Activate plugin, this will create a hardware instance of the GPIO layer
  if (gpio_plugin_ == nullptr || gpio_plugin_->on_activate(state) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Activation of gpio_plugin failed!!");
    return CallbackReturn::FAILURE;
  }

  // Activate first — if encoder fails, roll back motor.
  if (motor_.on_activate(state) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Motor activation failed!!");

    gpio_plugin_->on_deactivate(state);
    return CallbackReturn::FAILURE;
  }

  if (encoder_.on_activate(state) != CallbackReturn::SUCCESS)
  {
    motor_.on_deactivate(state);
    gpio_plugin_->on_deactivate(state);
    RCLCPP_ERROR(node_->get_logger(), "Encoder activation failed!!");
    return CallbackReturn::FAILURE;
  }

  // Duty convertor strategy — currently linear regression, swappable to PID.
  duty_conv_ = std::make_shared<rr_motor_controller::DutyConvertorLinearRegression>();
  running_.store(true, std::memory_order_release);

  pid_timer_ = std::thread(&RrMotorController::pid_cb_, this);
  sched_param sch{};
  sch.sched_priority = 80;
  if (pthread_setschedparam(pid_timer_.native_handle(), SCHED_FIFO, &sch) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Realtime scheduling failed for PID timer!!");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RrMotorController::on_deactivate(const State& state)
{
  // Signal all callbacks to stop, then cancel timers before destroying
  // the resources they reference.
  running_.store(false, std::memory_order_release);

  if (publisher_ != nullptr)
  {
    publisher_->on_deactivate();
  }

  if (subscription_ != nullptr)
  {
    subscription_ = nullptr;
  }

  // Deactivate hardware — continue through failures to ensure both are attempted.
  CallbackReturn rv = CallbackReturn::SUCCESS;
  if (pid_timer_.joinable())
  {
    pid_timer_.join();
  }

  if (encoder_.on_deactivate(state) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Encoder deactivation failed!!");
    rv = CallbackReturn::FAILURE;
  }

  if (motor_.on_deactivate(state) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Motor deactivation failed!!");
    rv = CallbackReturn::FAILURE;
  }
  if (gpio_plugin_->on_deactivate(state) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "GPIO plugin deactivation failed!!");
    rv = CallbackReturn::FAILURE;
  }

  return rv;
}

CallbackReturn RrMotorController::on_cleanup(const State& state)
{
  (void)state;
  subscription_.reset();
  publisher_.reset();
  duty_conv_.reset();
  tick_cb_ = nullptr;
  return CallbackReturn::SUCCESS;
}

// This method will significantly change, it will no longer be a subscription service.
// or get an interface request, more than likely it will get ran in some sort of loop
// possibly with hardware clock.
void RrMotorController::process_cmd(const rr_interfaces::msg::Motors& req)
{
  RCLCPP_DEBUG(node_->get_logger(), "subscriber callback is getting called!");

  if (!running_.load(std::memory_order_acquire))
  {
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "seeing if motor needs to be updated");
  if (req.motors.size() > static_cast<std::size_t>(motor_pos_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "attempting to update the motor");
    target_velocity_.store(static_cast<double>(req.motors.at(motor_pos_).velocity), std::memory_order_release);
    direction_.store(req.motors.at(motor_pos_).direction, std::memory_order_release);
  }
}

void RrMotorController::pid_cb_()
{
  bool running = running_.load(std::memory_order_acquire);
  struct itimerspec ts
  {
  };
  ts.it_value.tv_sec = 0;
  ts.it_value.tv_nsec = PID_TIMER_DELTA * 1'000'000;
  ts.it_interval.tv_sec = 0;
  ts.it_interval.tv_nsec = PID_TIMER_DELTA * 1'000'000;

  auto timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
  if (timer_fd < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "unable to create timer");
    running_.store(false, std::memory_order_relaxed);
    return;
  }

  if (timerfd_settime(timer_fd, 0, &ts, nullptr) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "unable to set timer");
    close(timer_fd);
    running_.store(false, std::memory_order_relaxed);
    return;
  }

  uint64_t expiration = 0;

  while (running)
  {
    ssize_t ret = read(timer_fd, &expiration, sizeof(expiration));
    if (ret < 0)
    {
      if (errno == EINTR)
        continue;
      RCLCPP_ERROR(node_->get_logger(), "error reading PID timer");
      close(timer_fd);
      running_.store(false, std::memory_order_relaxed);
      return;
    }

    auto duty = static_cast<int>(
        duty_conv_->compute(target_velocity_, velocity_.load(std::memory_order_acquire), PID_TIMER_DELTA));
    motor_.set_pwm(duty);

    // only change direction if it is different, this could create some instability.
    bool dir = static_cast<bool>(direction_.load());
    if (motor_.get_direction() != dir)
    {
      motor_.set_direction(dir);
    }

    running = running_.load(std::memory_order_acquire);
  }
  close(timer_fd);
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

  // Only accumulate timing from pulses within the valid window.
  if (tick_status == TickStatus::HEALTHY && delta_us > MIN_DELTA_US && delta_us < MAX_DELTA_US)
  {
    healthy_pulses_.fetch_add(1, std::memory_order_relaxed);
    delta_us_ct_.fetch_add(1, std::memory_order_acq_rel);
    delta_us_accum_.fetch_add(static_cast<uint64_t>(delta_us), std::memory_order_acq_rel);
  }

  // Track all pulses (healthy or not) toward the revolution boundary.
  int old_count = delta_ct_.fetch_add(1, std::memory_order_acq_rel);

  // On the pulse that completes a full revolution, compute velocity.
  if (old_count == ppr_ - 1)
  {
    boundary_triggers_.fetch_add(1, std::memory_order_relaxed);

    // CAS reset: ensures exactly one thread processes the revolution boundary.
    int expected = ppr_;
    if (delta_ct_.compare_exchange_strong(expected, 0, std::memory_order_acq_rel, std::memory_order_acquire))
    {
      uint64_t accum = delta_us_accum_.exchange(0, std::memory_order_acq_rel);
      int ct = delta_us_ct_.exchange(0, std::memory_order_acq_rel);

      if (accum > 0 && ct > 0)
      {
        // Note that each motor controller is its own thread, therefore dpp_ and friends,
        // can not be overwritten by antyhing outside of the thread.
        double avg_us = static_cast<double>(accum) / static_cast<double>(ct);
        double new_vel = (dpp_ * 1000.0) / avg_us;
        double current_vel = velocity_.load(std::memory_order_acquire);

        // Smooth with EMA (alpha = 0.3)
        double smoothed_vel = 0.7 * current_vel + 0.3 * new_vel;
        velocity_.store(smoothed_vel, std::memory_order_release);
      }
    }
  }
}

}  // namespace rr_motor_controller

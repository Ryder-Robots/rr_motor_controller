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

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

// Include standard/ROS headers first (before visibility workaround)
#include <string>
#include <atomic>
#include <climits>
#include <pluginlib/class_loader.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_motor_controller/motor.hpp"
#include "rr_motor_controller/encoder.hpp"
#include "rr_motor_controller/dutyconv.hpp"
#include "rr_motor_controller/dc_linear.hpp"
#include "rr_motor_controller/rr_motor_controller_common.hpp"

// Test-only visibility workaround — allows direct access to private members
// for verifying internal state in unit tests. Scoped to this file only.
#define private public
#define protected public
#include "rr_motor_controller/rr_motor_controller.hpp"
#undef private
#undef protected

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using TickStatus = rr_motor_controller::TickStatus;

class RrMotorControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctrl_ = std::make_shared<rr_motor_controller::RrMotorController>();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_controller_node");
  }

  // Helper: put the controller into a testable state with known config
  // (bypasses pluginlib by setting fields directly)
  void configure_for_test(int ppr = 8, int wheel_radius = 20) {
    ctrl_->node_ = node_;
    ctrl_->ppr_ = ppr;
    ctrl_->wheel_radius_ = wheel_radius;
    ctrl_->dpp_ = (2.0 * M_PI * wheel_radius) / ppr;
    ctrl_->motor_pos_ = 0;
    ctrl_->running_.store(true, std::memory_order_release);
  }

  // Helper: fire N healthy encoder pulses with given delta_us
  void fire_healthy_pulses(int count, uint32_t delta_us) {
    for (int i = 0; i < count; ++i) {
      ctrl_->encoder_cb_(0, delta_us, 0, TickStatus::HEALTHY);
    }
  }

  std::shared_ptr<rr_motor_controller::RrMotorController> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

// ---------------------------------------------------------------------------
// NOTE: ConstructorDeclaresAllParameters removed — RrMotorController is no
// longer a lifecycle node. Parameters are declared and owned by the node
// passed to on_configure(). Parameter declaration tests belong in the ECU
// node tests (test_rr_motor_ecu_node or equivalent).
// ---------------------------------------------------------------------------
// encoder_cb_ — gating
// ---------------------------------------------------------------------------

TEST_F(RrMotorControllerTest, EncoderCbIgnoredWhenNotRunning) {
  configure_for_test();
  ctrl_->running_.store(false, std::memory_order_release);

  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::HEALTHY);
  EXPECT_EQ(ctrl_->total_pulses_.load(), 0);
}

TEST_F(RrMotorControllerTest, EncoderCbCountsTotalPulses) {
  configure_for_test();

  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::HEALTHY);
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::TIMEOUT);
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::NOISE_REJECTED);

  EXPECT_EQ(ctrl_->total_pulses_.load(), 3);
}

// ---------------------------------------------------------------------------
// encoder_cb_ — healthy pulse filtering
// ---------------------------------------------------------------------------

TEST_F(RrMotorControllerTest, EncoderCbCountsHealthyPulses) {
  configure_for_test();

  // Healthy pulse within timing window
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::HEALTHY);
  EXPECT_EQ(ctrl_->healthy_pulses_.load(), 1);

  // Non-healthy status — not counted as healthy
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::TIMEOUT);
  EXPECT_EQ(ctrl_->healthy_pulses_.load(), 1);

  // Healthy but below MIN_DELTA_US (300)
  ctrl_->encoder_cb_(0, 200, 0, TickStatus::HEALTHY);
  EXPECT_EQ(ctrl_->healthy_pulses_.load(), 1);

  // Healthy but above MAX_DELTA_US (3000)
  ctrl_->encoder_cb_(0, 4000, 0, TickStatus::HEALTHY);
  EXPECT_EQ(ctrl_->healthy_pulses_.load(), 1);

  // Healthy and within window
  ctrl_->encoder_cb_(0, 500, 0, TickStatus::HEALTHY);
  EXPECT_EQ(ctrl_->healthy_pulses_.load(), 2);
}

// ---------------------------------------------------------------------------
// encoder_cb_ — revolution boundary and velocity computation
// ---------------------------------------------------------------------------

TEST_F(RrMotorControllerTest, EncoderCbComputesVelocityAfterFullRevolution) {
  configure_for_test(8, 20);
  // dpp_ = (2 * PI * 20) / 8 = 5*PI ≈ 15.70796

  // Fire 8 healthy pulses (one full revolution), each 1000us apart
  fire_healthy_pulses(8, 1000);

  // Expected: avg_us = 1000, velocity = (dpp_ * 1000) / 1000 = dpp_ = 5*PI
  // EMA: smoothed = 0.7 * 0.0 + 0.3 * (5*PI) = 1.5*PI
  double expected_dpp = (2.0 * M_PI * 20.0) / 8.0;
  double expected_vel = 0.3 * ((expected_dpp * 1000.0) / 1000.0);
  EXPECT_NEAR(ctrl_->velocity_.load(), expected_vel, 1e-10);
  EXPECT_EQ(ctrl_->boundary_triggers_.load(), 1);
}

TEST_F(RrMotorControllerTest, EncoderCbEmaSmoothing) {
  configure_for_test(8, 20);
  double dpp = (2.0 * M_PI * 20.0) / 8.0;

  // First revolution: 8 pulses at 1000us
  fire_healthy_pulses(8, 1000);
  double raw_vel_1 = (dpp * 1000.0) / 1000.0;
  double expected_1 = 0.3 * raw_vel_1;  // 0.7 * 0 + 0.3 * raw
  EXPECT_NEAR(ctrl_->velocity_.load(), expected_1, 1e-10);

  // Second revolution: 8 pulses at 500us (faster)
  fire_healthy_pulses(8, 500);
  double raw_vel_2 = (dpp * 1000.0) / 500.0;
  double expected_2 = 0.7 * expected_1 + 0.3 * raw_vel_2;
  EXPECT_NEAR(ctrl_->velocity_.load(), expected_2, 1e-10);

  EXPECT_EQ(ctrl_->boundary_triggers_.load(), 2);
}

TEST_F(RrMotorControllerTest, EncoderCbDeltaCounterResetsAfterRevolution) {
  configure_for_test(8, 20);

  fire_healthy_pulses(8, 1000);
  // After one revolution delta_ct_ should be back to 0
  EXPECT_EQ(ctrl_->delta_ct_.load(), 0);

  // Partial revolution
  fire_healthy_pulses(3, 1000);
  EXPECT_EQ(ctrl_->delta_ct_.load(), 3);
}

TEST_F(RrMotorControllerTest, EncoderCbNoVelocityIfAllPulsesOutOfWindow) {
  configure_for_test(4, 20);

  // Fire 4 pulses that are all outside the timing window (too fast)
  for (int i = 0; i < 4; ++i) {
    ctrl_->encoder_cb_(0, 100, 0, TickStatus::HEALTHY);  // 100 < MIN_DELTA_US (300)
  }

  // Boundary triggered but no healthy pulses accumulated, so velocity stays 0
  EXPECT_EQ(ctrl_->boundary_triggers_.load(), 1);
  EXPECT_DOUBLE_EQ(ctrl_->velocity_.load(), 0.0);
}

TEST_F(RrMotorControllerTest, EncoderCbMixedHealthyAndUnhealthyPulses) {
  configure_for_test(4, 20);
  double dpp = (2.0 * M_PI * 20.0) / 4.0;

  // 2 healthy pulses at 1000us + 2 unhealthy (TIMEOUT) — all count toward revolution
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::HEALTHY);
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::HEALTHY);
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::TIMEOUT);
  ctrl_->encoder_cb_(0, 1000, 0, TickStatus::TIMEOUT);

  // avg_us computed from 2 healthy pulses only: 1000
  double raw_vel = (dpp * 1000.0) / 1000.0;
  double expected = 0.3 * raw_vel;
  EXPECT_NEAR(ctrl_->velocity_.load(), expected, 1e-10);
  EXPECT_EQ(ctrl_->healthy_pulses_.load(), 2);
  EXPECT_EQ(ctrl_->total_pulses_.load(), 4);
}

// ---------------------------------------------------------------------------
// process_cmd
// ---------------------------------------------------------------------------

TEST_F(RrMotorControllerTest, ProcessCmdStoresVelocityAndDirection) {
  configure_for_test();

  rr_motor_controller::MotorCommand cmd{};
  cmd.velocity = 42.0f;
  cmd.direction = static_cast<int8_t>(rr_motor_controller::Motor::BACKWARD);
  cmd.ttl_ns = UINT64_MAX;  // far future: now >= UINT64_MAX is false → command processed

  ctrl_->process_cmd(cmd);

  EXPECT_DOUBLE_EQ(ctrl_->target_velocity_.load(), 42.0);
  EXPECT_EQ(ctrl_->direction_.load(), rr_motor_controller::Motor::BACKWARD);
}

// process_cmd discards commands when now >= ttl_ns (command has expired).
TEST_F(RrMotorControllerTest, ProcessCmdDiscardedWhenExpired) {
  configure_for_test();
  ctrl_->target_velocity_.store(99.0, std::memory_order_release);

  rr_motor_controller::MotorCommand cmd{};
  cmd.velocity = 42.0f;
  cmd.direction = static_cast<int8_t>(rr_motor_controller::Motor::FORWARD);
  cmd.ttl_ns = 0;  // already expired: now >= 0 is always true → discarded

  ctrl_->process_cmd(cmd);

  EXPECT_DOUBLE_EQ(ctrl_->target_velocity_.load(), 99.0);  // unchanged
}

TEST_F(RrMotorControllerTest, ProcessCmdIgnoredWhenNotRunning) {
  configure_for_test();
  ctrl_->running_.store(false, std::memory_order_release);
  ctrl_->target_velocity_.store(99.0, std::memory_order_release);

  rr_motor_controller::MotorCommand cmd{};
  cmd.velocity = 42.0f;
  cmd.direction = static_cast<int8_t>(rr_motor_controller::Motor::FORWARD);
  cmd.ttl_ns = UINT64_MAX;  // valid TTL — ensures running_ gate is what rejects this

  ctrl_->process_cmd(cmd);

  EXPECT_DOUBLE_EQ(ctrl_->target_velocity_.load(), 99.0);  // unchanged
}

// ---------------------------------------------------------------------------
// on_cleanup
// ---------------------------------------------------------------------------

TEST_F(RrMotorControllerTest, OnCleanupResetsTickCallback) {
  configure_for_test();
  ctrl_->tick_cb_ = [](int, uint32_t, uint32_t, TickStatus) {};
  EXPECT_NE(ctrl_->tick_cb_, nullptr);

  rclcpp_lifecycle::State state;
  auto result = ctrl_->on_cleanup(state);
  EXPECT_EQ(result, CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl_->tick_cb_, nullptr);
}

// ---------------------------------------------------------------------------
// DPP computation verification
// ---------------------------------------------------------------------------

TEST_F(RrMotorControllerTest, DppComputedCorrectly) {
  configure_for_test(16, 50);
  double expected_dpp = (2.0 * M_PI * 50.0) / 16.0;
  EXPECT_NEAR(ctrl_->dpp_, expected_dpp, 1e-10);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
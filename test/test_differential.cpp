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
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Test-only visibility workaround — allows direct access to private/protected
// members for verifying internal state. Scoped to this file only.
#define private public
#define protected public
#include "rr_motor_controller/differential.hpp"
#undef private
#undef protected

using namespace rr_motor_controller;
constexpr int DD_LEFT = DifferentialCmdProc::DD_LEFT;
constexpr int DD_RIGHT = DifferentialCmdProc::DD_RIGHT;

// ---------------------------------------------------------------------------
// Constants shared across tests
// ---------------------------------------------------------------------------

static constexpr double   kWheelBase = 0.3;          // 30 cm between wheels
static constexpr uint64_t kTtlNs     = 500000000ULL; // 500 ms command TTL
static constexpr double   kTolerance = 1e-6;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static geometry_msgs::msg::Twist make_twist(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist t;
  t.linear.x  = linear_x;
  t.angular.z = angular_z;
  return t;
}

// ---------------------------------------------------------------------------
// Fixture — configured DifferentialCmdProc with known wheel geometry
//
// Simulates the state the object is in when owned by rr_motor_ecu_node after
// the lifecycle configure transition has completed.
// ---------------------------------------------------------------------------

class DifferentialCmdProcTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_diff_node");
    diff_ = std::make_shared<DifferentialCmdProc>();

    // on_configure reads parameters via get_parameter_or with integer default 0.
    // We call it to exercise the configure path (history resize, node storage,
    // is_configured flag), then override the geometry fields directly so that
    // tests are not coupled to the ROS parameter type inference rules.
    diff_->on_configure(node_);
    diff_->wheel_base_ = kWheelBase;
    diff_->ttl_ns_     = kTtlNs;
  }

  // Simulate one ECU timer cycle: proc_odom drives update() internally.
  nav_msgs::msg::Odometry odom_tick()
  {
    return diff_->proc_odom({});
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<DifferentialCmdProc> diff_;
};

// ---------------------------------------------------------------------------
// Unconfigured guard — verifies is_configured short-circuits both methods.
// This mirrors the ECU node receiving messages before the lifecycle configure
// transition has completed.
// ---------------------------------------------------------------------------

TEST(DifferentialCmdProcGuardTest, UnconfiguredProcTwistReturnsZeroCommands)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("guard_node");

  DifferentialCmdProc diff;
  // Deliberately skip on_configure — is_configured stays false.
  // proc_twist returns a zero-initialised array (not an empty container).
  auto cmds = diff.proc_twist(make_twist(1.0, 0.0));
  EXPECT_FLOAT_EQ(cmds[0].velocity, 0.0f);
  EXPECT_FLOAT_EQ(cmds[1].velocity, 0.0f);
}

TEST(DifferentialCmdProcGuardTest, UnconfiguredProcOdomReturnsDefaultOdom)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("guard_node2");

  DifferentialCmdProc diff;
  auto odom = diff.proc_odom({});
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, 0.0);
  EXPECT_TRUE(odom.header.frame_id.empty());
}

// ---------------------------------------------------------------------------
// proc_twist — command structure
// ---------------------------------------------------------------------------

TEST_F(DifferentialCmdProcTest, ProcTwistReturnsExactlyTwoCommands)
{
  auto cmds = diff_->proc_twist(make_twist(1.0, 0.0));
  ASSERT_EQ(cmds.size(), 2u);
}

TEST_F(DifferentialCmdProcTest, ProcTwistForwardMotionEqualWheelSpeeds)
{
  // linear.x = 1.0, angular.z = 0  →  v_left = v_right = 1.0
  auto cmds = diff_->proc_twist(make_twist(1.0, 0.0));
  ASSERT_EQ(cmds.size(), 2u);

  EXPECT_FLOAT_EQ(cmds[DifferentialCmdProc::DD_LEFT].velocity,  1.0f);
  EXPECT_FLOAT_EQ(cmds[DifferentialCmdProc::DD_RIGHT].velocity, 1.0f);

  // Both forward (direction != 0)
  EXPECT_NE(cmds[DifferentialCmdProc::DD_LEFT].direction,  0);
  EXPECT_NE(cmds[DifferentialCmdProc::DD_RIGHT].direction, 0);
}

TEST_F(DifferentialCmdProcTest, ProcTwistBackwardMotionBothBackward)
{
  // linear.x = -1.0, angular.z = 0  →  v_left = v_right = -1.0
  auto cmds = diff_->proc_twist(make_twist(-1.0, 0.0));
  ASSERT_EQ(cmds.size(), 2u);

  EXPECT_FLOAT_EQ(cmds[DifferentialCmdProc::DD_LEFT].velocity,  1.0f);  // magnitude stored
  EXPECT_FLOAT_EQ(cmds[DifferentialCmdProc::DD_RIGHT].velocity, 1.0f);

  // Both backward (direction == 0)
  EXPECT_EQ(cmds[DifferentialCmdProc::DD_LEFT].direction,  0);
  EXPECT_EQ(cmds[DifferentialCmdProc::DD_RIGHT].direction, 0);
}

TEST_F(DifferentialCmdProcTest, ProcTwistCCWRotationRightFasterThanLeft)
{
  // angular.z > 0 (CCW): v_right > v_left
  // v_left  = 0 - (1.0 * 0.3/2) = -0.15
  // v_right = 0 + (1.0 * 0.3/2) = +0.15
  auto cmds = diff_->proc_twist(make_twist(0.0, 1.0));
  ASSERT_EQ(cmds.size(), 2u);

  EXPECT_FLOAT_EQ(cmds[DD_LEFT].velocity,  0.15f);
  EXPECT_FLOAT_EQ(cmds[DD_RIGHT].velocity, 0.15f);

  EXPECT_EQ(cmds[DD_LEFT].direction,  0);   // backward
  EXPECT_NE(cmds[DD_RIGHT].direction, 0);   // forward
}

TEST_F(DifferentialCmdProcTest, ProcTwistCWRotationLeftFasterThanRight)
{
  // angular.z < 0 (CW): v_left > v_right
  auto cmds = diff_->proc_twist(make_twist(0.0, -1.0));
  ASSERT_EQ(cmds.size(), 2u);

  EXPECT_EQ(cmds[DD_RIGHT].direction, 0);   // right backward
  EXPECT_NE(cmds[DD_LEFT].direction,  0);   // left forward
}

TEST_F(DifferentialCmdProcTest, ProcTwistCombinedMotionAsymmetricSpeeds)
{
  // linear.x = 0.5, angular.z = 0.5, wheel_base = 0.3
  // v_left  = 0.5 - (0.5 * 0.15) = 0.425
  // v_right = 0.5 + (0.5 * 0.15) = 0.575
  auto cmds = diff_->proc_twist(make_twist(0.5, 0.5));
  ASSERT_EQ(cmds.size(), 2u);

  EXPECT_NEAR(cmds[DD_LEFT].velocity,  0.425f, 1e-5f);
  EXPECT_NEAR(cmds[DD_RIGHT].velocity, 0.575f, 1e-5f);

  // Both forward
  EXPECT_NE(cmds[DD_LEFT].direction,  0);
  EXPECT_NE(cmds[DD_RIGHT].direction, 0);
}

// ---------------------------------------------------------------------------
// proc_twist — command history (ECU feeds history used by proc_odom)
// ---------------------------------------------------------------------------

TEST_F(DifferentialCmdProcTest, ProcTwistPopulatesCommandHistory)
{
  EXPECT_TRUE(diff_->command_history_[DD_LEFT].empty());
  EXPECT_TRUE(diff_->command_history_[DD_RIGHT].empty());

  diff_->proc_twist(make_twist(1.0, 0.0));

  EXPECT_EQ(diff_->command_history_[DD_LEFT].size(),  1u);
  EXPECT_EQ(diff_->command_history_[DD_RIGHT].size(), 1u);
}

TEST_F(DifferentialCmdProcTest, ProcTwistMultipleCallsGrowHistory)
{
  diff_->proc_twist(make_twist(1.0, 0.0));
  diff_->proc_twist(make_twist(0.5, 0.0));
  diff_->proc_twist(make_twist(0.0, 1.0));

  EXPECT_EQ(diff_->command_history_[DD_LEFT].size(),  3u);
  EXPECT_EQ(diff_->command_history_[DD_RIGHT].size(), 3u);
}

TEST_F(DifferentialCmdProcTest, ProcTwistCommandTtlSetInFuture)
{
  uint64_t before_ns = static_cast<uint64_t>(node_->now().nanoseconds());
  auto cmds = diff_->proc_twist(make_twist(1.0, 0.0));
  uint64_t after_ns  = static_cast<uint64_t>(node_->now().nanoseconds());

  // TTL must be strictly after the command was issued
  EXPECT_GT(cmds[DD_LEFT].ttl_ns,  before_ns);
  EXPECT_GT(cmds[DD_RIGHT].ttl_ns, before_ns);

  // TTL must be approximately now + kTtlNs (allow 10ms clock jitter)
  EXPECT_NEAR(
    static_cast<double>(cmds[DD_LEFT].ttl_ns),
    static_cast<double>(after_ns + kTtlNs),
    10000000.0);  // 10 ms in ns
}

// ---------------------------------------------------------------------------
// proc_odom — message structure
// ---------------------------------------------------------------------------

TEST_F(DifferentialCmdProcTest, ProcOdomFrameIdIsOdom)
{
  auto odom = odom_tick();
  EXPECT_EQ(odom.header.frame_id, "odom");
}

TEST_F(DifferentialCmdProcTest, ProcOdomChildFrameIdIsBaseLink)
{
  auto odom = odom_tick();
  EXPECT_EQ(odom.child_frame_id, "base_link");
}

TEST_F(DifferentialCmdProcTest, ProcOdomStampIsNonZero)
{
  auto odom = odom_tick();
  int64_t stamp_ns = rclcpp::Time(odom.header.stamp).nanoseconds();
  EXPECT_GT(stamp_ns, 0);
}

TEST_F(DifferentialCmdProcTest, ProcOdomFirstCallPoseIsOrigin)
{
  // First call only seeds last_update_ns_ — pose should remain at origin.
  auto odom = odom_tick();
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.z, 0.0);
}

TEST_F(DifferentialCmdProcTest, ProcOdomFirstCallInitializesLastUpdateNs)
{
  EXPECT_EQ(diff_->last_update_ns_, 0u);
  odom_tick();
  EXPECT_GT(diff_->last_update_ns_, 0u);
}

TEST_F(DifferentialCmdProcTest, ProcOdomCovarianceZeroByDefault)
{
  auto odom = odom_tick();
  for (auto v : odom.pose.covariance) {
    EXPECT_DOUBLE_EQ(v, 0.0);
  }
}

// ---------------------------------------------------------------------------
// ECU integration loop — simulates the rr_motor_ecu_node control cycle:
//
//   1. cmd_vel subscriber fires  → proc_twist() → MotorCommands sent to wheels
//   2. Odometry timer fires      → proc_odom()  → Odometry published on /odom
//
// Real wall-clock sleeps are used because update() integrates against the
// steady clock via node_->now(). Direction/sign checks are used rather than
// exact values to avoid flakiness from scheduling jitter.
// ---------------------------------------------------------------------------

TEST_F(DifferentialCmdProcTest, EcuLoopForwardMotionPositiveX)
{
  odom_tick();                                      // seed last_update_ns_
  diff_->proc_twist(make_twist(1.0, 0.0));          // ECU receives /cmd_vel
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  auto odom = odom_tick();                          // ECU timer fires

  EXPECT_GT(odom.pose.pose.position.x, 0.0);
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, kTolerance);
}

TEST_F(DifferentialCmdProcTest, EcuLoopBackwardMotionNegativeX)
{
  odom_tick();
  diff_->proc_twist(make_twist(-1.0, 0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  auto odom = odom_tick();

  EXPECT_LT(odom.pose.pose.position.x, 0.0);
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, kTolerance);
}

TEST_F(DifferentialCmdProcTest, EcuLoopCCWRotationIncreasesTheta)
{
  // angular.z > 0 → CCW → theta_ should increase
  odom_tick();
  diff_->proc_twist(make_twist(0.0, 1.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  odom_tick();

  EXPECT_GT(diff_->theta_, 0.0);
}

TEST_F(DifferentialCmdProcTest, EcuLoopCWRotationDecreasesTheta)
{
  // angular.z < 0 → CW → theta_ should decrease
  odom_tick();
  diff_->proc_twist(make_twist(0.0, -1.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  odom_tick();

  EXPECT_LT(diff_->theta_, 0.0);
}

TEST_F(DifferentialCmdProcTest, EcuLoopStraightMotionLeavesYNearZero)
{
  odom_tick();
  diff_->proc_twist(make_twist(1.0, 0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  auto odom = odom_tick();

  // No angular component — robot should track along x-axis
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, kTolerance);
  EXPECT_NEAR(diff_->theta_, 0.0, kTolerance);
}

TEST_F(DifferentialCmdProcTest, EcuLoopLinearVelocitySetAfterForwardMotion)
{
  odom_tick();
  diff_->proc_twist(make_twist(1.0, 0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  auto odom = odom_tick();

  EXPECT_GT(odom.twist.twist.linear.x, 0.0);
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.0, kTolerance);
}

TEST_F(DifferentialCmdProcTest, EcuLoopAngularVelocitySetAfterCCWRotation)
{
  odom_tick();
  diff_->proc_twist(make_twist(0.0, 1.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  auto odom = odom_tick();

  EXPECT_GT(odom.twist.twist.angular.z, 0.0);
}

TEST_F(DifferentialCmdProcTest, EcuLoopHistoryPrunedToAtLeastOneEntry)
{
  // Use a 1 ns TTL so commands expire immediately.
  diff_->ttl_ns_ = 1ULL;

  odom_tick();
  diff_->proc_twist(make_twist(1.0, 0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  odom_tick();   // prune_history() is called inside update()

  // prune_history always keeps at least one command as the active baseline
  EXPECT_GE(diff_->command_history_[DD_LEFT].size(),  1u);
  EXPECT_GE(diff_->command_history_[DD_RIGHT].size(), 1u);
}

TEST_F(DifferentialCmdProcTest, EcuLoopMultipleTwistsCommandHistoryGrows)
{
  // ECU node may issue many successive commands before the odom timer fires.
  odom_tick();
  diff_->proc_twist(make_twist(1.0, 0.0));
  diff_->proc_twist(make_twist(0.8, 0.2));
  diff_->proc_twist(make_twist(0.5, 0.5));

  EXPECT_EQ(diff_->command_history_[DD_LEFT].size(),  3u);
  EXPECT_EQ(diff_->command_history_[DD_RIGHT].size(), 3u);
}

TEST_F(DifferentialCmdProcTest, EcuLoopSecondOdomCycleAccumulatesPose)
{
  // Two complete ECU cycles: pose should accumulate across both.
  odom_tick();
  diff_->proc_twist(make_twist(1.0, 0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  odom_tick();   // cycle 1 — x_ now positive

  double x_after_first = diff_->x_;
  EXPECT_GT(x_after_first, 0.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  odom_tick();   // cycle 2 — x_ should be larger still

  EXPECT_GT(diff_->x_, x_after_first);
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

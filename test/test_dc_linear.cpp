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
#include "rr_motor_controller/dc_linear.hpp"

using DutyLR = rr_motor_controller::DutyConvertorLinearRegression;

class DcLinearTest : public ::testing::Test {
 protected:
  DutyLR dc_;
};

// SLOPE = 118.5, OFFSET = -5.8, MIN = 0.0, MAX = 100.0

TEST_F(DcLinearTest, ZeroSetpointClampedToMin) {
  // 118.5 * 0 + (-5.8) = -5.8 → clamped to 0
  EXPECT_DOUBLE_EQ(dc_.compute(0.0, 0.0, 100), 0.0);
}

TEST_F(DcLinearTest, ModerateSetpointLinearOutput) {
  // 118.5 * 0.5 + (-5.8) = 53.45
  EXPECT_DOUBLE_EQ(dc_.compute(0.5, 0.0, 100), 53.45);
}

TEST_F(DcLinearTest, HighSetpointClampedToMax) {
  // 118.5 * 1.0 + (-5.8) = 112.7 → clamped to 100
  EXPECT_DOUBLE_EQ(dc_.compute(1.0, 0.0, 100), 100.0);
}

TEST_F(DcLinearTest, NegativeSetpointClampedToMin) {
  // 118.5 * (-1.0) + (-5.8) = -124.3 → clamped to 0
  EXPECT_DOUBLE_EQ(dc_.compute(-1.0, 0.0, 100), 0.0);
}

TEST_F(DcLinearTest, MeasurementAndDtIgnored) {
  double base = dc_.compute(0.5, 0.0, 0);
  EXPECT_DOUBLE_EQ(dc_.compute(0.5, 99.9, 0), base);
  EXPECT_DOUBLE_EQ(dc_.compute(0.5, 0.0, 5000), base);
  EXPECT_DOUBLE_EQ(dc_.compute(0.5, 42.0, 9999), base);
}

TEST_F(DcLinearTest, ExactMinBoundary) {
  // Find setpoint where output is exactly 0: 118.5 * x - 5.8 = 0 → x = 5.8/118.5
  double threshold = DutyLR::OFFSET / -DutyLR::SLOPE;  // negative because OFFSET is -5.8
  EXPECT_DOUBLE_EQ(dc_.compute(threshold, 0.0, 100), 0.0);
  // Slightly below → still 0 (clamped)
  EXPECT_DOUBLE_EQ(dc_.compute(threshold - 0.01, 0.0, 100), 0.0);
}

TEST_F(DcLinearTest, ExactMaxBoundary) {
  // Find setpoint where output is exactly 100: 118.5 * x - 5.8 = 100 → x = 105.8/118.5
  double threshold = (DutyLR::MAX_VAL - DutyLR::OFFSET) / DutyLR::SLOPE;
  double result = dc_.compute(threshold, 0.0, 100);
  EXPECT_NEAR(result, 100.0, 1e-10);
  // Slightly above → still 100 (clamped)
  EXPECT_DOUBLE_EQ(dc_.compute(threshold + 0.1, 0.0, 100), 100.0);
}

TEST_F(DcLinearTest, SmallPositiveSetpoint) {
  // 118.5 * 0.05 + (-5.8) = 5.925 - 5.8 = 0.125
  EXPECT_NEAR(dc_.compute(0.05, 0.0, 100), 0.125, 1e-10);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
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

namespace rr_motor_controller
{
    /**
     * Provide a conversion routine from m/s to duty cycles. Note that this 
     * can be a linear regression conversion, or something more complex such as a PID.
     * 
     * It will be up to the motor controller to decide what to use.
     */
    class DutyConversion {
        public:
        /**
         * @fn compute
         * @brief converts m/s to a duty cycle.
         * @param setpoint expected velocity m/s
         * @param measurement current velocity m/s
         * @param dt time since last sample in milliseconds.
         * @return duty cyle to achieve this
         * 
         * Note that for linear regression algorithms the mesurement and dt, is not needed
         * but should be included to allow a smooth conversion to PID algorithms once
         * imperial evidence has been taken, and motor tolerence under load is known.
         */
        virtual double compute(double setpoint, double measurement, int64_t dt) = 0;
    };
}
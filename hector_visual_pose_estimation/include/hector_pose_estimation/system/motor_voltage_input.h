//=================================================================================================
// Copyright (c) 2013, Thorsten Graber, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_pose_estimation/system_input.h>
#include <hector_pose_estimation/matrix.h>
#include <hector_uav_msgs/MotorStatus.h>

#ifndef HECTOR_POSE_ESTIMATION_MOTOR_VOLTAGE_INPUT_H
#define HECTOR_POSE_ESTIMATION_MOTOR_VOLTAGE_INPUT_H

namespace hector_pose_estimation {

class MotorVoltageInput : public SystemInput
{
public:
    enum InputIndex {
        VOLTAGE_0 = 1,
        VOLTAGE_1,
        VOLTAGE_2,
        VOLTAGE_3
    };
    static const unsigned int InputDimension = VOLTAGE_3;
    typedef ColumnVector_<InputDimension> InputVector;

    MotorVoltageInput()
        : u_(InputDimension)
    {}
    MotorVoltageInput(InputVector const& u)
        : u_(InputDimension)
    {
        setValue(u);
    }
    MotorVoltageInput(const hector_uav_msgs::MotorStatus& status)
        : u_(InputDimension)
    {
        setValue(status);
    }
    virtual ~MotorVoltageInput() {}

    virtual void setValue(InputVector const& u) { u_ = u; }
    virtual void setValue(const hector_uav_msgs::MotorStatus& status) {
        if(status.voltage.size() == 4)
        {
            u_(VOLTAGE_0) = status.voltage[0];
            u_(VOLTAGE_1) = status.voltage[1];
            u_(VOLTAGE_2) = status.voltage[2];
            u_(VOLTAGE_3) = status.voltage[3];
        }
        else
        {
            u_(VOLTAGE_0) = 0.0;
            u_(VOLTAGE_1) = 0.0;
            u_(VOLTAGE_2) = 0.0;
            u_(VOLTAGE_3) = 0.0;
            ROS_WARN("Motor voltage vector size has to be 4, but it is %ld ! Use 0V as motor voltrages!",status.voltage.size());
        }
    }

    virtual InputVector const &getVector() const { return u_; }
    virtual ColumnVector_<4> getVoltage() const { return u_.sub(VOLTAGE_0, VOLTAGE_3); }

    virtual InputVector &operator=(InputVector const& u) { setValue(u); return u_; }
    virtual InputVector &operator=(const hector_uav_msgs::MotorStatus& status) { setValue(status); return u_; }

protected:
    InputVector u_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MOTOR_VOLTAGE_INPUT_H

//=================================================================================================
// Copyright (c) 2013, Thorsten Graber and Johannes Meyer and Alexander Sendobry, TU Darmstadt
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

#ifndef HECTOR_QUADROTOR_MODEL_H
#define HECTOR_QUADROTOR_MODEL_H

#include <geometry_msgs/Wrench.h>
#include <hector_uav_msgs/MotorPWM.h>
#include <hector_uav_msgs/Supply.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <nav_msgs/Odometry.h>

namespace hector_quadrotor_model{

template <class Derived>
struct Types_ {
  struct State;
  struct Input;
  struct Output;
  struct Parameters;
};

template <class Derived>
class SystemModel {
public:
    // all Derived classes have to provide the following nested types:
    typedef typename Types_<Derived>::State State;
    typedef typename Types_<Derived>::Input Input;
    typedef typename Types_<Derived>::Output Output;
    typedef typename Types_<Derived>::Parameters Parameters;

    // all Derived classes have to provide the following static function:
    // static void Derived::f(State& x_pred, const State& x, const double dt,
    //               const Input& input,
    //               Output& output,
    //               const Parameters& parameters);

    /* constructors */
    SystemModel(const State& initial_state = State()) : x_(initial_state) {}
    virtual ~SystemModel() {}

    /* system update (prediction) */
    virtual void update(const ros::Time& stamp, const Input& u) {
        State x_pred;
        double dt = (stamp - stamp_).toSec();
        if (stamp_ == ros::Time()) dt = 0.0;

        Derived::f(x_pred, x_, dt, u, y_, parameters_);
        x_ = x_pred;
        stamp_ = stamp;
    }

    /* system update (prediction) */
    virtual void update(const double& dt, const Input& u) {
        State x_pred;
        Derived::f(x_pred, x_, dt, u, y_, parameters_);
        x_ = x_pred;
    }

    /* reset model */
    virtual void reset() {
        x_ = State();
        y_ = Output();
        stamp_ = ros::Time();
    }

    /* getter/setter functions */
    virtual State& x() { return x_; }
    virtual const State& x() const { return x_; }
    virtual Output& y() { return y_; }
    virtual const Output& y() const { return y_; }
    virtual Parameters& parameters() { return parameters_; }
    virtual const Parameters& parameters() const { return parameters_; }
    virtual void setTimestamp(ros::Time& time) { stamp_ = time; }
    virtual const ros::Time& getTimestamp() const { return stamp_; }

    // add more getters in the Derived class...

protected:
    State x_;
    Output y_;
    Parameters parameters_;

private:
    ros::Time stamp_;
};

class Propulsion;
template <> struct Types_<Propulsion> {
    typedef boost::array<double,4> State;

    struct Input {
        hector_uav_msgs::MotorStatus motor_status;
        geometry_msgs::Twist twist;
    };

    struct Output {
        geometry_msgs::Wrench wrench;
        hector_uav_msgs::MotorStatus motor_status;
    };

    struct Parameters {
        double k_t;
        double CT2s;
        double CT1s;
        double CT0s;
        double Psi;
        double J_M;
        double R_A;
        double l_m;
    };
};

class Propulsion : public SystemModel<Propulsion> {
public:
    static void f(State& x_pred, const State& x, const double dt,
                  const Input& input,
                  Output& output,
                  const Parameters& parameters);

    using SystemModel<Propulsion>::update;
    void update(const double& dt, const hector_uav_msgs::MotorStatus& motor_status, const geometry_msgs::Twist& twist) {
        Input u;
        u.motor_status = motor_status;
        u.twist = twist;
        update(dt, u);
    }
    void update(const ros::Time& stamp, const hector_uav_msgs::MotorStatus& motor_status, const geometry_msgs::Twist& twist) {
        Input u;
        u.motor_status = motor_status;
        u.twist = twist;
        update(stamp, u);
    }
    static void motorspeed(double &x_pred, const double &x, const double &dt, const double &input_voltage, const double &input_torque, double &output, const Parameters &parameters);

    const hector_uav_msgs::MotorStatus& getMotorStatus() const { return y_.motor_status; }
    const geometry_msgs::Wrench& getWrench() const { return y_.wrench; }
};

class Aerodynamics;
template <> struct Types_<Aerodynamics> {
    struct State {};

    struct Input {
        geometry_msgs::Twist twist;
        geometry_msgs::Vector3 wind;
    };

    struct Output {
        geometry_msgs::Wrench wrench;
    };

    struct Parameters {
        double C_wxy;
        double C_wz;
        double C_mxy;
        double C_mz;
    };
};

class Aerodynamics : public SystemModel<Aerodynamics> {
public:
    static void f(State& x_pred, const State& x, const double dt,
                  const Input& input,
                  Output& output,
                  const Parameters& parameters);

    using SystemModel::update;
    void update(const double& dt, const geometry_msgs::Twist& twist, const geometry_msgs::Vector3& wind) {
        Input u;
        u.twist = twist;
        u.wind = wind;
        update(dt, u);
    }

    void update(const ros::Time& stamp, const geometry_msgs::Twist& twist, const geometry_msgs::Vector3& wind) {
        Input u;
        u.twist = twist;
        u.wind = wind;
        update(stamp, u);
    }

    const geometry_msgs::Wrench& getWrench() const { return y_.wrench; }
};

class Quadrotor;
template <> struct Types_<Quadrotor> {
    struct State {
        Propulsion::State propulsion;
        Aerodynamics::State aerodynamics;
    };

    struct Input {
        hector_uav_msgs::MotorStatus motor_status;
        geometry_msgs::Twist twist;
        geometry_msgs::Vector3 wind;
    };

    struct Output {
        geometry_msgs::Wrench wrench;
        hector_uav_msgs::MotorStatus motor_status;
    };

    struct Parameters {
        Propulsion::Parameters propulsion;
        Aerodynamics::Parameters aerodynamics;
        double gravity;
        double m;
        double I_x;
        double I_y;
        double I_z;
    };
};

class Quadrotor : public SystemModel<Quadrotor> {
public:
    static void f(State& x_pred, const State& x, const double dt,
                  const Input& input,
                  Output& output,
                  const Parameters& parameters);

    using SystemModel::update;
    void update(const double& dt, const hector_uav_msgs::MotorStatus& status, const geometry_msgs::Twist& twist, const geometry_msgs::Vector3& wind) {
        Input u;
        u.motor_status.voltage = status.voltage;
        u.twist = twist;
        u.wind = wind;
        update(dt, u);
    }

    void update(const ros::Time& time, const hector_uav_msgs::MotorStatus& status, const geometry_msgs::Twist& twist, const geometry_msgs::Vector3& wind) {
        Input u;
        u.motor_status.voltage = status.voltage;
        u.twist = twist;
        u.wind = wind;
        update(time, u);
    }

    const hector_uav_msgs::MotorStatus& getMotorStatus() const { return y_.motor_status; }
    const geometry_msgs::Wrench& getWrench() const { return y_.wrench; }
};

}

#endif

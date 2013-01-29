//=================================================================================================
// Copyright (c) 2013, Thorsten Graber, Johannes Meyer, Alexander Sendobry and Martin Nowara, TU Darmstadt
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

#include <ros/ros.h>
#include <hector_quadrotor_model/quadrotor_model.h>
#include <hector_pose_estimation/system/quadrotor_system_model.h>

namespace hector_pose_estimation {

QuadrotorSystemModel::QuadrotorSystemModel()
{
    /// initialize quadrotor model
    quad_model = new hector_quadrotor_model::Quadrotor();
    // environment parameters
    parameters().add("gravity", quad_model->parameters().gravity);
    // quadrotor parameters
    parameters().add("m", quad_model->parameters().m);
    parameters().add("I_x", quad_model->parameters().I_x);
    parameters().add("I_y", quad_model->parameters().I_y);
    parameters().add("I_z", quad_model->parameters().I_z);
    // quadrotor aerodynamics parameters
    parameters().add("C_wxy", quad_model->parameters().aerodynamics.C_wxy);
    parameters().add("C_wz", quad_model->parameters().aerodynamics.C_wz);
    parameters().add("C_mxy", quad_model->parameters().aerodynamics.C_mxy);
    parameters().add("C_mz", quad_model->parameters().aerodynamics.C_mz);
    // quadrotor propulsion parameters
    parameters().add("k_t", quad_model->parameters().propulsion.k_t);
    parameters().add("CT2s", quad_model->parameters().propulsion.CT2s);
    parameters().add("CT1s", quad_model->parameters().propulsion.CT1s);
    parameters().add("CT0s", quad_model->parameters().propulsion.CT0s);
    parameters().add("Psi", quad_model->parameters().propulsion.Psi);
    parameters().add("J_M", quad_model->parameters().propulsion.J_M);
    parameters().add("R_A", quad_model->parameters().propulsion.R_A);
    parameters().add("l_m", quad_model->parameters().propulsion.l_m);


    rate_stddev_ = 0.0;
    angular_acceleration_stddev_ = 10.0 * M_PI/180.0;
    acceleration_stddev_ = 1.0e-2;
    velocity_stddev_ = 0.0;
    voltage_stddev_ = sqrt(2.0)*1.0e-2;
    parameters().add("rate_stddev", rate_stddev_);
    parameters().add("angular_acceleration_stddev", angular_acceleration_stddev_);
    parameters().add("acceleration_stddev", acceleration_stddev_);
    parameters().add("velocity_stddev", velocity_stddev_);
    parameters().add("voltage_stddev", voltage_stddev_);
}

bool QuadrotorSystemModel::init()
{
    noise_ = 0.0;
    noise_(QUATERNION_W,QUATERNION_W) = noise_(QUATERNION_X,QUATERNION_X) = noise_(QUATERNION_Y,QUATERNION_Y) = noise_(QUATERNION_Z,QUATERNION_Z) = pow(0.5 * rate_stddev_, 2); // will be overridden in CovarianceGet() !
    noise_(RATE_X,RATE_X) = noise_(RATE_Y,RATE_Y) = noise_(RATE_Z,RATE_Z) = pow(angular_acceleration_stddev_, 2);
    noise_(POSITION_X,POSITION_X) = noise_(POSITION_Y,POSITION_Y) = noise_(POSITION_Z,POSITION_Z) = pow(velocity_stddev_, 2);
    noise_(VELOCITY_X,VELOCITY_X) = noise_(VELOCITY_Y,VELOCITY_Y) = noise_(VELOCITY_Z,VELOCITY_Z) = pow(acceleration_stddev_, 2);
    this->AdditiveNoiseSigmaSet(noise_);

    // input noise ?

    return true;
}

QuadrotorSystemModel::~QuadrotorSystemModel()
{
    delete quad_model;
}

SystemStatus QuadrotorSystemModel::getStatusFlags() const
{
    SystemStatus flags = measurement_status_;
    //     flags |= STATE_XY_POSITION | STATE_Z_POSITION;
    if (flags & STATE_XY_POSITION) flags |= STATE_XY_VELOCITY;
    if (flags & STATE_Z_POSITION)  flags |= STATE_Z_VELOCITY;
    if (flags & STATE_XY_VELOCITY) flags |= STATE_ROLLPITCH;
    if (flags & STATE_ROLLPITCH) flags   |= STATE_XY_VELOCITY;
    return flags;
}

//--> System equation of this model xpred = x_(k+1) = f(x,u)
ColumnVector QuadrotorSystemModel::ExpectedValueGet(double dt) const
{
    /// predict state x based on wrench
    double m          = quad_model->parameters().m;
    double fbx        = quad_model->y().wrench.force.x;
    double fby        = quad_model->y().wrench.force.y;
    double fbz        = quad_model->y().wrench.force.z;
    double I_x        = quad_model->parameters().I_x;
    double I_y        = quad_model->parameters().I_y;
    double I_z        = quad_model->parameters().I_z;
    double tbx        = quad_model->y().wrench.torque.x;
    double tby        = quad_model->y().wrench.torque.y;
    double tbz        = quad_model->y().wrench.torque.z;


    double wbx        = x_(RATE_X);
    double wby        = x_(RATE_Y);
    double wbz        = x_(RATE_Z);
    q0                = x_(QUATERNION_W);
    q1                = x_(QUATERNION_X);
    q2                = x_(QUATERNION_Y);
    q3                = x_(QUATERNION_Z);
    double pnx        = x_(POSITION_X);
    double pny        = x_(POSITION_Y);
    double pnz        = x_(POSITION_Z);
    double vnx        = x_(VELOCITY_X);
    double vny        = x_(VELOCITY_Y);
    double vnz        = x_(VELOCITY_Z);
    double vbx        = (q0*q0+q1*q1-q2*q2-q3*q3)*vnx + (2.0*q1*q2+2.0*q0*q3)    *vny + (2.0*q1*q3-2.0*q0*q2)    *vnz;
    double vby        = (2.0*q1*q2-2.0*q0*q3)    *vnx + (q0*q0-q1*q1+q2*q2-q3*q3)*vny + (2.0*q2*q3+2.0*q0*q1)    *vnz;
    double vbz        = (2.0*q1*q3+2.0*q0*q2)    *vnx + (2.0*q2*q3-2.0*q0*q1)    *vny + (q0*q0-q1*q1-q2*q2+q3*q3)*vnz;

    x_pred_ = x_;

    // Angular Velocity
    x_pred_(RATE_X)          = wbx + dt*((tbx - wby*wbz*(I_z-I_y))/I_x);
    x_pred_(RATE_Y)          = wby + dt*((tby - wbz*wbx*(I_x-I_z))/I_y);
    x_pred_(RATE_Z)          = wbz + dt*((tbz - wbx*wby*(I_y-I_x))/I_z);

    // Atitude
    x_pred_(QUATERNION_W)    = q0 + dt*0.5*(          (-wbx)*q1+(-wby)*q2+(-wbz)*q3);
    x_pred_(QUATERNION_X)    = q1 + dt*0.5*(( wbx)*q0          +(-wbz)*q2+( wby)*q3);
    x_pred_(QUATERNION_Y)    = q2 + dt*0.5*(( wby)*q0+( wbz)*q1          +(-wbx)*q3);
    x_pred_(QUATERNION_Z)    = q3 + dt*0.5*(( wbz)*q0+(-wby)*q1+( wbx)*q2          );

    // Velocity (without coriolis forces) and Position
    if (getStatusFlags() & STATE_XY_VELOCITY) {
        x_pred_(VELOCITY_X)  = vnx + dt*(((q0*q0+q1*q1-q2*q2-q3*q3)*fbx + (2.0*q1*q2-2.0*q0*q3)    *fby + (2.0*q1*q3+2.0*q0*q2)    *fbz)/m + vby*wbz-vbz*wby);
        x_pred_(VELOCITY_Y)  = vny + dt*(((2.0*q1*q2+2.0*q0*q3)    *fbx + (q0*q0-q1*q1+q2*q2-q3*q3)*fby + (2.0*q2*q3-2.0*q0*q1)    *fbz)/m + vbz*wbx-vbx*wbz);
    }
    if (getStatusFlags() & STATE_Z_VELOCITY) {
        x_pred_(VELOCITY_Z)  = vnz + dt*(((2.0*q1*q3-2.0*q0*q2)    *fbx + (2.0*q2*q3+2.0*q0*q1)    *fby + (q0*q0-q1*q1-q2*q2+q3*q3)*fbz)/m + vbx*wby-vby*wbx + quad_model->parameters().gravity);
    }

    if (getStatusFlags() & STATE_XY_POSITION) {
        x_pred_(POSITION_X)  = pnx + dt*(vnx);
        x_pred_(POSITION_Y)  = pny + dt*(vny);
    }
    if (getStatusFlags() & STATE_Z_POSITION) {
        x_pred_(POSITION_Z)  = pnz + dt*(vnz);
    }

    return x_pred_ + AdditiveNoiseMuGet(); // + noise?
}

//--> Covariance
// Warning: CovarianceGet() must be called AFTER ExpectedValueGet(...) or dfGet(...)
// unfortunately MatrixWrapper::SymmetricMatrix CovarianceGet(const MatrixWrapper::ColumnVector& u, const MatrixWrapper::ColumnVector& x) cannot be overridden
SymmetricMatrix QuadrotorSystemModel::CovarianceGet(double dt) const
{
    double rate_variance_4 = 0.25 * pow(rate_stddev_, 2);
    noise_(QUATERNION_W,QUATERNION_W) = rate_variance_4 * (q1*q1+q2*q2+q3*q3);
    noise_(QUATERNION_X,QUATERNION_X) = rate_variance_4 * (q0*q0+q2*q2+q3*q3);
    noise_(QUATERNION_Y,QUATERNION_Y) = rate_variance_4 * (q0*q0+q1*q1+q3*q3);
    noise_(QUATERNION_Z,QUATERNION_Z) = rate_variance_4 * (q0*q0+q1*q1+q2*q2);
    // return noise_ * (dt*dt);
    return noise_ * dt;
}

//--> Jacobian matrices A
Matrix QuadrotorSystemModel::dfGet(unsigned int i, double dt) const
{
    q0     = x_(QUATERNION_W);
    q1     = x_(QUATERNION_X);
    q2     = x_(QUATERNION_Y);
    q3     = x_(QUATERNION_Z);

    /// calculate body acceleration and body rate based on motor voltages

    // calculate wrench based on motor voltages, velocity (and wind)
    hector_uav_msgs::MotorStatus status;
    status.voltage[0]             = u_(MotorVoltageInput::VOLTAGE_0);
    status.voltage[1]             = u_(MotorVoltageInput::VOLTAGE_1);
    status.voltage[2]             = u_(MotorVoltageInput::VOLTAGE_2);
    status.voltage[3]             = u_(MotorVoltageInput::VOLTAGE_3);
    geometry_msgs::Vector3 nav_vel;
    nav_vel.x                     = x_(VELOCITY_X);
    nav_vel.y                     = x_(VELOCITY_Y);
    nav_vel.z                     = x_(VELOCITY_Z);
    geometry_msgs::Twist twist;
    twist.linear                  = getBodyVel(nav_vel);
    twist.angular.x               = x_(RATE_X);
    twist.angular.y               = x_(RATE_Y);
    twist.angular.z               = x_(RATE_Z);
    geometry_msgs::Vector3 v_wind;
    v_wind.x                      = 0.0;
    v_wind.y                      = 0.0;
    v_wind.z                      = 0.0;
    quad_model->x().propulsion[0] = x_internal_(1);
    quad_model->x().propulsion[1] = x_internal_(2);
    quad_model->x().propulsion[2] = x_internal_(3);
    quad_model->x().propulsion[3] = x_internal_(4);
    quad_model->update(dt,status,twist,v_wind);
    x_pred_internal_(1)           = quad_model->x().propulsion[0];
    x_pred_internal_(2)           = quad_model->x().propulsion[1];
    x_pred_internal_(3)           = quad_model->x().propulsion[2];
    x_pred_internal_(4)           = quad_model->x().propulsion[3];

    // calculate body acceleration and body rate based on wrench
    double m          = quad_model->parameters().m;
    double fbx        = quad_model->y().wrench.force.x;
    double fby        = quad_model->y().wrench.force.y;
    double fbz        = quad_model->y().wrench.force.z;
    double I_x        = quad_model->parameters().I_x;
    double I_y        = quad_model->parameters().I_y;
    double I_z        = quad_model->parameters().I_z;
    double tbx        = quad_model->y().wrench.torque.x;
    double tby        = quad_model->y().wrench.torque.y;
    double tbz        = quad_model->y().wrench.torque.z;

    double wbx        = x_(RATE_X);
    double wby        = x_(RATE_Y);
    double wbz        = x_(RATE_Z);
    q0                = x_(QUATERNION_W);
    q1                = x_(QUATERNION_X);
    q2                = x_(QUATERNION_Y);
    q3                = x_(QUATERNION_Z);
    double pnx        = x_(POSITION_X);
    double pny        = x_(POSITION_Y);
    double pnz        = x_(POSITION_Z);
    double vnx        = x_(VELOCITY_X);
    double vny        = x_(VELOCITY_Y);
    double vnz        = x_(VELOCITY_Z);
    double vbx        = (q0*q0+q1*q1-q2*q2-q3*q3)*vnx + (2.0*q1*q2+2.0*q0*q3)    *vny + (2.0*q1*q3-2.0*q0*q2)    *vnz;
    double vby        = (2.0*q1*q2-2.0*q0*q3)    *vnx + (q0*q0-q1*q1+q2*q2-q3*q3)*vny + (2.0*q2*q3+2.0*q0*q1)    *vnz;
    double vbz        = (2.0*q1*q3+2.0*q0*q2)    *vnx + (2.0*q2*q3-2.0*q0*q1)    *vny + (q0*q0-q1*q1-q2*q2+q3*q3)*vnz;
/*
    if (i == 1)
    {
        // set jacobian matrix A
        A_(QUATERNION_W,QUATERNION_X) = dt*(-0.5*wbx);
        A_(QUATERNION_W,QUATERNION_Y) = dt*(-0.5*wby);
        A_(QUATERNION_W,QUATERNION_Z) = dt*(-0.5*wbz);
        A_(QUATERNION_W,RATE_X)       = -0.5*dt*q1;
        A_(QUATERNION_W,RATE_Y)       = -0.5*dt*q2;
        A_(QUATERNION_W,RATE_Z)       = -0.5*dt*q3;

        A_(QUATERNION_X,QUATERNION_W) = dt*( 0.5*wbx);
        A_(QUATERNION_X,QUATERNION_Y) = dt*( 0.5*wbz);
        A_(QUATERNION_X,QUATERNION_Z) = dt*(-0.5*wby);
        A_(QUATERNION_X,RATE_X)       =  0.5*dt*q0;
        A_(QUATERNION_X,RATE_Y)       =  0.5*dt*q3;
        A_(QUATERNION_X,RATE_Z)       = -0.5*dt*q2;

        A_(QUATERNION_Y,QUATERNION_W) = dt*( 0.5*wby);
        A_(QUATERNION_Y,QUATERNION_X) = dt*(-0.5*wbz);
        A_(QUATERNION_Y,QUATERNION_Z) = dt*( 0.5*wbx);
        A_(QUATERNION_Y,RATE_X)       = -0.5*dt*q3;
        A_(QUATERNION_Y,RATE_Y)       =  0.5*dt*q0;
        A_(QUATERNION_Y,RATE_Z)       =  0.5*dt*q1;

        A_(QUATERNION_Z,QUATERNION_W) = dt*( 0.5*wbz);
        A_(QUATERNION_Z,QUATERNION_X) = dt*( 0.5*wby);
        A_(QUATERNION_Z,QUATERNION_Y) = dt*(-0.5*wbx);
        A_(QUATERNION_Z,RATE_X)       =  0.5*dt*q2;
        A_(QUATERNION_Z,RATE_Y)       = -0.5*dt*q1;
        A_(QUATERNION_Z,RATE_Z)       =  0.5*dt*q0;

        if (getStatusFlags() & STATE_XY_VELOCITY) {
            A_(VELOCITY_X,QUATERNION_W) = dt*(-2.0*q3*aby+2.0*q2*abz+2.0*q0*abx);
            A_(VELOCITY_X,QUATERNION_X) = dt*( 2.0*q2*aby+2.0*q3*abz+2.0*q1*abx);
            A_(VELOCITY_X,QUATERNION_Y) = dt*(-2.0*q2*abx+2.0*q1*aby+2.0*q0*abz);
            A_(VELOCITY_X,QUATERNION_Z) = dt*(-2.0*q3*abx-2.0*q0*aby+2.0*q1*abz);

            A_(VELOCITY_Y,QUATERNION_W) = dt*(2.0*q3*abx-2.0*q1*abz+2.0*q0*aby);
            A_(VELOCITY_Y,QUATERNION_X) = dt*(2.0*q2*abx-2.0*q1*aby-2.0*q0*abz);
            A_(VELOCITY_Y,QUATERNION_Y) = dt*(2.0*q1*abx+2.0*q3*abz+2.0*q2*aby);
            A_(VELOCITY_Y,QUATERNION_Z) = dt*(2.0*q0*abx-2.0*q3*aby+2.0*q2*abz);

        } else {
            A_(VELOCITY_X,QUATERNION_W) = 0.0;
            A_(VELOCITY_X,QUATERNION_X) = 0.0;
            A_(VELOCITY_X,QUATERNION_Y) = 0.0;
            A_(VELOCITY_X,QUATERNION_Z) = 0.0;

            A_(VELOCITY_Y,QUATERNION_W) = 0.0;
            A_(VELOCITY_Y,QUATERNION_X) = 0.0;
            A_(VELOCITY_Y,QUATERNION_Y) = 0.0;
            A_(VELOCITY_Y,QUATERNION_Z) = 0.0;
        }

        if (getStatusFlags() & STATE_Z_VELOCITY) {
            A_(VELOCITY_Z,QUATERNION_W) = dt*(-2.0*q2*abx+2.0*q1*aby+2.0*q0*abz);
            A_(VELOCITY_Z,QUATERNION_X) = dt*( 2.0*q3*abx+2.0*q0*aby-2.0*q1*abz);
            A_(VELOCITY_Z,QUATERNION_Y) = dt*(-2.0*q0*abx+2.0*q3*aby-2.0*q2*abz);
            A_(VELOCITY_Z,QUATERNION_Z) = dt*( 2.0*q1*abx+2.0*q2*aby+2.0*q3*abz);

        } else {
            A_(VELOCITY_Z,QUATERNION_W) = 0.0;
            A_(VELOCITY_Z,QUATERNION_X) = 0.0;
            A_(VELOCITY_Z,QUATERNION_Y) = 0.0;
            A_(VELOCITY_Z,QUATERNION_Z) = 0.0;
        }

        if (getStatusFlags() & STATE_XY_POSITION) {
            A_(POSITION_X,VELOCITY_X)   = dt;
            A_(POSITION_Y,VELOCITY_Y)   = dt;
        } else {
            A_(POSITION_X,VELOCITY_X)   = 0.0;
            A_(POSITION_Y,VELOCITY_Y)   = 0.0;
        }

        if (getStatusFlags() & STATE_Z_POSITION) {
            A_(POSITION_Z,VELOCITY_Z)   = dt;
        } else {
            A_(POSITION_Z,VELOCITY_Z)   = 0.0;
        }
        //----------------------------------------------------------

        return A_;
    }

    if(i == 0)
    {
        // set jacobian matrix B

        return B_;
    }
    */
}

geometry_msgs::Vector3 QuadrotorSystemModel::getBodyVel(const geometry_msgs::Vector3 &nav_vel) const {
    geometry_msgs::Vector3 body_vel;
    body_vel.x  = (q0*q0+q1*q1-q2*q2-q3*q3)*nav_vel.x + (2.0*q1*q2+2.0*q0*q3)    *nav_vel.y + (2.0*q1*q3-2.0*q0*q2)    *nav_vel.z;
    body_vel.y  = (2.0*q1*q2-2.0*q0*q3)    *nav_vel.x + (q0*q0-q1*q1+q2*q2-q3*q3)*nav_vel.y + (2.0*q2*q3+2.0*q0*q1)    *nav_vel.z;
    body_vel.z  = (2.0*q1*q3+2.0*q0*q2)    *nav_vel.x + (2.0*q2*q3-2.0*q0*q1)    *nav_vel.y + (q0*q0-q1*q1-q2*q2+q3*q3)*nav_vel.z;

    return body_vel;
}

void QuadrotorSystemModel::Limit(StateVector& x) const {
    normalize(x);
}

void QuadrotorSystemModel::normalize(StateVector& x) {
    double s = 1.0/sqrt(x(QUATERNION_W)*x(QUATERNION_W)+x(QUATERNION_X)*x(QUATERNION_X)+x(QUATERNION_Y)*x(QUATERNION_Y)+x(QUATERNION_Z)*x(QUATERNION_Z));
    x(QUATERNION_W) *= s;
    x(QUATERNION_X) *= s;
    x(QUATERNION_Y) *= s;
    x(QUATERNION_Z) *= s;
}

} // namespace hector_pose_estimation

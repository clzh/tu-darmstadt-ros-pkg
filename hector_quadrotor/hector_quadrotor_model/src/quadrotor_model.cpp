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

#include "hector_quadrotor_model/quadrotor_model.h"

namespace hector_quadrotor_model {

void Aerodynamics::f(State &x_pred, const State &x, const double dt, const Input &input, Output &output, const Parameters &parameters) {
    // parameters
    double C_wxy = parameters.C_wxy;
    double C_wz  = parameters.C_wz;
    double C_mxy = parameters.C_mxy;
    double C_mz  = parameters.C_mz;

    // input
    double u     = input.twist.linear.x - input.wind.x;
    double v     = input.twist.linear.y - input.wind.y;
    double w     = input.twist.linear.z - input.wind.z;
    double p     = input.twist.angular.x;
    double q     = input.twist.angular.y;
    double r     = input.twist.angular.z;

    // temporarily used vector
    double absoluteVelocity        = sqrt(pow(u,2.0) + pow(v,2.0) + pow(w,2.0));
    double absoluteAngularVelocity = sqrt(pow(p,2.0) + pow(q,2.0) + pow(r,2.0));

    // system outputs
    // calculate drag force
    output.wrench.force.x  = -C_wxy* absoluteVelocity*u;
    output.wrench.force.y  = -C_wxy* absoluteVelocity*v;
    output.wrench.force.z  = -C_wz * absoluteVelocity*w;

    // calculate draq torque
    output.wrench.torque.x = -C_mxy* absoluteAngularVelocity*p;
    output.wrench.torque.y = -C_mxy* absoluteAngularVelocity*q;
    output.wrench.torque.z = -C_mz * absoluteAngularVelocity*r;
}


void Propulsion::f(State &x_pred, const State &x, const double dt, const Input &input, Output &output, const Parameters &parameters) {

    boost::array<double,4> U   = {{0.0}};
    boost::array<double,4> v_1 = {{0.0}};
    boost::array<double,4> M_e = {{0.0}};
    boost::array<double,4> F_m = {{0.0}};
    boost::array<double,3> F   = {{0.0}};

    // parameters
    double CT2s = parameters.CT2s;
    double CT1s = parameters.CT1s;
    double CT0s = parameters.CT0s;
    double Psi  = parameters.Psi;
    double k_t  = parameters.k_t;
    double l_m  = parameters.l_m;

    // input variables
    double u    = input.twist.linear.x;
    double v    = input.twist.linear.y;
    double w    = input.twist.linear.z;
    double p    = input.twist.angular.x;
    double q    = input.twist.angular.y;
    double r    = input.twist.angular.z;

    U[0]        = input.motor_status.voltage[0];
    U[1]        = input.motor_status.voltage[1];
    U[2]        = input.motor_status.voltage[2];
    U[3]        = input.motor_status.voltage[3];

    v_1[0] = - w + l_m*q;
    v_1[1] = - w - l_m*p;
    v_1[2] = - w - l_m*q;
    v_1[3] = - w + l_m*p;

    // calculate thrust for all 4 rotors
    for (int i=0; i<4; i++)
    {
        if (v_1[i] < 0)
            // if the flow speed at infinity is negative
            F_m[i] = CT2s*pow(v_1[i],2.0) + CT1s*v_1[i]*x[i] + CT0s*pow(x[i],2.0);

        else
            // if the flow speed at infinity is positive
            F_m[i] = -CT2s*pow(v_1[i],2.0) + CT1s*v_1[i]*x[i] + CT0s*pow(x[i],2.0);

        // sum up all rotor forces
        F[2] = F[2] + F_m[i];

        motorspeed(x_pred[i], x[i], dt, U[i], k_t*F_m[i], M_e[i], parameters);
    }

    // System output, i.e. force and torque of quadrotor
    output.wrench.force.x = F[0];
    output.wrench.force.y = F[1];
    output.wrench.force.z = F[2];

    // torque for rotating quadrocopter around x-axis is the mechanical torque
    output.wrench.torque.x = (F_m[3]-F_m[1])*l_m;
    // torque for rotating quadrocopter around y-axis is the mechanical torque
    output.wrench.torque.y = (F_m[0]-F_m[2])*l_m;
    // torque for rotating quadrocopter around z-axis is the electrical torque
    output.wrench.torque.z = (-M_e[0]-M_e[2]+M_e[1]+M_e[3]);

    // motor speeds (rad/s)
    output.motor_status.frequency[0] = x_pred[0];
    output.motor_status.frequency[1] = x_pred[1];
    output.motor_status.frequency[2] = x_pred[2];
    output.motor_status.frequency[3] = x_pred[3];

    // motor current (A)
    output.motor_status.current[0]   = M_e[0] / Psi;
    output.motor_status.current[1]   = M_e[1] / Psi;
    output.motor_status.current[2]   = M_e[2] / Psi;
    output.motor_status.current[3]   = M_e[3] / Psi;
}

void Propulsion::motorspeed(double &x_pred, const double &x, const double &dt, const double &input_voltage, const double &input_torque, double &output, const Parameters &parameters) {
    // parameters for Roxxy2827-34 motor with 10x4.5 propeller
    double Psi     = parameters.Psi;
    double J_M     = parameters.J_M;
    double R_A     = parameters.R_A;

    // temporarily used Expressions
    double U       = input_voltage;
    double M_m     = input_torque;
    double omega_m = x;

    // Use euler solver to predict next time step
    // predicted motor speed
    x_pred         = omega_m + dt*(Psi/R_A*(U-Psi*omega_m) - M_m)/J_M;
    // predict torque
    output         = Psi/R_A*(U - Psi*x_pred);
}

void Quadrotor::f(State &x_pred, const State &x, const double dt, const Input &input, Output &output, const Parameters &parameters) {

    // update propulsion model
    Propulsion::Input propulsion_input;
    propulsion_input.motor_status = input.motor_status;
    propulsion_input.twist = input.twist; //getBodyTwist();
    Propulsion::Output propulsion_output;
    Propulsion::f(x_pred.propulsion, x.propulsion, dt, propulsion_input, propulsion_output, parameters.propulsion);

    // update aerodynamics model
    Aerodynamics::Input aerodynamics_input;
    aerodynamics_input.wind = input.wind;
    aerodynamics_input.twist = propulsion_input.twist; //getBodyTwist();
    Aerodynamics::Output aerodynamics_output;
    Aerodynamics::f(x_pred.aerodynamics, x.aerodynamics, dt, aerodynamics_input, aerodynamics_output, parameters.aerodynamics);

    // calculate wrench
    output.wrench.force.x  = propulsion_output.wrench.force.x  + aerodynamics_output.wrench.force.x;
    output.wrench.force.y  = propulsion_output.wrench.force.y  + aerodynamics_output.wrench.force.y;
    output.wrench.force.z  = propulsion_output.wrench.force.z  + aerodynamics_output.wrench.force.z;
    output.wrench.torque.x = propulsion_output.wrench.torque.x + aerodynamics_output.wrench.torque.x;
    output.wrench.torque.y = propulsion_output.wrench.torque.y + aerodynamics_output.wrench.torque.y;
    output.wrench.torque.z = propulsion_output.wrench.torque.z + aerodynamics_output.wrench.torque.z;
}

}

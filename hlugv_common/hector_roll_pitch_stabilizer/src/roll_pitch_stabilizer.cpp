//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
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
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <string>
#include "hector_roll_pitch_stabilizer/DoScan.h"

std::string p_base_frame_;
std::string p_base_stabilized_frame_;


tf::TransformListener* tfL_;
tf::StampedTransform transform_;

ros::Publisher pub_desired_roll_angle_;
ros::Publisher pub_desired_pitch_angle_;
ros::ServiceServer scan_server_;

bool updatesEnabled = true;

void stabilize() {
  try
  {
      tfL_->lookupTransform(p_base_frame_, p_base_stabilized_frame_, ros::Time(0), transform_);

      tfScalar yaw, pitch, roll;
      transform_.getBasis().getEulerYPR(yaw, pitch, roll);

      std_msgs::Float64 tmp;

      tmp.data = -roll;
      pub_desired_roll_angle_.publish(tmp);

      tmp.data = -pitch;
      pub_desired_pitch_angle_.publish(tmp);    
  }
  catch(tf::TransformException e)
  {
    //ROS_ERROR("Transform failed %s",e.what());
    //odom_to_map.setIdentity();
  }
}

void updateTimerCallback(const ros::TimerEvent& event)
{
  if ( updatesEnabled ) {
    stabilize();
  }
}

bool doScan(hector_roll_pitch_stabilizer::DoScan::Request &request, hector_roll_pitch_stabilizer::DoScan::Response &response) {
  updatesEnabled = false;
  
  std_msgs::Float64 tmp;
  
  //request.max_angle_pitch, request.step, request.sleep_time_ms);

  stabilize();
    
  // traverse pitch angle
  for ( double pos = 0; pos > request.min_angle_pitch; pos -= request.step ) {
    tmp.data = pos;
    pub_desired_pitch_angle_.publish(tmp);
    usleep(1000*request.sleep_time_ms);
  }
  
  for ( double pos = request.min_angle_pitch; pos <= request.max_angle_pitch; pos += request.step ) {
    tmp.data = pos;
    pub_desired_pitch_angle_.publish(tmp);
    usleep(1000*request.sleep_time_ms);
  }
 
  for ( double pos = request.max_angle_pitch; pos >= 0.0; pos -= request.step ) {
    tmp.data = pos;
    pub_desired_pitch_angle_.publish(tmp);
    usleep(1000*request.sleep_time_ms);
  }
  
  // traverse roll angle
  for ( double pos = 0; pos > request.min_angle_roll; pos -= request.step ) {
    tmp.data = pos;
    pub_desired_pitch_angle_.publish(tmp);
    usleep(1000*request.sleep_time_ms);
  }
  
  for ( double pos = request.min_angle_roll; pos <= request.max_angle_roll; pos += request.step ) {
    tmp.data = pos;
    pub_desired_roll_angle_.publish(tmp);
    usleep(1000*request.sleep_time_ms);
  }
    
  for ( double pos = request.max_angle_roll; pos >= 0.0; pos -= request.step ) {
    tmp.data = pos;
    pub_desired_pitch_angle_.publish(tmp);
    usleep(1000*request.sleep_time_ms);
  }
  
  updatesEnabled = true;  
  
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("base_frame", p_base_frame_, std::string("base_frame"));
  pn.param("base_stabilized_frame", p_base_stabilized_frame_, std::string("base_stabilized_frame"));

  tfL_ = new tf::TransformListener();

  ros::Timer update_timer = pn.createTimer(ros::Duration(1.0 / 30), &updateTimerCallback, false);

  pub_desired_roll_angle_ = pn.advertise<std_msgs::Float64>("/desired_roll_angle",10,false);
  pub_desired_pitch_angle_ = pn.advertise<std_msgs::Float64>("/desired_pitch_angle",10,false);
  
  scan_server_ = n.advertiseService(std::string("/hector_roll_pitch_stabilizer/do_scan"), &doScan);

  ros::spin();

  delete tfL_;

  return 0;
}


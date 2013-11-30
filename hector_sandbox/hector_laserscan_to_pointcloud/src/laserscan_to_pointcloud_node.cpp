//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
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
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class LaserscanToPointcloud
{
public:

  LaserscanToPointcloud()
  {
    ros::NodeHandle nh_;

    scan_sub_ = nh_.subscribe("scan", 1, &LaserscanToPointcloud::scanCallback, this);
    point_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud",1,false);


    ros::NodeHandle pnh_("~");
    pnh_.param("max_range", p_max_range_, 29.0);
    pnh_.param("min_range", p_min_range_, 0.0);


    pnh_.param("use_high_fidelity_projection", p_use_high_fidelity_projection_, false);

    if (p_use_high_fidelity_projection_){
      pnh_.param("target_frame", p_target_frame_, std::string("NO_TARGET_FRAME_SPECIFIED"));

      if (p_target_frame_ == "NO_TARGET_FRAME_SPECIFIED"){
        ROS_ERROR("No target frame specified! Needs to be set for high fidelity projection to work");
        p_use_high_fidelity_projection_ = false;
        return;
      }

      tfl_.reset(new tf::TransformListener());

    }

  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    cloud2_.data.clear();

    const sensor_msgs::LaserScan* scan_to_convert = scan_in.get();

    if (p_min_range_ > 0.0){
      scan_min_range_ = *scan_in;

      size_t num_scans = scan_min_range_.ranges.size();

      std::vector<float>& ranges_vec = scan_min_range_.ranges;

      float min_range = static_cast<float>(p_min_range_);

      for (size_t i = 0; i < num_scans; ++i){
        if (ranges_vec[i] < min_range){
          ranges_vec[i] = -INFINITY;
        }
      }

      scan_to_convert = &scan_min_range_;
    }

    if (p_use_high_fidelity_projection_){
      projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_to_convert, cloud2_, *tfl_, p_max_range_, laser_geometry::channel_option::Intensity);
    }else{
      projector_.projectLaser(*scan_to_convert, cloud2_, p_max_range_, laser_geometry::channel_option::Intensity);
    }


    point_cloud2_pub_.publish(cloud2_);
  }

protected:
  ros::Subscriber scan_sub_;
  ros::Publisher point_cloud2_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;

  double p_max_range_;
  double p_min_range_;
  bool p_use_high_fidelity_projection_;
  std::string p_target_frame_;

  laser_geometry::LaserProjection projector_;

  sensor_msgs::PointCloud2 cloud2_;
  sensor_msgs::LaserScan scan_min_range_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_laserscan_to_pointcloud_node");

  LaserscanToPointcloud ls;

  ros::spin();
}

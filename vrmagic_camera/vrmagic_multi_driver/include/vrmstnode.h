/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, TU Darmstadt.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of TU Darmstadt nor the names of the
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <dynamic_reconfigure/server.h>
#include <vrmagic_multi_driver/CamParamsConfig.h>

#include <boost/thread.hpp>

#include <vrmagic_devkit_wrapper/vrmusbcam2.h>

class PropertyCache;

class VRMagicStereoNode
{
private:
        bool calibrated;
	unsigned int framesDelivered;
        unsigned int width, height;
	ros::NodeHandle leftNs, rightNs, thisNode;
	image_transport::CameraPublisher camPubLeft, camPubRight;
	sensor_msgs::CameraInfo leftCalib, rightCalib;
        ros::ServiceServer leftCalibUpdate, rightCalibUpdate;
	// it would be better to use CameraInfoManager, if saving to cameraeprom is not required
  dynamic_reconfigure::Server<vrmagic_multi_driver::CamParamsConfig> dConfServer;
        ros::Rate fpsLimit;
        VRmUsbCamDevice device;
	PropertyCache *props;
	boost::mutex camAccess, calibAccess, timerAccess;
        sensor_msgs::Image imgLeft;
        sensor_msgs::Image imgRight;
	const std::string frame_id;
	
  void propertyUpdate(vrmagic_multi_driver::CamParamsConfig &config, uint32_t level);

        bool runUpdateLeft(sensor_msgs::SetCameraInfo::Request &req,
            sensor_msgs::SetCameraInfo::Response &res);

        bool runUpdateRight(sensor_msgs::SetCameraInfo::Request &req,
            sensor_msgs::SetCameraInfo::Response &res);

        void storeCalibration();
        void loadCalibration();
        void AnnounceTopics();
        void AbandonTopics();
        void broadcastFrame();
	void grabFrame(VRmDWORD port, sensor_msgs::Image &img, const ros::Time &triggerTime);
        void initCam(VRmDWORD camDesired);
	void initProperties();

public:
        VRMagicStereoNode(VRmDWORD camDesired);
        ~VRMagicStereoNode();
        void spin();
	void retireCam();

};

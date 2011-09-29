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
#include "vrmstnode.h"

#include "sourceformatlist.h"
#include "formatindicator.h"

#include <iostream>
#include <sstream>

#include <signal.h>

VRMagicStereoNode *vrsnode = NULL;

class VRGrabException : public std::string
{
public:
    VRGrabException(const char *err) : std::string(err) {}
};

class VRControlException : public std::string
{
public:
    VRControlException(const char *err) : std::string(err) {}
};

class PropertyCache
{
public:
    float exposureTime;
    int gainLeft, gainRight;
    float fps;
    VRmBOOL useLEDs;
};

void VRMagicStereoNode::propertyUpdate(vrmagic_stereo::CamParamsConfig &config, uint32_t level)
{
    if(props->exposureTime != config.exposureTime)
    {
	boost::lock_guard<boost::mutex> lock(camAccess);

	if(!VRmUsbCamStop(device))
	    throw VRControlException("VRmUsbCamStop failed.");

	float newVal = config.exposureTime;
	if(!VRmUsbCamSetPropertyValueF(device, VRM_PROPID_MULTICAM_MASTER_EXPOSURE_TIME_F, &newVal))
	    std::cerr << "VRmUsbCamSetPropertyValueF(MULTICAM_MASTER_EXPOSURE_TIME_F) failed." << std::endl;
	else
	    props->exposureTime = newVal;

	if(!VRmUsbCamStart(device))
	    throw VRControlException("VRmUsbCamStart failed.");

    }

    if(props->gainLeft != config.gainLeft)
    {
	boost::lock_guard<boost::mutex> lock(camAccess);

	VRmPropId sensor = VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_1;
	VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensor);

	int newVal = config.gainLeft;
	if(!VRmUsbCamSetPropertyValueI(device, VRM_PROPID_CAM_GAIN_MONOCHROME_I, &newVal))
	    std::cerr << "VRmUsbCamSetPropertyValueI(VRM_PROPID_CAM_GAIN_MONOCHROME_I) failed." << std::endl;
	else
	    props->gainLeft = newVal;
    }

    if(props->gainRight != config.gainRight)
    {
	boost::lock_guard<boost::mutex> lock(camAccess);

	VRmPropId sensor = VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_3;
	VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensor);

	int newVal = config.gainRight;
	if(!VRmUsbCamSetPropertyValueI(device, VRM_PROPID_CAM_GAIN_MONOCHROME_I, &newVal))
	    std::cerr << "VRmUsbCamSetPropertyValueI(VRM_PROPID_CAM_GAIN_MONOCHROME_I) failed." << std::endl;
	else
	    props->gainRight = newVal;
    }

    if(props->useLEDs != config.useLEDs)
    {
	boost::lock_guard<boost::mutex> lock(camAccess);
	VRmBOOL newValue = config.useLEDs;
	if(!VRmUsbCamSetPropertyValueB(device, VRM_PROPID_DEVICE_STATUS_LED_B, &newValue))
	    std::cerr << "VRmUsbCamSetPropertyValueB(DEVICE_STATUS_LED_B) failed." << std::endl;
	else
	    props->useLEDs = newValue;
    }

    if(props->fps != config.fps)
    {
	boost::lock_guard<boost::mutex> lock(timerAccess);
	fpsLimit = ros::Rate(config.fps);
	props->fps = config.fps;
    }

}

bool VRMagicStereoNode::runUpdateLeft(sensor_msgs::SetCameraInfo::Request &req,
    sensor_msgs::SetCameraInfo::Response &res)
{
    boost::lock_guard<boost::mutex> lock(calibAccess);
    leftCalib = req.camera_info;
    storeCalibration();
    res.success = true;
    res.status_message = "Calibration updated";
    return true;
}

bool VRMagicStereoNode::runUpdateRight(sensor_msgs::SetCameraInfo::Request &req,
    sensor_msgs::SetCameraInfo::Response &res)
{
    boost::lock_guard<boost::mutex> lock(calibAccess);
    rightCalib = req.camera_info;
    storeCalibration();
    res.success = true;
    res.status_message = "Calibration updated";
    return true;
}

void VRMagicStereoNode::loadCalibration()
{
        VRmUserData *uData;
        if(!VRmUsbCamLoadUserData(device, &uData))
        {
            std::cerr << "VRmUsbCamLoadUserData failed."  << std::endl;
            return;
        }

        if(uData->m_length == 0)
        {
            std::cerr << "there is no calibration stored on the camera."  << std::endl;
            return;
        }

        ros::serialization::IStream stream(uData->mp_data, uData->m_length);
        ros::serialization::deserialize(stream, leftCalib);
        ros::serialization::deserialize(stream, rightCalib);

        VRmUsbCamFreeUserData(&uData);

        calibrated = true;
}

void VRMagicStereoNode::storeCalibration()
{
    uint32_t leftLen = ros::serialization::serializationLength(leftCalib);
    uint32_t rightLen = ros::serialization::serializationLength(rightCalib);

    VRmUserData *uData;
    if(!VRmUsbCamNewUserData(&uData, leftLen + rightLen))
    {
        std::cerr << "VRmUsbCamNewUserData failed."  << std::endl;
        return;
    }
    ros::serialization::OStream stream(uData->mp_data, leftLen + rightLen);
    ros::serialization::serialize(stream, leftCalib);
    ros::serialization::serialize(stream, rightCalib);

    camAccess.lock();
    if(!VRmUsbCamSaveUserData(device, uData))
    {
        std::cerr << "VRmUsbCamSaveUserData failed."  << std::endl;
    }
    camAccess.unlock();

    std::cout << "calibration written to camera." << std::endl;

    VRmUsbCamFreeUserData(&uData);
}


void VRMagicStereoNode::AnnounceTopics()
{
	camPubLeft = image_transport::ImageTransport(leftNs).advertiseCamera("image_raw", 2);
	camPubRight = image_transport::ImageTransport(rightNs).advertiseCamera("image_raw", 2);
        leftCalibUpdate = leftNs.advertiseService("set_camera_info", &VRMagicStereoNode::runUpdateLeft, this);
        rightCalibUpdate = rightNs.advertiseService("set_camera_info", &VRMagicStereoNode::runUpdateRight, this);
}

void VRMagicStereoNode::AbandonTopics()
{
        camPubLeft.shutdown();
        camPubRight.shutdown();
	leftCalibUpdate.shutdown();
	rightCalibUpdate.shutdown();
}

void VRMagicStereoNode::broadcastFrame()
{
    camAccess.lock();
    VRmRetVal success = VRmUsbCamSoftTrigger(device);
    camAccess.unlock();
    if(!success)
        throw VRGrabException("VRmUsbCamSoftTrigger failed.");

    ros::Time triggerTime = ros::Time::now();

    grabFrame(1, imgLeft, triggerTime);
    grabFrame(3, imgRight, triggerTime);
    leftCalib.header.stamp = triggerTime;
    leftCalib.header.frame_id = frame_id;
    rightCalib.header.stamp = triggerTime;
    rightCalib.header.frame_id = frame_id;

    try
    {
	boost::lock_guard<boost::mutex> lock(calibAccess);
	camPubLeft.publish(imgLeft, leftCalib);
     }
     catch(ros::serialization::StreamOverrunException &crap)
     {
	 std::cerr << "stream overrun in left channel" << std::endl;
     }

     try
     {
	 boost::lock_guard<boost::mutex> lock(calibAccess);
	 camPubRight.publish(imgRight, rightCalib);
      }
      catch(ros::serialization::StreamOverrunException &crap)
      {
	  std::cerr << "stream overrun in right channel" << std::endl;
      }
}

void VRMagicStereoNode::grabFrame(VRmDWORD port, sensor_msgs::Image &img, const ros::Time &triggerTime)
{
    img.width = width;
    img.height = height;
    img.step = width * 2;
    img.encoding = sensor_msgs::image_encodings::MONO16;
    img.data.resize(height * img.step);
    img.header.stamp = triggerTime;
    img.header.frame_id = frame_id;

    VRmImage *VRimg = NULL;
    camAccess.lock();
    VRmRetVal success = VRmUsbCamLockNextImageEx(device, port, &VRimg, NULL);
    camAccess.unlock();
    if(!success)
    {
        std::stringstream err;
        err << "VRmUsbCamLockNextImageEx failed for port" << port <<  ".";
        throw VRGrabException(err.str().c_str());
    }

    for(unsigned int i = 0; i < height * width; i++)
    {
	img.data[i * 2 + 1] = VRimg->mp_buffer[i * 2] >> 6;
	img.data[i * 2] = (VRimg->mp_buffer[i * 2] << 2)
		| (VRimg->mp_buffer[i * 2 + 1] & 0x3);
    }

    if(!VRmUsbCamUnlockNextImage(device, &VRimg))
        throw VRGrabException("VRmUsbCamUnlockNextImage failed.");

    if(!VRmUsbCamFreeImage(&VRimg))
        throw VRGrabException("VRmUsbCamFreeImage failed.");
}

void VRMagicStereoNode::initProperties()
{
    props = new PropertyCache();

    if(!VRmUsbCamGetPropertyValueF(device, VRM_PROPID_MULTICAM_MASTER_EXPOSURE_TIME_F, &props->exposureTime))
	std::cerr << "VRmUsbCamGetPropertyValueF(MULTICAM_MASTER_EXPOSURE_TIME_F) failed." << std::endl;

    if(!VRmUsbCamGetPropertyValueB(device, VRM_PROPID_DEVICE_STATUS_LED_B, &props->useLEDs))
	std::cerr << "VRmUsbCamGetPropertyValueB(DEVICE_STATUS_LED_B) failed." << std::endl;

    VRmPropId sensor = VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_1;
    VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensor);
    if(!VRmUsbCamGetPropertyValueI(device, VRM_PROPID_CAM_GAIN_MONOCHROME_I, &props->gainLeft))
	std::cerr << "VRmUsbCamGetPropertyValueI(VRM_PROPID_CAM_GAIN_MONOCHROME_I) failed." << std::endl;

    sensor = VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_3;
    VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensor);
    if(!VRmUsbCamGetPropertyValueI(device, VRM_PROPID_CAM_GAIN_MONOCHROME_I, &props->gainRight))
	std::cerr << "VRmUsbCamGetPropertyValueI(VRM_PROPID_CAM_GAIN_MONOCHROME_I) failed." << std::endl;

    props->fps = 0.5;
}

void VRMagicStereoNode::initCam(VRmDWORD camDesired)
{
    VRmDWORD libversion;
    if(!VRmUsbCamGetVersion(&libversion))
        throw VRControlException("VRmUsbCamGetVersion failed.");
    else
        std::cout << "VR Magic lib has version " << libversion << std::endl;

    VRmDWORD size;
    if(!VRmUsbCamGetDeviceKeyListSize(&size))
        throw VRControlException("VRmUsbCamGetDeviceKeyListSize failed.");

    if(camDesired >= size)
        throw VRControlException("Invalid device index.");

    VRmDeviceKey* devKey;
    if(!VRmUsbCamGetDeviceKeyListEntry(camDesired, &devKey))
        throw VRControlException("VRmUsbCamGetDeviceKeyListEntry failed.");

    if(devKey->m_busy)
        throw VRControlException("device busy");

    if(!VRmUsbCamOpenDevice(devKey, &device))
        throw VRControlException("VRmUsbCamOpenDevice failed.");

    std::cout << "device " << devKey->mp_product_str << " [" << devKey->mp_manufacturer_str
	    << "] opened" << std::endl;

    if(!VRmUsbCamFreeDeviceKey(&devKey))
        std::cerr << "VRmUsbCamFreeDeviceKey failed." << std::endl;

    SourceFormatList sFmtList(device);
    SourceFormatList::iterator fmt = find_if(sFmtList.begin(), sFmtList.end(), FormatIndicator());
    if(fmt == sFmtList.end())
        throw VRControlException("no acceptable format found.");

    leftCalib.width = rightCalib.width =  width = fmt->m_width;
    leftCalib.height = rightCalib.height = height = fmt->m_height;

    if(!VRmUsbCamSetSourceFormatIndex(device, fmt - sFmtList.begin()))
        throw VRControlException("failed to select desired format.");

    VRmPropId mode = VRM_PROPID_GRAB_MODE_TRIGGERED_SOFT;
    if (!VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_MODE_E, &mode))
        throw VRControlException("failed to set software trigger (VRM_PROPID_GRAB_MODE_TRIGGERED_SOFT).");

    if(!VRmUsbCamStart(device))
        throw VRControlException("VRmUsbCamStart failed.");
}

void VRMagicStereoNode::retireCam()
{
    if(!VRmUsbCamStop(device))
        std::cerr << "VRmUsbCamStop failed." << std::endl;

    if(!VRmUsbCamCloseDevice(device))
            std::cerr << "VRmUsbCamCloseDevice failed." << std::endl;

    VRmUsbCamCleanup();
}

VRMagicStereoNode::VRMagicStereoNode(VRmDWORD camDesired) : calibrated(false), framesDelivered(0),
    leftNs("left"), rightNs("right"), fpsLimit(0.5), frame_id("camer_optical_frame")
{
    leftCalib.K[0] = rightCalib.K[0] = 0.0;
    initCam(camDesired);
    loadCalibration();
    initProperties();
    dConfServer.setCallback(boost::bind(&VRMagicStereoNode::propertyUpdate, this, _1, _2));
    AnnounceTopics();
}

VRMagicStereoNode::~VRMagicStereoNode()
{
    retireCam();
    AbandonTopics();
    delete props;
}

void VRMagicStereoNode::spin()
{
        while(ros::ok())
        {
            try
            {
                broadcastFrame();
            }
            catch(VRGrabException &ex)
            {
                std::cerr << ex << std::endl;
            }
            ros::spinOnce();
            framesDelivered++;

	    boost::lock_guard<boost::mutex> lock(timerAccess);
            fpsLimit.sleep();
        }
}

void forceShutdown(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    std::cerr << "Segfault, shutting down camera !" << std::endl;

    if (vrsnode)
	vrsnode->retireCam();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrmagic_stereo_node", ros::init_options::AnonymousName);

	signal(SIGSEGV, forceShutdown);

        try
        {
	    vrsnode = new VRMagicStereoNode(0);
	    vrsnode->spin();
        }
        catch(VRControlException &cex)
        {
            std::cerr << cex << std::endl;
            VRmUsbCamCleanup();
        }

	delete vrsnode;
	
	return 0;
}

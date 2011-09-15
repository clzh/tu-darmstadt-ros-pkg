#include "object_tracker.h"

#include <hector_nav_msgs/GetDistanceToObstacle.h>
#include <worldmodel_msgs/VerifyObject.h>

#include <Eigen/Geometry>
#include <math.h>

#include "LinearMath/btTransform.h"
#include <tf_conversions/tf_eigen.h>
#include "Object.h"

#include <boost/algorithm/string.hpp>

namespace object_tracker {

ObjectTracker::ObjectTracker()
  : nh("worldmodel")
  , _project_objects(false)
  , _frame_id("map")
  , _default_distance(1.0)
  , _distance_variance(1.0)
  , _angle_variance(5.0 * M_PI / 180.0)
  , _min_height(-999.9)
  , _max_height(999.9)
{
  ros::NodeHandle global_nh;
  sysCommandSubscriber = global_nh.subscribe("syscommand", 10, &ObjectTracker::sysCommandCb, this);

  imagePerceptSubscriber = nh.subscribe("image_percept", 10, &ObjectTracker::imagePerceptCb, this);
  posePerceptSubscriber = nh.subscribe("pose_percept", 10, &ObjectTracker::posePerceptCb, this);
  modelPublisher = nh.advertise<worldmodel_msgs::ObjectModel>("objects", 10, false);
  modelUpdatePublisher = nh.advertise<worldmodel_msgs::Object>("object", 10, false);

  Object::setNamespace(nh.getNamespace());

  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("project_objects", _project_objects);
  priv_nh.getParam("frame_id", _frame_id);
  priv_nh.getParam("default_distance", _default_distance);
  priv_nh.getParam("distance_variance", _distance_variance);
  priv_nh.getParam("angle_variance", _angle_variance);
  priv_nh.getParam("min_height", _min_height);
  priv_nh.getParam("max_height", _max_height);

  std::string verification_services_str;
  priv_nh.getParam("verfication_services", verification_services_str);
  std::vector<std::string> verification_services;
  boost::algorithm::split(verification_services, verification_services_str, boost::is_any_of(","));
  for(std::vector<std::string>::iterator it = verification_services.begin(); it != verification_services.end(); ++it) {
    if (it->empty()) continue;
    verificationServices.push_back(global_nh.serviceClient<worldmodel_msgs::VerifyObject>(*it));
    if (verificationServices.back().exists()) {
      ROS_INFO("Using verification service %s", it->c_str());
    } else {
      ROS_WARN("Verification service %s is not (yet) there...", it->c_str());
    }
  }

  distanceToObstacle = global_nh.serviceClient<hector_nav_msgs::GetDistanceToObstacle>("get_distance_to_obstacle", true);
  if (_project_objects && !distanceToObstacle.waitForExistence(ros::Duration(5.0))) {
    ROS_WARN("_project_objects is true, but GetDistanceToObstacle service is not (yet) available");
  }

  setObjectState = nh.advertiseService("set_object_state", &ObjectTracker::setObjectStateCb, this);
  addObject = nh.advertiseService("add_object", &ObjectTracker::addObjectCb, this);
  getObjectModel = nh.advertiseService("get_object_model", &ObjectTracker::getObjectModelCb, this);

  drawings.setNamespace("worldmodel");
}

ObjectTracker::~ObjectTracker()
{}

void ObjectTracker::sysCommandCb(const std_msgs::StringConstPtr &sysCommand)
{
  if (sysCommand->data == "reset") {
    ROS_INFO("Resetting object model.");
    model.reset();
  }
}

void ObjectTracker::imagePerceptCb(const worldmodel_msgs::ImagePerceptConstPtr &percept)
{
  worldmodel_msgs::PosePerceptPtr posePercept(new worldmodel_msgs::PosePercept);

  posePercept->header = percept->header;
  posePercept->info = percept->info;

  // get camera model
  if (cameraModels.count(percept->header.frame_id) == 0) {
    cameraModels[percept->header.frame_id].fromCameraInfo(percept->camera_info);
  }
  const image_geometry::PinholeCameraModel& cameraModel = cameraModels[percept->header.frame_id];
  cv::Point3d direction_cv = cameraModel.projectPixelTo3dRay(cv::Point2d(percept->x + percept->width/2, percept->y + percept->height/2));
  tf::Point direction_tf(direction_cv.z, -direction_cv.x, -direction_cv.y);
  tf::Quaternion direction_quaternion(direction_tf.y()/direction_tf.x(), -direction_tf.z()/direction_tf.x(), 0.0);

  // fill posePercept from direction vector
  tf::pointTFToMsg(direction_tf.normalized() * _default_distance, posePercept->pose.pose.position);
  tf::quaternionTFToMsg(direction_quaternion, posePercept->pose.pose.orientation);

  // forward to posePercept callback
  posePerceptCb(posePercept);
}

void ObjectTracker::posePerceptCb(const worldmodel_msgs::PosePerceptConstPtr &percept)
{
  // convert pose in tf
  tf::Pose pose;
  tf::poseMsgToTF(percept->pose.pose, pose);
  tf::Quaternion direction(pose.getOrigin().y()/pose.getOrigin().x(), -pose.getOrigin().z()/pose.getOrigin().x(), 0.0);

  // retrieve distance information
  float distance = pose.getOrigin().length();

  if (_project_objects) {
    hector_nav_msgs::GetDistanceToObstacle::Request request;
    hector_nav_msgs::GetDistanceToObstacle::Response response;

    // project image percept to the next obstacle
    request.point.header = percept->header;
    tf::pointTFToMsg(pose.getOrigin(), request.point.point);
    if (distanceToObstacle.call(request, response) && response.distance > 0.0) {
      // distance = std::max(response.distance - 0.1f, 0.0f);
      distance = std::max(response.distance, 0.0f);
      pose.setOrigin(pose.getOrigin().normalized() * distance);
      ROS_DEBUG("Projected percept to a distance of %.1f m", distance);
    } else {
      ROS_DEBUG("Ignoring percept due to unknown or infinite distance");
      return;
    }
  }

  // extract variance matrix
  Eigen::Matrix<float,6,6> temp;
  for(unsigned int i = 0; i < 36; ++i) temp(i) = percept->pose.covariance[i];
  Eigen::Matrix3f covariance = temp.block<3,3>(0,0);

  // if no variance is given, set variance to default
  if (covariance.isZero()) {
    covariance(0,0) = _distance_variance;
    covariance(1,1) = std::max(distance*distance, 1.0f) * _angle_variance;
    covariance(2,2) = covariance(1,1);
  }

  // rotate covariance matrix depending on the position in the image
  Eigen::Matrix3f rotation_camera_object(Eigen::Quaternionf(direction.w(), direction.x(), direction.y(), direction.z()).matrix());
  covariance = rotation_camera_object * covariance * rotation_camera_object.transpose();

  // project percept coordinates to map frame
  tf::StampedTransform cameraTransform;
  if (!_frame_id.empty() && percept->header.frame_id != _frame_id) {
    // retrieve camera transformation from tf
    try {
      tf.waitForTransform(_frame_id, percept->header.frame_id, percept->header.stamp, ros::Duration(1.0));
      tf.lookupTransform(_frame_id, percept->header.frame_id, percept->header.stamp, cameraTransform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    pose = cameraTransform * pose;

    // rotate covariance matrix to map coordinates
    Eigen::Quaterniond eigen_rotation;
    tf::RotationTFToEigen(cameraTransform.getRotation(), eigen_rotation);
    Eigen::Matrix3f rotation_map_camera(eigen_rotation.toRotationMatrix().cast<float>());
    covariance = rotation_map_camera * covariance * rotation_map_camera.transpose();
  }
  Eigen::Vector3f position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());

  // check height
  float relative_height = pose.getOrigin().z() - cameraTransform.getOrigin().z();
  if (relative_height < _min_height || relative_height > _max_height) {
    ROS_INFO("Discarding %s percept with height %f", percept->info.class_id.c_str(), relative_height);
    return;
  }

  // fix height (assume camera is always at 0.475m)
  // pose.setOrigin(tf::Point(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() - cameraTransform.getOrigin().z() + 0.475f));

  // calculate observation support
  float support = 0.0;
  if (!percept->info.object_id.empty()) {
    support = percept->info.object_support;
  } else if (!percept->info.class_id.empty()) {
    support = percept->info.class_support;
  }

  if (support == 0.0) {
    ROS_WARN("Ignoring percept with support == 0.0");
    return;
  }

  // lock model
  model.lock();

  // find correspondence
  ObjectPtr object;
  float min_distance = 1.0f;
  if (percept->info.object_id.empty()) {
    for(ObjectModel::iterator it = model.begin(); it != model.end(); ++it) {
      ObjectPtr x = *it;
      if (!percept->info.class_id.empty() && percept->info.class_id != x->getClassId()) continue;
      Eigen::Vector3f diff = x->getPosition() - position;
      float distance = (diff.transpose() * (x->getCovariance() + covariance).inverse() * diff)[0];
      if (distance < min_distance) {
        object = x;
        min_distance = distance;
      }
    }
  }

  if (object && object->getState() < 0) {
    ROS_DEBUG("Percept was associated to object %s, which has a fixed state", object->getObjectId().c_str());
    model.unlock();
    return;
  }

  // create new object
  if (!object) {
    object = model.add(percept->info.class_id, percept->info.object_id);

    object->setPosition(position);
    object->setCovariance(covariance);
    object->setSupport(support);

    ROS_INFO("Found new object %s of class %s at (%f,%f)!", object->getObjectId().c_str(), object->getClassId().c_str(), position.x(), position.y());

  // or update existing object
  } else if (support > 0.0) {
    object->update(position, covariance, support);

  // or simply decrease support
  } else {
    object->addSupport(support);
  }

  // set object orientation
  geometry_msgs::Quaternion object_orientation;
  tf::quaternionTFToMsg(pose.getRotation(), object_orientation);
  object->setOrientation(object_orientation);

  // update object header
  std_msgs::Header header = percept->header;
  header.frame_id = _frame_id;
  object->setHeader(header);

  // unlock model
  model.unlock();

  // call victim verification
  if (verificationServices.size() > 0) {
    worldmodel_msgs::VerifyObject::Request request;
    worldmodel_msgs::VerifyObject::Response response;

    request.object = object->getObjectMessage();

    for(std::vector<ros::ServiceClient>::iterator it = verificationServices.begin(); it != verificationServices.end(); ++it) {
      if (it->call(request, response)) {
        if (response.response == response.DISCARD) {
          ROS_INFO("Discarded object %s due to DISCARD message from service %s", object->getObjectId().c_str(), it->getService().c_str());
          object->setState(worldmodel_msgs::ObjectState::DISCARDED);
        }
        if (response.response == response.CONFIRM) {
          ROS_INFO("We got a CONFIRMation for object %s from service %s!", object->getObjectId().c_str(), it->getService().c_str());
          object->addSupport(100.0);
        }
        if (response.response == response.UNKNOWN) {
          ROS_INFO("Verification service %s cannot help us with object %s at the moment :-(", it->getService().c_str(), object->getObjectId().c_str());
        }
      }
    }
  }

  modelUpdatePublisher.publish(object->getObjectMessage());

  publishModel();
}

bool ObjectTracker::setObjectStateCb(worldmodel_msgs::SetObjectState::Request& request, worldmodel_msgs::SetObjectState::Response& response) {
  model.lock();

  ObjectPtr object = model.getObject(request.object_id);
  if (!object) {
    model.unlock();
    return false;
  }

  object->setState(request.new_state.state);
  modelUpdatePublisher.publish(object->getObjectMessage());

  model.unlock();
  publishModel();
  return true;
}

bool ObjectTracker::addObjectCb(worldmodel_msgs::AddObject::Request& request, worldmodel_msgs::AddObject::Response& response) {
  ObjectPtr object;
  bool newObject = false;

  // check if object already exist
  if (!request.object.info.object_id.empty()) {
    object = model.getObject(request.object.info.object_id);
  }

  // create a new object if object does not exist
  if (!object) {
    object.reset(new Object(request.object.info.class_id, request.object.info.object_id));
    newObject = true;
  }

  std_msgs::Header header = request.object.header;
  if (header.stamp.isZero()) header.stamp = ros::Time::now();

  geometry_msgs::PoseWithCovariance pose;
  if (request.map_to_next_obstacle) {
    pose.covariance = request.object.pose.covariance;
    if (!mapToNextObstacle(request.object.pose.pose, header, pose.pose)) {
      return false;
    }
  } else {
    pose = request.object.pose;
  }

  // extract variance matrix
  Eigen::Matrix<float,6,6> temp;
  for(unsigned int i = 0; i < 36; ++i) temp(i) = pose.covariance[i];
  Eigen::Matrix3f covariance = temp.block<3,3>(0,0);

  // if no variance is given, set variance to default
  if (covariance.isZero()) {
    pose.covariance[0] = 1.0;
    pose.covariance[7] = 1.0;
    pose.covariance[14] = 1.0;
  }

  if (!transformPose(pose, pose, header)) return false;

  model.lock();

  object->setHeader(header);
  object->setPose(pose);
  object->setState(request.object.state.state);
  object->setSupport(request.object.info.support);

  if (newObject) model.add(object);
  response.object = object->getObjectMessage();
  modelUpdatePublisher.publish(response.object);

  model.unlock();

  publishModel();
  return true;
}

bool ObjectTracker::getObjectModelCb(worldmodel_msgs::GetObjectModel::Request& request, worldmodel_msgs::GetObjectModel::Response& response) {
  response.model = *(model.getObjectModelMessage());
  return true;
}

bool ObjectTracker::mapToNextObstacle(const geometry_msgs::Pose& source, const std_msgs::Header &header, geometry_msgs::Pose &mapped) {
  if (!distanceToObstacle.exists()) return false;

  // retrieve distance information
  float distance = _default_distance;
  hector_nav_msgs::GetDistanceToObstacle::Request request;
  hector_nav_msgs::GetDistanceToObstacle::Response response;

  // project image percept to the next obstacle
  request.point.header = header;
  request.point.point = source.position;
  // tf::pointTFToMsg(cameraTransform.getOrigin(), request.pose.pose.position);
  // tf::Quaternion direction_quaternion = tf::Quaternion(atan(direction.y/direction.x), atan(direction.z/direction.x), 0.0);
  // direction_quaternion *= cameraTransform.getRotation();
  // tf::quaternionTFToMsg(direction_quaternion, request.pose.pose.orientation);
  if (distanceToObstacle.call(request, response) && response.distance > 0.0) {
    // distance = std::max(response.distance - 0.1f, 0.0f);
    distance = std::max(response.distance, 0.0f);
  } else {
    ROS_DEBUG("Could not map object to next obstacle due to unknown or infinite distance");
    return false;
  }

  tf::Pose sourceTF;
  tf::poseMsgToTF(source, sourceTF);
  sourceTF.setOrigin(sourceTF.getOrigin().normalized() * distance);
  tf::poseTFToMsg(sourceTF, mapped);

  return true;
}

bool ObjectTracker::transformPose(const geometry_msgs::Pose& from, geometry_msgs::Pose &to, std_msgs::Header &header, tf::StampedTransform *transform_ptr) {
  // retrieve transformation from tf
  tf::StampedTransform transform;
  try {
    tf.waitForTransform(_frame_id, header.frame_id, header.stamp, ros::Duration(1.0));
    tf.lookupTransform(_frame_id, header.frame_id, header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::Pose tfPose;
  tf::poseMsgToTF(from, tfPose);
  tfPose = transform * tfPose;
  tf::poseTFToMsg(tfPose, to);

  header.frame_id = _frame_id;
  if (transform_ptr) *transform_ptr = transform;

  return true;
}

bool ObjectTracker::transformPose(const geometry_msgs::PoseWithCovariance& from, geometry_msgs::PoseWithCovariance &to, std_msgs::Header &header) {
  tf::StampedTransform transform;

  if (!transformPose(from.pose, to.pose, header, &transform)) return false;

  // TODO
  // rotate covariance matrix

  return true;
}

void ObjectTracker::publishModel() {
  // Publish all model data on topic /objects
  modelPublisher.publish(model.getObjectModelMessage());

  // Visualize victims and covariance in rviz
  drawings.setTime(ros::Time::now());
  drawings.setColor(1.0, 0.0, 0.0, 1.0);

  model.lock();
  for(ObjectModel::iterator it = model.begin(); it != model.end(); ++it) {
    ObjectPtr object = *it;
    drawings.addMarker(object->getVisualization());
    drawings.drawCovariance(Eigen::Vector2f(object->getPosition().x(), object->getPosition().y()), object->getCovariance().block<2,2>(0,0));
  }
  model.unlock();
  drawings.sendAndResetData();
}

} // namespace object_tracker


int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  object_tracker::ObjectTracker tracker;
  ros::spin();

  exit(0);
}
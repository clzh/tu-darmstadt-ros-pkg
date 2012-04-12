#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

std::string g_odometry_topic;
std::string g_pose_topic;
std::string g_frame_id;
std::string g_footprint_frame_id;
std::string g_stabilized_frame_id;
std::string g_child_frame_id;

void addTransform(std::vector<geometry_msgs::TransformStamped>& transforms, const tf::StampedTransform& tf)
{
  transforms.resize(transforms.size()+1);
  tf::transformStampedTFToMsg(tf, transforms.back());
}

void sendTransform(geometry_msgs::Pose const &pose, const std_msgs::Header& header, std::string child_frame_id = "")
{
  static tf::TransformBroadcaster br;
  static std::vector<geometry_msgs::TransformStamped> transforms(3);

  tf::StampedTransform tf;
  tf.frame_id_ = header.frame_id;
  if (!g_frame_id.empty()) tf.frame_id_ = g_frame_id;
  tf.stamp_ = header.stamp;
  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  transforms.clear();

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf::Point position;
  tf::pointMsgToTF(pose.position, position);

  // footprint intermediate transform (x,y,yaw)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_footprint_frame_id) {
    tf.child_frame_id_ = g_footprint_frame_id;
    tf.setOrigin(tf::Vector3(position.x(), position.y(), 0.0));
    tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
    addTransform(transforms, tf);

    yaw = 0.0;
    position.setX(0.0);
    position.setY(0.0);
    tf.frame_id_ = g_footprint_frame_id;
  }

  // stabilized intermediate transform (z)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_stabilized_frame_id) {
    tf.child_frame_id_ = g_stabilized_frame_id;
    tf.setOrigin(tf::Vector3(0.0, 0.0, position.z()));
    tf.setBasis(tf::Matrix3x3::getIdentity());
    addTransform(transforms, tf);

    position.setZ(0.0);
    tf.frame_id_ = g_stabilized_frame_id;
  }

  // stabilized intermediate transform (z)
  tf.child_frame_id_ = child_frame_id;
  tf.setOrigin(position);
  tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
  addTransform(transforms, tf);

  br.sendTransform(transforms);
}

void odomCallback(nav_msgs::Odometry const &odometry) {
  sendTransform(odometry.pose.pose, odometry.header, odometry.child_frame_id);
}

void poseCallback(geometry_msgs::PoseStamped const &pose) {
  sendTransform(pose.pose, pose.header);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_to_tf");

  g_footprint_frame_id = "base_footprint";
  g_stabilized_frame_id = "base_stabilized";

  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("odometry_topic", g_odometry_topic);
  priv_nh.getParam("pose_topic", g_pose_topic);
  priv_nh.getParam("frame_id", g_frame_id);
  priv_nh.getParam("footprint_frame_id", g_footprint_frame_id);
  priv_nh.getParam("stabilized_frame_id", g_stabilized_frame_id);
  priv_nh.getParam("child_frame_id", g_child_frame_id);

  ros::NodeHandle node;
  ros::Subscriber sub1, sub2;
  if (!g_odometry_topic.empty()) sub1 = node.subscribe(g_odometry_topic, 10, &odomCallback);
  if (!g_pose_topic.empty())     sub2 = node.subscribe(g_pose_topic, 10, &poseCallback);

  if (!sub1 && !sub2) {
    ROS_FATAL("Params odometry_topic and pose_topic are empty... nothing to do for me!");
    return 1;
  }

  ros::spin();
  return 0;
}
#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

void ApplyTransform2Pose(geometry_msgs::Pose &pose_B, const tf2::Transform &T_W_B){
  tf2::Transform T_B_Object;
  tf2::fromMsg(pose_B,T_B_Object);
  geometry_msgs::Pose pose_W;
  tf2::toMsg(T_W_B*T_B_Object,pose_B);
}

double deg2rad(double deg){
  return deg/180*M_PI;
}

geometry_msgs::TransformStamped paramFrameToMsg(ros::NodeHandle &n, std::string childframe){
  geometry_msgs::TransformStamped T;
  T.header.stamp = ros::Time::now();
  T.child_frame_id = childframe;
  std::vector<double> ori, pos;
  std::string param = "/planning_frame/" + childframe + "/";
  n.param<std::string>(param + "parent", T.header.frame_id, "world");
  n.param<std::vector<double>>(param + "position", pos, {0,0,0});
  T.transform.translation.x = pos[0];
  T.transform.translation.y = pos[1];
  T.transform.translation.z = pos[2];
  n.param<std::vector<double>>(param + "orientation", ori, {0,0,0});
  tf2::Quaternion quat;
  quat.setRPY(deg2rad(ori[0]),deg2rad(ori[1]),deg2rad(ori[2]));
  T.transform.rotation = tf2::toMsg(quat);
  ROS_INFO_STREAM("Loaded transform: " << T.header.frame_id << " -> " << T.child_frame_id);
  return T;
}

#endif

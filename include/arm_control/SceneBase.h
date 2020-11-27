#ifndef __SCENEBASE_H__
#define __SCENEBASE_H__

/* ROS */
#include <ros/ros.h>

/* MoveIt */
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

/* ROS transform messages */
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class SceneBase {
public:
  ros::NodeHandle *RosNode = 0;
  moveit::planning_interface::MoveGroupInterface *ur5_move_group_interface = 0;
  moveit::planning_interface::PlanningSceneInterface
      *ur5_planning_scene_interface = 0;
  planning_scene::PlanningScene *ur5_planning_scene = 0;

  //指针指向的内容不可改变
  const robot_state::JointModelGroup *ur5_joint_model_group = 0;

  tf2_ros::StaticTransformBroadcaster *StaticTFBroadcaster = 0;
  tf2_ros::TransformBroadcaster *TFBroadcaster = 0;
  tf2_ros::TransformListener *TFListener = 0;
  tf2_ros::Buffer *TFBuffer = 0;

  /* Constants */
  /* 0 > may reduce warnings about ApplyPlanningScene */
  const double CALL_DILAY = 0.0;
  const std::vector<std::string> JOINT_NAMES = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

  std::string PrintVector(const std::vector<double> &v, int precis = 3);
  std::string PrintVector(const geometry_msgs::Pose &v, int precis = 3);

  bool ApplyAttachedCollisionObject(moveit_msgs::AttachedCollisionObject &msg,
                                    double delay = -1);

  bool ApplyCollisionObject(moveit_msgs::CollisionObject &msg,
                            double delay = -1);

  SceneBase(ros::NodeHandle &node, const std::string PLANNING_GROUP = "");
};

#endif

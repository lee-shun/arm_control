/*******************************************************************************
 *
 * @file
 *
 * @author lee-shun
 *
 * @email 2015097272@qq.com
 *
 * @data
 *
 * @version 1.0
 *
 * @brief
 *
 * @copyright
 *
 *******************************************************************************/
#ifndef _APPLE_TRACK_HPP_
#define _APPLE_TRACK_HPP_

#include "SceneBase.h"
#include <arm_control/PlanPath.h>
#include <arm_control/ServiceInt.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

class AppleTrack;

/**
 * apple class
 */
class Apple {
public:
  enum class AppleStatus { Unpicked, Picked, Missing };
  AppleStatus status = AppleStatus::Unpicked;

  moveit_msgs::CollisionObject cobj_apple;
  geometry_msgs::TransformStamped T_W_A_msg;

  int id = -1;
  double apple_rad = 0.04;

  AppleTrack *SceneBasePtr;
  Apple(int id_, geometry_msgs::Pose pose_, AppleTrack *scenebase);
  ~Apple();
};

/**
 * AppleTrack class
 */
class AppleTrack : public SceneBase {
private:
  int AppleGraspedID = -1;
  ros::Subscriber sub_apple_new;
  ros::ServiceServer svc_apple_coll;
  ros::ServiceServer svc_apple_grasp;
  std::map<int, Apple *> AppleList;
  moveit_msgs::AttachedCollisionObject cobj_apple_grasped;

  /**
   * Arrange the AppleList
   */
  void SubCallback_apple_new(const geometry_msgs::PoseArray::ConstPtr &msg);

  /**
   * send Applist->CollisionObject.cpp->Moveit，set the collision
   */
  bool SvcCallback_apple_coll(arm_control::ServiceInt::Request &req,
                              arm_control::ServiceInt::Request &res);

  bool SvcCallback_AppleGrasp(arm_control::ServiceInt::Request &req,
                              arm_control::ServiceInt::Response &res);

public:
  ros::Publisher pub_AppleMsg;
  void UpdateAppleTF();
  AppleTrack(ros::NodeHandle &node);
  ~AppleTrack(){};//TODO:the space of "new" operations should be freed in AppleList
};

#endif

/*******************************************************************************
 *
 * @file CollisionObjectsNew.hpp
 *
 * @author lee-shun
 *
 * @email 2015097272@qq.com
 *
 * @data today
 *
 * @version 1.0
 *
 * @brief 在规划场景之中添加指定的避障对象
 *
 * @copyright
 *
 *******************************************************************************/
#ifndef __COLLOSION_OBJECTS_NEW__
#define __COLLOSION_OBJECTS_NEW__

#include "SceneBase.h"
#include "Transforms.h"
#include <arm_control/CollCheck.h>
#include <arm_control/ServiceInt.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

class CollHandler : public SceneBase {
private:
  const std::vector<std::string> COLL_OBJ_NAMES = {"mobile_base",
                                                   "gripper_base", "gripper"};
  /* 允许忽略某些碰撞，不检查 */
  collision_detection::AllowedCollisionMatrix acm;
  geometry_msgs::TransformStamped T_E_C_msg;
  tf2::Transform T_W_B;

  /* Publishers */
  ros::Publisher pub_set_joint;
  ros::Subscriber sub_joint_state;
  ros::Subscriber sub_add_apple;
  ros::ServiceServer svc_CollCheck;
  ros::ServiceServer svc_CollGripper;

  bool GripperCollisionEnabled = true;

  /**
   * 基座的外形
   */
  const double platform_height = 0.05;
  const double platform_size = 1.0;
  const double platform_thick = 0.05;
  const double platform_recess = 0.2;
  const double base_size = 0.15;
  const double cbox_height = 0.1;
  const double cbox_behind = 0.4;
  const double cbox_size = 0.4;

  /**
   * 抓手以及相机的外形
   * 可外部载入
   */
  double gripper_base_rad = 0.05;
  double gripper_base_length = 0.20;
  double gripper_length = 0.13;
  double gripper_rad = 0.07;
  double realsense_dist = 0.02;
  double realsense_depth = 0.08;
  double realsense_width = 0.14;
  double realsense_height = 0.06;

  moveit_msgs::CollisionObject cobj_mobile_base;
  moveit_msgs::AttachedCollisionObject cobj_gripper_base, cobj_gripper;

  /* Service callbacks */
  bool SvcCallback_CollCheck(arm_control::CollCheck::Request &req,
                             arm_control::CollCheck::Response &res);
  bool
  SvcCallback_CollisionGripperEnabled(arm_control::ServiceInt::Request &req,
                                      arm_control::ServiceInt::Response &res);
  /* Collision functions */
  bool AddBaseObjects();

  bool AddGripperObjects(std::string baseframe);

  bool AddCollisionGripper(bool add, double delay = 0);

  /* Subscriber callback */
  void SubCallback_process_coll_msg(
      const moveit_msgs::CollisionObject::ConstPtr &msg);

public:
  CollHandler(ros::NodeHandle &node, const std::string &PLANNING_GROUP,
              bool loadbase = true, bool loadgripper = true);
  /**
   * TODO:此处注意要删除new获得对象空间
   */
  ~CollHandler(){};
};

#endif

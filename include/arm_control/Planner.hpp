#ifndef _Planner_HPP_
#define _Planner_HPP_

/*******************************************************************************
 *
 * @file Planner.hpp
 *
 * @author lee-shun
 *
 * @email 2015097272@qq.com
 *
 * @version 1.0
 *
 * @brief
 *
 * @copyright
 *
 *******************************************************************************/
#include "SceneBase.h"
#include "Transforms.h"
#include <arm_control/PlanPath.h>
#include <arm_control/ServiceInt.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class Planner;

class PathPlan {
public:
  moveit::planning_interface::MoveGroupInterface::Plan Plan;

public:
  moveit_msgs::RobotTrajectory Trajectory;
  std::vector<double> endJointPos;
  std::vector<double> startJointPos;
  int ID;

  PathPlan(int id, moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    ID = id;
    Plan = plan;
    Trajectory = Plan.trajectory_;
    startJointPos = Plan.start_state_.joint_state.position;
    endJointPos = Plan.trajectory_.joint_trajectory.points.back().positions;
  }

  PathPlan(int id, const moveit_msgs::RobotTrajectory &trajectory,
           const std::vector<double> &start) {
    ID = id;
    Trajectory = trajectory;
    startJointPos = start;
    endJointPos = Trajectory.joint_trajectory.points.back().positions;
  }

  ~PathPlan() {}
};

class Planner : public SceneBase {
private:
  /* CollHandler* CollisionHandler; */
  ros::ServiceServer srv_PlanPath;
  ros::ServiceServer srv_PlanExecute;
  ros::Subscriber sub_EnableWall;
  std::vector<PathPlan> PathPlans;
  std::vector<geometry_msgs::Pose> Waypoints;
  ros::ServiceClient svc_EnableGripper;
  int ID = 0;
  bool GripperEnabled = false;

  /* Wall object */
  moveit_msgs::CollisionObject cobj_wall;

  bool SvcCallback_PlanPath(arm_control::PlanPath::Request &req,
                            arm_control::PlanPath::Response &res);
  bool SvcCallback_PlanExecute(arm_control::ServiceInt::Request &req,
                               arm_control::ServiceInt::Response &res);
  bool SubClientGripper();

  void SubCallback_Wall(const std_msgs::Float64MultiArray::ConstPtr &msg);
public:
  Planner(ros::NodeHandle &node, const std::string &PLANNING_GROUP);
};
#endif

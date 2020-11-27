#include "../include/arm_control/SceneBase.h"

ros::NodeHandle *RosNode;
moveit::planning_interface::MoveGroupInterface *ur5_move_group_interface;
moveit::planning_interface::PlanningSceneInterface
    *ur5_planning_scene_interface;
planning_scene::PlanningScene *ur5_planning_scene;
const robot_state::JointModelGroup *ur5_joint_model_group;

std::string SceneBase::PrintVector(const std::vector<double> &v, int precis) {
  std::stringstream ss;
  ss << "(" << std::fixed << std::setprecision(precis);
  for (int i = 0; i < v.size(); i++) {
    ss << v[i];
    if (i < (v.size() - 1)) {
      ss << ", ";
    }
  }
  ss << ")";
  return ss.str();
}

std::string SceneBase::PrintVector(const geometry_msgs::Pose &v, int precis) {
  std::stringstream ss;
  ss << "(" << std::fixed << std::setprecision(precis);
  ss << v.position.x << " " << v.position.y << " " << v.position.z << ", ";
  ss << v.orientation.x << " " << v.orientation.y << " " << v.orientation.z
     << " " << v.orientation.w << ")";
  return ss.str();
}

bool SceneBase::ApplyAttachedCollisionObject(
    moveit_msgs::AttachedCollisionObject &msg, double delay) {
  if (delay < 0) {
    delay = CALL_DILAY;
  }
  try {
    while (!ur5_planning_scene->processAttachedCollisionObjectMsg(msg)) {
      ROS_INFO_STREAM("Planning scene attached FUNK!");
    };
    while (!ur5_planning_scene_interface->applyAttachedCollisionObject(msg)) {
      // ROS_INFO_STREAM("Planning scene interface attached FUNK!");
      ros::Duration(delay).sleep();
    };
  } catch (...) {
    ROS_ERROR_STREAM(msg.object.id << " is not attached to the robot!");
  }
  return true;
}

/**
 *
 */
bool SceneBase::ApplyCollisionObject(moveit_msgs::CollisionObject &msg,
                                     double delay) {
  if (delay < 0) {
    delay = CALL_DILAY;
  }
  try {
    while (!ur5_planning_scene->processCollisionObjectMsg(msg)) {
      ROS_INFO_STREAM("Planning scene FUNK!");
    };
    while (!ur5_planning_scene_interface->applyCollisionObject(msg)) {
      // ROS_INFO_STREAM("Planning scene interface FUNK!");
      ros::Duration(delay).sleep();
    };
  } catch (...) {
    ROS_ERROR_STREAM(msg.id << " is not in the environment!");
  }
  return true;
}

SceneBase::SceneBase(ros::NodeHandle &node, const std::string PLANNING_GROUP) {
  RosNode = &node;

  if (PLANNING_GROUP.compare("") != 0) {
    ur5_move_group_interface =
        new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    ur5_joint_model_group =
        ur5_move_group_interface->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP); // Pointer for speed
  }

  ur5_planning_scene_interface =
      new moveit::planning_interface::PlanningSceneInterface; //&ur5_psi;

  robot_model_loader::RobotModelLoader ur5_ModelLoader("robot_description");
  robot_model::RobotModelPtr ur5_KinematicModel = ur5_ModelLoader.getModel();
  planning_scene::PlanningScene ur5_PlanningScene(ur5_KinematicModel);
  ur5_planning_scene = new planning_scene::PlanningScene(
      ur5_KinematicModel); // &ur5_PlanningScene;
}

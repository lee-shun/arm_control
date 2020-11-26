// ROS
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

// Custom messages
#include <ur5_control/CollCheck.h>
#include <ur5_control/ServiceInt.h>

// Base class
#include "obj/SceneBase.h"

// Custom matrix transformation definitions
#include "obj/Transforms.h"

class CollHandler : public SceneBase {
  // Constant strings
  const std::vector<std::string> COLL_OBJ_NAMES = {"mobile_base",
                                                   "gripper_base", "gripper"};
  collision_detection::AllowedCollisionMatrix acm;

  geometry_msgs::TransformStamped T_E_C_msg;
  tf2::Transform T_W_B;

  // Publishers
  ros::Publisher pub_set_joint;
  ros::Subscriber sub_joint_state;
  ros::Subscriber sub_add_apple;
  ros::ServiceServer svc_CollCheck;
  ros::ServiceServer svc_CollGripper;

  bool GripperCollisionEnabled = true;

  // Object descriptions
  const double platform_height = 0.05;
  const double platform_size = 1.0;
  const double platform_thick = 0.05;
  const double platform_recess = 0.2;
  const double base_size = 0.15;
  const double cbox_height = 0.1;
  const double cbox_behind = 0.4;
  const double cbox_size = 0.4;

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

  // Service callbacks
  bool SvcCallback_CollCheck(ur5_control::CollCheck::Request &req,
                             ur5_control::CollCheck::Response &res) {
    sensor_msgs::JointState joint;
    collision_detection::CollisionRequest collision_request,
        collision_request_self;
    collision_detection::CollisionResult collision_result,
        collision_result_self;

    // Get robot current state pointer
    robot_state::RobotState &ur5_state =
        ur5_planning_scene->getCurrentStateNonConst();
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;

    if (!GripperCollisionEnabled) {
      ROS_INFO("Gripper collision ignored.");
    }

    // Set joint position to robot current state
    joint.name = JOINT_NAMES;
    if (req.JointPosition.size() < 6) {
      ROS_WARN("Returning collision of current robot state.");
      joint.position = ur5_move_group_interface->getCurrentJointValues();
    } else {
      joint.position = req.JointPosition;
      ur5_state.setJointGroupPositions(ur5_joint_model_group, joint.position);
      ur5_planning_scene->setCurrentState(ur5_state);
    }

    // Check self collision
    ur5_planning_scene->checkSelfCollision(collision_request_self,
                                           collision_result_self);
    ROS_INFO_STREAM("Current state "
                    << (collision_result_self.collision ? "IS" : "is not")
                    << " in self collision.");

    // Check environment collision if not in self collision
    if (!collision_result_self.collision) {
      ur5_planning_scene->checkCollision(collision_request, collision_result,
                                         ur5_state, acm);
      ROS_INFO_STREAM("Current state "
                      << (collision_result.collision ? "IS" : "is not")
                      << " in collision with environment.");
    }

    res.Collision = (collision_result.collision ? 1 : 0) +
                    (collision_result_self.collision ? 2 : 0);

    if (req.VisualiseRobot) {
      pub_set_joint.publish(joint);
    }

    return true;
  }

  bool
  SvcCallback_CollisionGripperEnabled(ur5_control::ServiceInt::Request &req,
                                      ur5_control::ServiceInt::Response &res) {
    bool success = false;
    if (req.Data == -1) {
      res.Response = GripperCollisionEnabled;
      success = true;
    } else {
      success = true;
      if (GripperCollisionEnabled != req.Data) {
        GripperCollisionEnabled = req.Data;
        success = AddCollisionGripper(GripperCollisionEnabled);
      }
      res.Response = GripperCollisionEnabled;
    }
    ROS_INFO_STREAM("Gripper collisions "
                    << (req.Data == -1 ? "currently " : "")
                    << (GripperCollisionEnabled ? "en" : "dis") << "abled.");
    return success;
  }

  // Collision functions
  bool AddBaseObjects() {
    // Define mobile base collision object
    cobj_mobile_base.header.frame_id = ur5_planning_scene->getPlanningFrame();
    cobj_mobile_base.id = COLL_OBJ_NAMES[0]; // mobile_base
    cobj_mobile_base.operation = cobj_mobile_base.ADD;
    // Mobile platform

    /*
    shape_msgs::SolidPrimitive platform;
    platform.type = platform.BOX;
    platform.dimensions = {platform_size, platform_size, platform_thick};

    geometry_msgs::Pose platform_pose;
    platform_pose.orientation.w = 1.0;
    platform_pose.position.x = -platform_size / 2 + platform_recess;
    platform_pose.position.y = 0.0;
    platform_pose.position.z = -platform_height - platform_thick / 2;
    ApplyTransform2Pose(platform_pose,T_W_B);
    cobj_mobile_base.primitives.push_back(platform);
    cobj_mobile_base.primitive_poses.push_back(platform_pose);*/
    // Base stand
    shape_msgs::SolidPrimitive base;
    base.type = base.BOX;
    base.dimensions = {1.0, 1.0, platform_height};
    geometry_msgs::Pose base_pose;
    base_pose.orientation.w = 1.0;
    base_pose.position.x = 0.0;
    base_pose.position.y = 0.0;
    base_pose.position.z = -platform_height / 2;
    ApplyTransform2Pose(base_pose, T_W_B);
    cobj_mobile_base.primitives.push_back(base);
    cobj_mobile_base.primitive_poses.push_back(base_pose);
    // Control box
    /*
    shape_msgs::SolidPrimitive cbox;
    cbox.type = platform.BOX;
    cbox.dimensions = {cbox_size, cbox_size, platform_height + cbox_height};
    geometry_msgs::Pose cbox_pose;
    cbox_pose.orientation.w = 1.0;
    cbox_pose.position.x = -cbox_behind - cbox_size / 2;
    cbox_pose.position.y = 0.0;
    cbox_pose.position.z = (cbox_height - platform_height) / 2;
    ApplyTransform2Pose(cbox_pose,T_W_B);
    cobj_mobile_base.primitives.push_back(cbox);
    cobj_mobile_base.primitive_poses.push_back(cbox_pose);*/

    return ApplyCollisionObject(cobj_mobile_base);
  }

  bool AddGripperObjects(std::string baseframe) {
    // Define gripper base collision object
    cobj_gripper_base.link_name = baseframe;
    cobj_gripper_base.object.header.frame_id = baseframe;
    cobj_gripper_base.object.id = COLL_OBJ_NAMES[1]; // gripper_base
    cobj_gripper_base.object.operation = cobj_gripper_base.object.ADD;
    // Gripper base
    shape_msgs::SolidPrimitive gripper_base;
    gripper_base.type = gripper_base.CYLINDER;
    gripper_base.dimensions = {gripper_base_length - 0.01, gripper_base_rad};
    geometry_msgs::Pose gripper_base_pose;
    gripper_base_pose.orientation.w = 1;
    gripper_base_pose.position.x = 0.0;
    gripper_base_pose.position.y = 0.0;
    gripper_base_pose.position.z = gripper_base_length / 2 + 0.01;
    cobj_gripper_base.object.primitives.push_back(gripper_base);
    cobj_gripper_base.object.primitive_poses.push_back(gripper_base_pose);
    // Realsense
    shape_msgs::SolidPrimitive realsense;
    realsense.type = realsense.BOX;
    realsense.dimensions = {realsense_width, realsense_height, realsense_depth};
    geometry_msgs::Pose realsense_pose;
    realsense_pose.orientation.w = 1;
    realsense_pose.position.x = 0.0;
    realsense_pose.position.y = -(gripper_base_rad + realsense_height / 2);
    realsense_pose.position.z = realsense_depth / 2 + realsense_dist;
    cobj_gripper_base.object.primitives.push_back(realsense);
    cobj_gripper_base.object.primitive_poses.push_back(realsense_pose);

    // Define gripper collision object
    cobj_gripper.link_name = baseframe;
    cobj_gripper.object.header.frame_id = baseframe;
    cobj_gripper.object.id = COLL_OBJ_NAMES[2]; // gripper

    shape_msgs::SolidPrimitive shp_gripper;
    shp_gripper.type = shp_gripper.CYLINDER;
    shp_gripper.dimensions = {gripper_length, gripper_rad};
    geometry_msgs::Pose shp_gripper_pose;
    shp_gripper_pose.orientation.w = 1;
    shp_gripper_pose.position.x = 0.0;
    shp_gripper_pose.position.y = 0.0;
    shp_gripper_pose.position.z = gripper_base_length + gripper_length / 2;
    cobj_gripper.object.primitives.push_back(shp_gripper);
    cobj_gripper.object.primitive_poses.push_back(shp_gripper_pose);

    return ApplyAttachedCollisionObject(cobj_gripper_base) &&
           AddCollisionGripper(true);
  }

  bool AddCollisionGripper(bool add, double delay = 0) {
    if (add) {
      cobj_gripper.object.operation = cobj_gripper.object.ADD;
      return ApplyAttachedCollisionObject(cobj_gripper, delay);
    } else {
      cobj_gripper.object.operation = cobj_gripper.object.REMOVE;
      return ApplyAttachedCollisionObject(cobj_gripper, delay) &&
             ApplyCollisionObject(cobj_gripper.object, delay);
    }
  }

  // Subscriber callback
  void SubCallback_process_coll_msg(
      const moveit_msgs::CollisionObject::ConstPtr &msg) {
    moveit_msgs::CollisionObject obj;
    obj.header = msg->header;
    obj.id = msg->id;
    obj.primitive_poses = msg->primitive_poses;
    obj.primitives = msg->primitives;
    obj.operation = msg->operation;

    // ROS_INFO_STREAM("Received " << obj.id << " " << obj.operation);

    ApplyCollisionObject(obj);
    if (msg->operation == msg->ADD) {
      ROS_INFO_STREAM("Added " << msg->id);
    } else {
      ROS_INFO_STREAM("Removed " << msg->id);
    }
  }

  // Class Initialiser
public:
  CollHandler(ros::NodeHandle &node, const std::string &PLANNING_GROUP,
              bool loadbase = true, bool loadgripper = true)
      : SceneBase{node, PLANNING_GROUP} {
    std::map<std::string, moveit_msgs::AttachedCollisionObject> collaobj =
        ur5_planning_scene_interface->getAttachedObjects();
    for (auto i : collaobj) {
      i.second.object.operation = i.second.object.REMOVE;
      ur5_planning_scene_interface->applyAttachedCollisionObject(i.second);
      ROS_INFO_STREAM("Detached " << i.first << " from " << i.second.link_name
                                  << ".");
    }

    std::map<std::string, moveit_msgs::CollisionObject> collobj =
        ur5_planning_scene_interface->getObjects();
    std::vector<std::string> collname;
    for (auto i : collobj) {
      i.second.operation = i.second.REMOVE;
      ur5_planning_scene_interface->applyCollisionObject(i.second);
      ROS_INFO_STREAM("Removed " << i.first << " from existing scene.");
    }

    // ROS_INFO_STREAM("Stuff");
    if (loadbase || loadgripper) {
      // Get and set Allowed Collision Matrix
      acm = ur5_planning_scene->getAllowedCollisionMatrix();
      acm.setEntry("mobile_base", false);
      acm.setEntry("mobile_base", "base_link", true);
      pub_set_joint = RosNode->advertise<sensor_msgs::JointState>(
          "move_group/fake_controller_joint_states", 1000);
      svc_CollCheck =
          RosNode->advertiseService("ur5_control/scene/collision/check",
                                    &CollHandler::SvcCallback_CollCheck, this);
    }
    TFBuffer = new tf2_ros::Buffer();
    TFListener = new tf2_ros::TransformListener(*TFBuffer);
    sub_add_apple = RosNode->subscribe<moveit_msgs::CollisionObject>(
        "ur5_control/scene/collision/process_msg", 1000,
        &CollHandler::SubCallback_process_coll_msg, this);
    try {
      geometry_msgs::TransformStamped T_W_B_msg;
      ros::Duration tfwait(5);
      T_W_B_msg = TFBuffer->lookupTransform("world", "mobile_base",
                                            ros::Time(0), tfwait);
      tf2::convert(T_W_B_msg.transform, T_W_B);
      ROS_INFO_STREAM("Applied world -> mobile_base transform to mobile base "
                      "collision object.");
    } catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM("Frame mobile_base not found. " << ex.what());
      tf2::Quaternion q;
      q.setRPY(0, 0, -M_PI / 4);
      T_W_B = tf2::Transform(q, tf2::Vector3(0, 0, 0));
    }

    if (loadbase) {
      AddBaseObjects();
    }
    if (loadgripper) {
      std::string baseframe;
      node.param<std::string>("apple_gripper/parent_frame", baseframe, "tool0");
      node.param<double>("apple_gripper/base/length", gripper_base_length, 0.2);
      node.param<double>("apple_gripper/base/radius", gripper_base_rad, 0.05);
      node.param<double>("apple_gripper/claw/length", gripper_length, 0.13);
      node.param<double>("apple_gripper/claw/radius", gripper_rad, 0.07);
      node.param<double>("realsense/distx", realsense_dist, 0.02);
      node.param<double>("realsense/depth", realsense_depth, 0.08);
      node.param<double>("realsense/width", realsense_width, 0.14);
      node.param<double>("realsense/height", realsense_height, 0.06);
      AddGripperObjects(baseframe);
      svc_CollGripper = RosNode->advertiseService(
          "ur5_control/scene/collision/gripper_coll_enabled",
          &CollHandler::SvcCallback_CollisionGripperEnabled, this);
    }
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "ur5_control_collobjects");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1);

  // std::cout << node.getNamespace() << std::endl;

  spinner.start();

  // Set up planning scene
  std::string PLANNING_GROUP;
  node.param<std::string>("planning_group", PLANNING_GROUP, "manipulator");

  bool loadbase = true;
  ros::param::get("~load_base", loadbase);
  if (~loadbase) {
    ROS_INFO_STREAM("Mobile base collision model will not be loaded.");
  }

  bool loadgripper = true;
  ros::param::get("~load_gripper", loadgripper);
  if (~loadgripper) {
    ROS_INFO_STREAM("Gripper collision model will not be loaded.");
  }

  // Construct collision handler class
  CollHandler CollisionHandler(node, PLANNING_GROUP, loadbase, loadgripper);

  ROS_INFO("Static collision objects successfully added.");

  ros::waitForShutdown();

  return 0;
}

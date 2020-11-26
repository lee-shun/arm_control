// ROS
#include <ros/ros.h>

// Package message headers
#include <ur5_control/ServiceInt.h>
#include <ur5_control/PlanPath.h>
#include <std_msgs/Float64MultiArray.h>

// Other headers
#include <geometry_msgs/PoseArray.h>

// Base scene definitions
#include "obj/SceneBase.h"
#include "obj/Transforms.h"

class Planner: public SceneBase{ 
  class PathPlan{
   public:
    moveit::planning_interface::MoveGroupInterface::Plan Plan;
    
   public:
    moveit_msgs::RobotTrajectory Trajectory;// = NULL;
    std::vector<double> endJointPos;
    std::vector<double> startJointPos;
    int ID;

    PathPlan(int id, moveit::planning_interface::MoveGroupInterface::Plan &plan){//, std::vector<double> &start, std::vector<double> &end){
      ID = id;
      Plan = plan;
      Trajectory = Plan.trajectory_;
      startJointPos = Plan.start_state_.joint_state.position;
      endJointPos = Plan.trajectory_.joint_trajectory.points.back().positions;
    }

    PathPlan(int id, const moveit_msgs::RobotTrajectory &trajectory, const std::vector<double> &start){
      ID = id;
      Trajectory = trajectory;
      startJointPos = start;
      endJointPos = Trajectory.joint_trajectory.points.back().positions;
    }

    ~PathPlan(){/*
      if (Trajectory != NULL){ delete Trajectory; }
      ROS_INFO_STREAM("What1");
      if (Plan != NULL){ delete Plan; }
      ROS_INFO_STREAM("What2");*/
    }

  };

  //CollHandler* CollisionHandler;
  ros::ServiceServer srv_PlanPath;
  ros::ServiceServer srv_PlanExecute;
  ros::Subscriber sub_EnableWall;
  //actionlib::SimpleActionClient<vision_v2::detectionAction> *asv_SegmentApple;
  std::vector<PathPlan> PathPlans;
  std::vector<geometry_msgs::Pose> Waypoints;
  ros::ServiceClient svc_EnableGripper;
  int ID = 0;
  bool GripperEnabled = false;
  
  // Wall object
  moveit_msgs::CollisionObject cobj_wall;

  bool SvcCallback_PlanPath(ur5_control::PlanPath::Request &req, ur5_control::PlanPath::Response &res){
    bool plan_success = false;
    std::vector<double> joint_pos0;
    if (req.ClearAllPlans){
      PathPlans.clear();
      ROS_INFO("Removed all path plans.");
    }
    else if (req.JointPosition.size() > 0 || req.AddAsJointSpace){
      moveit::planning_interface::MoveGroupInterface::Plan js_plan;
      std::vector<double> joint_pos;
     
      robot_state::RobotState* start_state = 0;
      if (PathPlans.size() > 0){
        joint_pos0 = PathPlans.back().endJointPos;
        start_state = new robot_state::RobotState(*ur5_move_group_interface->getCurrentState());
        start_state->setJointGroupPositions(ur5_joint_model_group,joint_pos0);
        ur5_move_group_interface->setStartState(*start_state);
      }
      else{
        ur5_move_group_interface->setStartStateToCurrentState();
        joint_pos0 = ur5_move_group_interface->getCurrentJointValues();
      }
      
      bool pose_valid = true;
      if (req.AddAsJointSpace){
        if (start_state == 0){
          start_state = new robot_state::RobotState(*ur5_move_group_interface->getCurrentState());
        }
        pose_valid = start_state->setFromIK(ur5_joint_model_group, req.Pose);
        ROS_INFO_STREAM("IK was " << (pose_valid ? "" : "NOT " ) << "solved successfully.");
        if (pose_valid){
          start_state->copyJointGroupPositions(ur5_joint_model_group, joint_pos);
        }
      }
      else{
        joint_pos = req.JointPosition;
      }
      
      if (pose_valid){
        if (req.Speed == 0){
          req.Speed = 0.2;
        }
        ur5_move_group_interface->setMaxVelocityScalingFactor(req.Speed);
        ur5_move_group_interface->setMaxAccelerationScalingFactor(req.Speed);
        ur5_move_group_interface->setJointValueTarget(joint_pos);
        plan_success = (ur5_move_group_interface->plan(js_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      }
      else{
        plan_success = false;
      }

      ROS_INFO_STREAM("Plan from " << std::endl << "  " << PrintVector(joint_pos0) << " to " << 
        std::endl << "  " << PrintVector(joint_pos) << (plan_success ? " " : " NOT ") << "successful.");
      if (plan_success){
        PathPlans.push_back(PathPlan(ID, js_plan));
        ID++;
        res.PlanID = PathPlans.back().ID;
      }
      else{
        res.PlanID = -1;
      }
    }
    else if (req.ClearAllWaypoints){
      Waypoints.clear();
      ROS_INFO("All waypoints removed.");
    }
    else {
      moveit_msgs::RobotTrajectory trajectory;
      geometry_msgs::Pose pose0;

      if (PathPlans.size() > 0){
        joint_pos0 = PathPlans.back().endJointPos;
      }
      else{
        joint_pos0 = ur5_move_group_interface->getCurrentJointValues();
      }

      if (Waypoints.size() > 0){
        pose0 = Waypoints.back();
      }
      else if (PathPlans.size() > 0){
        robot_state::RobotState start_state(*ur5_move_group_interface->getCurrentState());
        start_state.setJointGroupPositions(ur5_joint_model_group,joint_pos0);        
        ur5_move_group_interface->setStartState(start_state);
        pose0 = ur5_move_group_interface->getCurrentPose().pose;
      }
      else{
        ur5_move_group_interface->setStartStateToCurrentState();
        pose0 = ur5_move_group_interface->getCurrentPose().pose;
      }
      Waypoints.push_back(req.Pose);
      if (req.AddWaypointOnly){
        ROS_INFO_STREAM("Added waypoint " << Waypoints.size() << " from" << std::endl << 
          "  " << PrintVector(pose0) << " to " << std::endl << 
          "  " << PrintVector(req.Pose));
      }
      else{
        if (req.Speed < 0){
          req.Speed = 0.0;
        }
        else if (req.Speed == 0){
          req.Speed = 3.0;
        }

        //bool GripperCollEnabled = CollisionHandler->GripperCollisionEnabled;
        ur5_control::ServiceInt GripperColl;
        /*
        GripperColl.request.Data = -1;
        svc_EnableGripper.call(GripperColl);
        */

        //ROS_INFO_STREAM((GripperEnabled ? "Gripper is enabled..." : "Gripper is NOT enabled..."));

        if (GripperEnabled && req.ExcludeGripper){
          GripperColl.request.Data = 0;
          while (!svc_EnableGripper.call(GripperColl)){
            ROS_WARN("Failed to call gripper collision service to disable collision.");
          }

          /*
          ur5_control::ServiceInt::Request greq;
          ur5_control::ServiceInt::Response grep;
          greq.Data = 0;
          CollisionHandler->SvcCallback_CollisionGripperEnabled(greq, grep);
          */
        }

        double fraction = ur5_move_group_interface->computeCartesianPath(Waypoints, 0.01, req.Speed, trajectory);
        plan_success = fraction > 0.999;

        std::stringstream ss;
        ss << "Path from " << std::endl << "  Base point: " <<PrintVector(pose0) << " to" << std::endl;
        for (int i = 0; i < Waypoints.size(); i++){
          ss << "  Waypoint " << i+1 << ": " << PrintVector(Waypoints[i]);
          if (i < Waypoints.size()-1){
            ss << " to" << std::endl;
          }
        }
        ROS_INFO_STREAM(ss.str() << " was " << fraction*100 << "% feasible. Plan " << (plan_success ? "" : "NOT ") << "successful.");

        if (GripperEnabled && req.ExcludeGripper){
          GripperColl.request.Data = 1;
          while (!svc_EnableGripper.call(GripperColl)){
            ROS_WARN("Failed to call gripper collision service to enable collision.");
          }
          /*
          ur5_control::ServiceInt::Request greq;
          ur5_control::ServiceInt::Response grep;
          greq.Data = 1;
          CollisionHandler->SvcCallback_CollisionGripperEnabled(greq, grep);
          */
        }

        if (plan_success){
          PathPlans.push_back(PathPlan(ID,trajectory,joint_pos0));
          ID++;
          res.PlanID = PathPlans.back().ID;
          ROS_INFO("Waypoints cleared.");
          Waypoints.clear();
        }
        else{
          Waypoints.pop_back();
          ROS_INFO("Waypoint not added.");
          res.PlanID = -1;
        }
      }
    }

    res.PlansInQueue = PathPlans.size();
    res.WaypointsInQueue = Waypoints.size();
    res.PlanSuccess = plan_success;
    return true;
    
  }

  bool SvcCallback_PlanExecute(ur5_control::ServiceInt::Request &req, ur5_control::ServiceInt::Response &res){
    int id = 0;
    try{
      int n = PathPlans.size();
      //ur5_move_group_interface->setStartStateToCurrentState();
      while (PathPlans.size() > 0){
        ROS_INFO_STREAM("Executing Plan ID " << PathPlans[0].ID << " (" << ++id << "/" << n << ")");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = PathPlans[0].Trajectory;
        ur5_move_group_interface->execute(plan);
        PathPlans.erase(PathPlans.begin());
        ROS_INFO_STREAM("Execution of Plan " << PathPlans[0].ID << " successful.");
      }
      ROS_INFO_STREAM("All plans executed successfully.");
      res.Response = id;
      return true;
    }
    catch(...){
      res.Response = id;
      return false;
    }
  }

  bool SubClientGripper(){
    bool flag = svc_EnableGripper.waitForExistence(ros::Duration(1.0));
    if (!flag){
      for (int i = 0; i < 3; i++){
        flag = svc_EnableGripper.waitForExistence(ros::Duration(1.0));
        if (flag){
          svc_EnableGripper = RosNode->serviceClient<ur5_control::ServiceInt>("ur5_control/scene/collision/gripper_coll_enabled",1);
          break;
        }
      }
      if (!flag){
        ROS_WARN("Gripper collision object not loaded. Skipping.");
      }
    }
    return flag;
  }

  void SubCallback_Wall(const std_msgs::Float64MultiArray::ConstPtr &msg){
    std::vector<double> p_(msg->data.begin(),msg->data.end());
    div_t numel = div (p_.size(), 9);
    
    cobj_wall.operation = cobj_wall.REMOVE;
    ApplyCollisionObject(cobj_wall);
    cobj_wall.primitives.clear();
    cobj_wall.primitive_poses.clear();

    if (numel.quot == 0 || numel.rem > 0){
      ROS_INFO_STREAM("Wall removed.");
    }
    else{
      while (p_.size() > 0){
        std::vector<double> p(p_.begin(), p_.begin()+9);
        shape_msgs::SolidPrimitive wall_obj;
        wall_obj.type = wall_obj.BOX;
        wall_obj.dimensions = {p[6],p[7],p[8]};
        geometry_msgs::Pose wall_pose;
        wall_pose.position.x = p[0];
        wall_pose.position.y = p[1];
        wall_pose.position.z = p[2];
        tf2::Quaternion q;
        q.setRPY(p[3],p[4],p[5]);
        wall_pose.orientation = tf2::toMsg(q);
        cobj_wall.primitives.push_back(wall_obj);
        cobj_wall.primitive_poses.push_back(wall_pose);
        ROS_INFO_STREAM("Setting wall: " << PrintVector(p));
        p_.erase(p_.begin(), p_.begin()+9);
      }
      cobj_wall.operation = cobj_wall.ADD;
      ApplyCollisionObject(cobj_wall);
    }
    
  }

 public:
  Planner(ros::NodeHandle &node, const std::string &PLANNING_GROUP) : SceneBase{node, PLANNING_GROUP}
  {
    StaticTFBroadcaster = new tf2_ros::StaticTransformBroadcaster;
    std::vector<std::string> keys;
    node.getParamNames(keys);
    for (auto i : keys){
      std::size_t idx1, idx2;
      idx1 = i.find("planning_frame");
      if (idx1 != std::string::npos){
        idx2 = i.find("/parent");
        if (idx2 != std::string::npos){
          StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, i.substr(16,idx2-16)));
        }
      }
    }

    /*
    StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, "mobile_base"));
    StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, "camera_link"));
    StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, "gripper_tip"));
    StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, "gripper_grasp"));
    StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, "apple_inspect"));
    StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, "apple_approach"));
    StaticTFBroadcaster->sendTransform(paramFrameToMsg(node, "apple_touch"));
    */

    srv_PlanPath = RosNode->advertiseService("ur5_control/plan/queue", &Planner::SvcCallback_PlanPath, this);
    srv_PlanExecute = RosNode->advertiseService("ur5_control/plan/execute", &Planner::SvcCallback_PlanExecute, this);
    svc_EnableGripper = RosNode->serviceClient<ur5_control::ServiceInt>("ur5_control/scene/collision/gripper_coll_enabled");
    sub_EnableWall = RosNode->subscribe<std_msgs::Float64MultiArray>("ur5_control/scene/collision/wall",100, &Planner::SubCallback_Wall, this);

    // Wait 5 seconds for gripper collision service to come up (requires mobile base TF)
    GripperEnabled = SubClientGripper();
    if (GripperEnabled){
      ROS_INFO_STREAM("Gripper collision model detected.");
    }
   
    // Set up wall object
    shape_msgs::SolidPrimitive wall_obj;
    cobj_wall.header.frame_id = "world";
    cobj_wall.id = "wall";
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "ur5_control_planner");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(2); 

  //std::cout << node.getNamespace() << std::endl;

  spinner.start();

  // Set up planning scene
  std::string PLANNING_GROUP;
  node.param<std::string>("planning_group", PLANNING_GROUP, "manipulator");

  // Construct path planning class
  Planner PathPlanner(node, PLANNING_GROUP);//, CollisionHandler);

  ROS_INFO("Planner successfully loaded.");

  ros::waitForShutdown();

  return 0;
}

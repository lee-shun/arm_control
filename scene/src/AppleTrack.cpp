// ROS
#include <ros/ros.h>

// Package message headers
#include <ur5_control/ServiceInt.h>
#include <ur5_control/PlanPath.h>

// Other headers
#include <geometry_msgs/PoseArray.h>

// Base scene definitions
#include "obj/SceneBase.h"

class AppleTrack: public SceneBase{
 private:
  class Apple{
   public:
    enum class AppleStatus {Unpicked, Picked, Missing};
    int id = -1;
    AppleStatus status = AppleStatus::Unpicked;
    double apple_rad = 0.04;
    moveit_msgs::CollisionObject cobj_apple;
    geometry_msgs::TransformStamped T_W_A_msg;
    //geometry_msgs::Pose pose;

    AppleTrack *SceneBasePtr;

    Apple(int id_, geometry_msgs::Pose pose_, AppleTrack *scenebase){

      id = id_;
      SceneBasePtr = scenebase;
      //pose = pose_;
      // Define apple collision object
      //cobj_apple.link_name = "world";
      cobj_apple.header.frame_id = "world";
      std::stringstream ss;
      ss << "apple_" << id;
      cobj_apple.id = ss.str(); //gripper
    
      shape_msgs::SolidPrimitive shp_apple;
      shp_apple.type = shp_apple.SPHERE;
      shp_apple.dimensions = {apple_rad};

      cobj_apple.primitives.push_back(shp_apple);
      cobj_apple.primitive_poses.push_back(pose_);

      cobj_apple.operation = cobj_apple.ADD;
      //SceneBasePtr->ApplyCollisionObject(cobj_apple);
      //ros::Duration(SceneBasePtr->CALL_DILAY).sleep();
      SceneBasePtr->pub_AppleMsg.publish(cobj_apple);
      geometry_msgs::TransformStamped *T = &T_W_A_msg;
      T->header.stamp = ros::Time::now();
      T->header.frame_id = "world";
      T->child_frame_id = ss.str();
      T->transform.rotation = pose_.orientation;
      T->transform.translation.x = pose_.position.x;
      T->transform.translation.y = pose_.position.y;
      T->transform.translation.z = pose_.position.z;
      //scenebase->StaticTFBroadcaster->sendTransform(*T);

    }

    ~Apple(){
      cobj_apple.operation = cobj_apple.REMOVE;
      //SceneBasePtr->ApplyCollisionObject(cobj_apple);
      T_W_A_msg.header.frame_id = "";
      //ROS_INFO_STREAM("Removed " << cobj_apple.id);
      SceneBasePtr->pub_AppleMsg.publish(cobj_apple);
    }

  };  

  void SubCallback_apple_new(const geometry_msgs::PoseArray::ConstPtr &msg)
  {
    for (auto &i : AppleList){
      delete i.second;
    }

    AppleList.clear();
    Apple *a;
    for (int i = 0; i < msg->poses.size(); i++){
      //Apple a(0,*msg->poses[i],ur5_planning_scene_interface,ur5_planning_scene);
      //geometry_msgs::Pose p = ;
      a = new Apple(i,msg->poses[i],this);
      //ApplyCollisionObject(a->cobj_apple);
      //std::cout << "x:" << p.position.x << std::endl;
      AppleList.insert({i,a});
    }
  }

  bool SvcCallback_apple_coll(ur5_control::ServiceInt::Request &req, ur5_control::ServiceInt::Request &res)
  {
    for (auto i : AppleList){
      if (req.Data > 0){
        i.second->cobj_apple.operation = i.second->cobj_apple.ADD;
      }
      else
      {
        i.second->cobj_apple.operation = i.second->cobj_apple.REMOVE;
      }
      pub_AppleMsg.publish(i.second->cobj_apple);
    }
    ROS_INFO_STREAM("Apple collisions " << (req.Data > 0 ? "enabled." : "disabled."));
    return true;
  }

  int AppleGraspedID = -1;
  bool SvcCallback_AppleGrasp(ur5_control::ServiceInt::Request &req, ur5_control::ServiceInt::Response &res){
    try{
      Apple* apple;
      if (AppleGraspedID > 0 || req.Data < 0){
        apple = AppleList.at(AppleGraspedID);
        cobj_apple_grasped.object = apple->cobj_apple;
        cobj_apple_grasped.object.operation = cobj_apple_grasped.object.REMOVE;
        ApplyAttachedCollisionObject(cobj_apple_grasped);

        std::map<std::string, geometry_msgs::Pose> a = ur5_planning_scene_interface->getObjectPoses({apple->cobj_apple.id});
        geometry_msgs::Pose p = a.at(apple->cobj_apple.id);
        apple->cobj_apple.primitive_poses = {p};
        ROS_INFO_STREAM(cobj_apple_grasped.object.id << " released at (" << std::fixed << std::setprecision(3)
          << p.position.x << ", " << p.position.y << ", " << p.position.z << ")");
      }

      if (AppleList.count(req.Data)>0){
        apple = AppleList.at(req.Data);

        std::map<std::string, geometry_msgs::Pose> a = ur5_planning_scene_interface->getObjectPoses({apple->cobj_apple.id});
        geometry_msgs::Pose p = a.at(apple->cobj_apple.id);

        apple->cobj_apple.operation = apple->cobj_apple.REMOVE;
        ApplyCollisionObject(apple->cobj_apple);
        ROS_INFO_STREAM(apple->cobj_apple.id << " removed from static world");

        apple->cobj_apple.operation = apple->cobj_apple.ADD;
        cobj_apple_grasped.object = apple->cobj_apple;
        ApplyAttachedCollisionObject(cobj_apple_grasped);

        ROS_INFO_STREAM(apple->cobj_apple.id << " grasped at (" << std::fixed << std::setprecision(3)
          << p.position.x << ", " << p.position.y << ", " << p.position.z << ")");
      }
      
      AppleGraspedID = req.Data;
      res.Response = AppleGraspedID;
    }
    catch(...){
      res.Response = -1;
    }
  }
  
  ros::Subscriber sub_apple_new;
  ros::ServiceServer svc_apple_coll;
  ros::ServiceServer svc_apple_grasp;
  std::map<int, Apple*> AppleList;
  moveit_msgs::AttachedCollisionObject cobj_apple_grasped;

 public:
  ros::Publisher pub_AppleMsg;
  void UpdateAppleTF(){
    for (auto i : AppleList){
      Apple *a = i.second;
      a->T_W_A_msg.header.stamp = ros::Time::now();
      TFBroadcaster->sendTransform(a->T_W_A_msg);
    }
    return;
  }

  AppleTrack(ros::NodeHandle &node) : SceneBase{node}
  {
    sub_apple_new = RosNode->subscribe("ur5_control/scene/apple_position_new", 1000, &AppleTrack::SubCallback_apple_new, this);
    svc_apple_coll = RosNode->advertiseService("ur5_control/scene/collision/apple_coll_enabled", &AppleTrack::SvcCallback_apple_coll, this);
    //svc_apple_grasp = RosNode->advertiseService("ur5_control/scene/apple_grasp_set", &AppleTrack::SvcCallback_AppleGrasp, this);
    cobj_apple_grasped.link_name = "gripper_grasp";
    TFBroadcaster = new tf2_ros::TransformBroadcaster();
    pub_AppleMsg = RosNode->advertise<moveit_msgs::CollisionObject>("ur5_control/scene/collision/process_msg",100);
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "ur5_control_apples");
  ros::NodeHandle node;
  ros::Rate r(10);

  // Construct apple tracking class
  AppleTrack AppleTracker(node);
  
  ROS_INFO("Apple tracker successfully loaded.");

  while (ros::ok()){
    AppleTracker.UpdateAppleTF();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

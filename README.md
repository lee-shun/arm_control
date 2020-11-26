# Aubo Low Level Control Code (ROS C++)
The low level control code was originally written for the UR5 for ROS Melodic (Ubuntu 18.04).  However, this has been modified to work on the Aubo robot in ROS Kinetic (Ubuntu 16.04) and tested successfully.

## Setting Up
### 1. Installation
These instructions should be followed any time there is an update to the control code.
1. Delete the old folders in your `catkin/src` folder: 
   * `ur5_control` (C++ low level control)
   * `apple` (MATLAB high level control) 
2. Extract the following folders into `catkin/src`, overwriting all files when asked
   * `ur5_control`
   * `vision_v2`
   * `control_test`
   * `apple`
3. Run `catkin_make` to compile the code.

### 2. Planning Frames
This section defines all frames used in path planning.  Some frames are fixed, while others can be modified, based on current gripper dimensions and camera locations.  Any time there is a change in the gripper design, the *custom-defined planning frames* should be modified accordingly.
#### Fixed Frames
##### World
The world frame is the fixed universal frame, and will not move with time.  This is automatically generated by the ROS/tf service.

##### Base link
The frame `base_link` defines the position of the base of the robot, relative to the `world` frame.  For the UR5 and Aubo i5 robots, `base_link` is coincident with `world`, and is generated by the robot's SRDF file.

##### End effector link
This frame is attached to the final frame of the robot and rotates with the wrist 3 frame.  It is called `ee_link`, where the outward distal length (along the gripper) is set as the *x*-axis.  This is defined by the robot's SRDF file (but manually added for the Aubo i5). Most gripper frames will be defined, relative to this frame.

#### Custom-defined Planning Frames
If you are setting this up for the first time, or you have a new gripper or changed the position of the Intel RealSense camera, you must adjust the planning frames to match the new dimensions (figure below).  Otherwise, path planning will not be accurate.

These frames should be modified in `config/planning_frames.yaml`.  The properties `position` and `orientation` are measured in metres and degrees respectively.  In addition, orientation is measured in fixed RPY (roll-pitch-yaw) angles (about x-y-z axes).

![Figure 1. Gripper frames.](https://gitlab.erc.monash.edu.au/apple-harvesting-robot/ur5-control/ur5-control-low-level/-/wikis/uploads/e825ba7b0e883dd5ec70363d3bd845f1/gripper_frames.png)
##### Mobile base
```
mobile_base: # New frame
  parent: "base_link"		# Parent frame
  position: [0, 0, 0] 		# Displacement (XYZ metres)
  orientation: [0, 0, 0] 	# Orientation (RPY degrees)
```
This is the location of the base of the robot, relative to the `base_link` frame (and also the `world` frame).  At LMGA, this orientation is set to -45 degrees in z about the base link frame, `[0, 0, -45]`.  For WeBull, this is set to `[0, 0, 0]` (coincident).
##### Camera link
```
camera_link:
  parent: "ee_link"
  position: [0.05425, 0.02, 0.1363]
  orientation: [0, 0, 0]
```
This is the location of the depth sensor on the RealSense camera, relative to the `ee_link`.  Without this definition, the RealSense data cannot be localised.  The accuracy of this frame is critically important for apple segmentation, where any slight offset in both position and orientation can affect the detected apple positions by more than 1 cm.
##### Gripper tip
```
gripper_tip:
  parent: "ee_link"
  position: [0.183, 0, 0]
  orientation: [0, 0, 0]
```
This frame defines the tip of the gripper, relative to the `ee_link`.  This serves as an intermediate frame to help define other frames related to apple grasping.
##### Gripper grasp
```
gripper_grasp:
  parent: "ee_link"
  position: [0.144, 0, 0]
  orientation: [0, 0, 0]
```
This frame defines where the apple should be located when a gripper has *grasped* an apple.  Upon a successful grasp, the apple frame is coincident with the `gripper_grasp` frame.
##### Apple inspect
```
apple_inspect:
  parent: "camera_link"
  position: [0.30, 0, 0]
  orientation: [0, 0, 0]
```
This frame defines where the (centre of the) apple should be when the robot is *inspecting* the apple for localisation and servoing purposes, before *approaching*.  At inspection, the apple frame is coincident with this frame.  In the default setting, the apple should be 30 cm directly in front of the RealSense camera.  This is why the position is set as `[0.30, 0, 0]`, relative to the `camera_link` frame.
##### Apple approach
```
apple_approach:
  parent: "gripper_tip"
  position: [0.08, 0, 0]
  orientation: [0, 0, 0]
```
This frame defines the apple location relative to the `gripper_tip` frame when *approaching* an apple before *grasping*.  By default, the centre of the apple should be 8 cm away from the gripper's end before grasping.
##### Apple touch
```
apple_touch:
  parent: "gripper_tip"
  position: [0.03, 0, 0]
  orientation: [0, 0, 0]
```
This frame defines the apple location relative to the `gripper_tip` frame when *touching* an apple.  This pose is for performing touch testing only, which can be used for servoing or testing the accuracy of path planning.  By default, the centre of the apple should be 3 cm from the gripper's tip.

### 3. Gripper Collision Object
Setting up the collision object is important to avoid damaging the gripper during the execution of complex trajectories.  All dimensions are relative to the `tool0` frame, which is attached to the rotating wrist 3 frame.  The dimensions can be modified in `config/gripper.yaml`, and allows you to set the gripper and RealSense camera dimensions in metres.
```
apple_gripper:
  parent_frame: "tool0"
  base:			# Gripper chassis before the claw
    length: 0.09
    radius: 0.05
  claw:			# Claw/finger part of the gripper
    length: 0.093
    radius: 0.08
realsense:		# RealSense collision box
  distx: 0.0		# Distance from tool0 frame0
  depth: 0.05425	# Thickness of the RealSense collision object
  width: 0.14 		# Width of the collision box
  height: 0.1 		# Height of the collision box
```

## Starting Up
1. **Launch the aubo_i5 driver and MoveIt! with RViz**, as you would if you want to control the physical robot, replacing `<robot_ip>` with the actual robot IP in the ROS network.
```
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=<robot ip>
```
2. **Launch low level ROS control interface**, which exposes low level ROS control commands that allow you to access all sensors, do path planning and control all aspects move the robot including grippers.  Note: any references to *UR5* can be ignored. They will work for the *aubo_i5*.
```
roslaunch ur5_control scene.launch
```
To ignore loading the mobile base collision object, add `load_base:=false` to the end of the above command.

To ignore loading the gripper collision object, add `load_gripper:=false` to the end of the above command.

If you see the following line, then the control code has been successfully compiled and loaded!
```
Planner successfully loaded.
```
3. **Launch the RealSense node** by opening another bash command window and type
```
roslaunch control_test realsense.launch
``` 
4. **Set up the RViz window** as recommended
   * *Planning Request > Goal State Alpha*, set to 0.2
   * *Scene Robot > Robot Alpha*, set to 0.5
   * `Add` > RViz > TF.  Set Marker Scale: 0.2 and uncheck Frames > All Enabled.  Then re-enable
      * apple_approach
      * apple_inspect
      * apple_touch
      * ee_link
      * gripper_grasp
      * mobile_base
   * `Add` > By topic > Image (raw).  Drag and dock the RealSense image to any convenient position in RViz.
5. **Execute the apple segmentation node** by opening another bash command window and typing
```
roscd vision_v2
python inference_realsense_new.py
```
6. **Execute the local apple servoing node** by opening another bash command window and typing
```
roscd vision_v2
python inference_realsense_local.py
```
7. **Launch the apple segmentation action service** by opening another bash command window and typing
```
roslaunch ur5_control segment_apple.launch
```
8. **Execute the gripper control node** by opening another bash command window and typing.
```
roscd ur5_control
cd gripper/scripts
python gripperwb.py
```

**Note:** We will try to combine steps 5 and 8 will combine at a later date.  They are separated for now because these services exist on separate computers at LMGA.

## ROS Interface
### 1. Apple Segmentation
#### 1.1 Detect All Apple Positions
#### Properties
Service: `/ur5_control/vision/segment_apple`  
Message: `geometry_msgs/PoseArray`  
Requested arguments:
* `int64` Data

Response arguments 
* `PoseArray` Poses
  (True if a path is successfully planned. False otherwise.)

#### Usage
Use this service to detect all apples within the viewport of the RealSense camera.
##### Segment apples in the World frame
Call this service, setting `Data = 0`.  The service will return `Poses` of the detected apples, relative to the World frame.
##### Segment apples in the Camera Link frame
Call this service, setting `Data = 1`.  The service will return `Poses` of the detected apples, relative to the `camera_link` frame.  This is useful for servoing purposes.

---
### 2. Path Planning
The control code exposes several new topics and services that allow low level planning of the UR5 robot, or any robot of a similar architecture such as Aubo.

#### 2.1 Drawing Apples in the Planner Scene

#### Properties
Topic: `/ur5_control/scene/apple_position_new`  
Message: `geometry_msgs/PoseArray`
#### Usage
Use this topic to place apples in the planning scene.  The apples are represented as spheres, and act as obstacles for path planning purposes.

To place apples in the planning scene, publish an array of `Pose` in the `geometry_msgs/PoseArray` message, where each pose represents the position and rotation (quaternion) of the apple.  The rotation of the apple defines the angle at which the end-effector will approach the apple.  In most cases, the end-effector will align itself relative to the x-axis of the apple to grasp.

Every new message published to this topic will remove all existing apple objects, and replace them with new ones.

---
### 2.2 Planning a Path
#### Properties
Service: `/ur5_control/plan/queue`  
Type: `ur5_control/PlanPath`  
Request arguments
* `float64[]` JointPosition 
* `geometry_msgs/Pose` Pose
  (Note: This is relative to the **world** frame)
* `float64` Speed
* `bool` AddWaypontOnly
* `bool` AddAsJointSpace
* `bool` ClearAllWaypoints
* `bool` ClearAllPlans
* `bool` ExcludeGripper

Response arguments 
* `bool` PlanSuccess
  (True if a path is successfully planned. False otherwise.)
* `int64` PlanID
  (A unique ID for a plan. This will continuously count upwards every time a path is planned.)
* `int64` PlansInQueue
  (How many plans are in the queue that are ready to execute.)
* `int64` WaypointsInQueue
  (How many waypoints are in the queue for the planner to plan for.)

#### Usage
For each scenario, set the required arguments before calling the `/ur5_control/plan/queue` service.  As soon as the service is called, a path is immediately planned but ***not*** executed.

When a path is planned, the last known configuration of the robot is used.  Therefore,
* If _no_ paths are in the queue, a path is planned from the robot's _initial position_ to the proposed goal configuration or pose.
* If there _are_ paths in the queue, a path is planned from the robot's _final position in the final plan_, to the proposed configuration or pose.

##### 2.2.1 Planning a joint space path (general planning)
Planned paths in the joint space will result in a non-linear end-effector path.  However, this mode of planning has a very high success rate, so use it if the end-effector trajectory is not important.

Set the `JointPosition` argument to be a vector of doubles, representing the proposed configuration of the arm in radians.

The `Speed` argument sets the scaled speed of each actuator.  If it is not set (or set to 0), then the default speed is 0.1 (10%) of maximum speed.

All other arguments are ignored, including `Pose`.

##### 2.2.2 Plan by defining an end-efector pose
Ensure that the `JointPosition` argument is empty.

Set the `Pose` argument to be the desired position of the end-effector, and also set `AddAsJointSpace` to `true`. This will calculate the desired joint position based on the given pose via inverse kinematics (IK), and then a joint space path is calculated.

##### 2.2.3 Planning a task space path (straight-line)
Straight line paths are important for grasping trajectories.  

To plan a straight-line path, ensure that the `JointPosition` argument is empty.  Then set the `Pose` argument to be the desired position of the end-effector.  This will add a goal waypoint to the queue, then immediately plan a linear path towards it.

The `Speed` argument sets the *jump threshold*, which detects significant jumps in the arm's joint positions.  If it is not set (or set to 0), then the default is 3.0, which is relatively safe.  Higher numbers allow greater chance for path planning success, but may result in some unsafe trajectories.  There is no optimum number at this point.
##### 2.2.4 Add waypoint 
Add a waypoint if a task space path requires multiple linear path segments.  Set `AddWaypointOnly` to `true`, and the given pose will be added as a waypoint.  In this scenario, _planning does not occur immediately after the service call_.  To plan a path with multiple waypoints, queue a task-space path with `AddWaypointOnly`  set to `false`.  This will plan a path immediately, processing all queued waypoints.

This mode of path planning has a low success rate.  It is recommended to do single-waypoint task space path planning for now.
##### 2.2.5 Gripper collisions
If a gripper collision object is attached, set the `ExcludeGripper` argument to be `true` to exclude it from path planning.

##### 2.2.6 Clearing plans and waypoints
Set either `ClearAllWaypoints` or `ClearAllPlans` to `true` to clear the planner of waypoints and plans.  Unfortunately, only one can be cleared at a time per call.

---
### 2.3 Executing  Path Plans
#### Properties
Service: `/ur5_control/plan/execute`  
Type: `ur5_control/ServiceInt`  
Request arguments
* `int64` Data

Response arguments 
* `int64` Response

#### Usage
To execute all plans in the queue, call the `/ur5_control/plan/execute` service with any arbitrary value set for `Data`.  If execution was successful, the service will respond with the number of plans executed in the `Response` argument.

## Adding Collision Objects
To add your own collision objects or modify existing ones, please go to the`CollHandler.cpp` source code, and modify the `bool AddBaseObjects()` method (approx Line 116).

## Troubleshooting
#### The following error occurs when launching the planner: `Group 'manipulator_i5' was not found.`
This means that a different robot was loaded, rather than the aubo_i5.  If this error occurs,
1. Find the planning group name in the RViz window, located in *Planning Request > Planning Group*.
2. Open `Planner.cpp`, and navigate to Line 401.
3. Replace the `PLANNING_GROUP` string with the Planning Group name obtained from RViz.
4. Run `catkin_make` to recompile the code.

#### Any further issues?
Contact Wesley Au (wesley.au@monash.edu) or Hugh Zhou (hugh.zhou@monash.edu)


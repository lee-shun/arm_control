#!/usr/bin/env python
from __future__ import print_function
import rospy,sys,math
#import moveit_commander
#from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from copy import deepcopy
import numpy as np
import actionlib
from ur5_control.msg import detectionAction,detectionGoal,detectionResult
from ur5_control.srv import GetPoses,GetPosesRequest,GetPosesResponse
#from msg_class import *
#import tf
import time
#from gripper import gripper

class robot_control:
  def __init__(self):
    rospy.init_node('segment_apple',anonymous=True)
    #################################
    self.client = actionlib.SimpleActionClient('vision_block',detectionAction)
    self.client.wait_for_server()
    self.client_local = actionlib.SimpleActionClient('vision_block_local',detectionAction)
    self.client_local.wait_for_server()
    self.goal=detectionGoal(True)
    #################################
    self.Service = rospy.Service('ur5_control/vision/segment_apple', GetPoses, self.svc_callback_segment_apple)
    rospy.loginfo("Successfully connected to Vision action server.")
    rospy.spin()

  def svc_callback_segment_apple(self,req):
    pos, _ori = self.vision_servo(req.Data)
    n_apples = len(pos)
    rospy.loginfo("Number of apples: %i"%(n_apples))

    pose_list = []

    for i in range(0,n_apples):
      p = Pose()
      p.position.x = pos[i][0]
      p.position.y = pos[i][1]
      p.position.z = pos[i][2]
      p.orientation.w = 1
      pose_list.append(p)
      
    poses = PoseArray()
    poses.poses = pose_list
    response = GetPosesResponse()
    response.Poses = poses
    return response
          
  def vision_servo(self,data):
    #rospy.loginfo("Data: %i"%(data))
    if data == 0:
      self.client.send_goal(self.goal)
      self.client.wait_for_result()
      vision_result=self.client.get_result()
    else:
      self.client_local.send_goal(self.goal)
      self.client_local.wait_for_result()
      vision_result=self.client_local.get_result()
    
    index=len(vision_result.objects.object)
    if index!=0:
      obj_pos,obj_ori=self.vision_result_obtain(vision_result)
      obj_pos,obj_ori=self.vision_result_filter(obj_pos,obj_ori)
    else:
      print('no object available !')
      obj_pos,obj_ori=[],[]
    return obj_pos,obj_ori
      
  def vision_result_obtain(self,vision_result):
    vision_result=vision_result.objects.object
    obj_pos=[]
    obj_ori=[]
    for i in range(len(vision_result)):
      pos=[vision_result[i].x/1000.,vision_result[i].y/1000.,vision_result[i].z/1000.]
      eular=[vision_result[i].sita,vision_result[i].phi,0]
      obj_pos.append(pos)
      obj_ori.append(eular)
    return obj_pos,obj_ori
      
  def vision_result_filter(self,obj_pos,obj_ori):
    ##sort the object list based on the shortest depth
    obj_pos_new=[]
    obj_ori_new=[]
    index=obj_pos.index(min(obj_pos))
    while len(obj_pos)>0:
      target=obj_pos[index]
      obj_pos_new.append(obj_pos[index])
      obj_ori_new.append(obj_ori[index])
      obj_pos.pop(index)
      obj_ori.pop(index)
      if len(obj_pos)>0:
        new_pos=np.array(obj_pos)
        dis=np.sum((new_pos-target)**2,axis=1)**0.5
        index=np.argmin(dis)
    return obj_pos_new,obj_ori_new

      
  def vision_matching(self,pos,pos_new,eular,eular_new):
    pos_array=np.array(pos)
    for i in range(len(pos_new)):
      dis=(np.sum((pos_array-np.array(pos_new[i]))**2,axis=1))**0.5
      if dis.min()>0.1:
        pos.append(pos_new[i])
        eular.append(eular_new[i])
        print('\n')
        print('********new target added***************')
        print('\n')
    return pos,eular
  
  
  
    
        
if __name__=='__main__':
  robot_control()
    
    
    
    
    
    
    
    
    

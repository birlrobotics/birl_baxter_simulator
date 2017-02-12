#!/usr/bin/env python
"""
service client module. create a class 
which contains a variety of service client fucntions
"""

from arm_move import pick_and_place
from birl_sim_examples.srv import *

import sys
import rospy
import copy

from gazebo_msgs.srv import (
    DeleteModel
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Empty,
)

  
def add_gazebo_model_client(_model_name,
                            _model_pose=Pose(position=Point(x=0.6, y=0, z=-0.115),
                                            orientation=Quaternion(x=0,y=0,z=0,w=1)),
                            _model_reference_frame="base"):
    rospy.wait_for_service('add_gazebo_box_model')
    try:
        add_gazebo_box_model = rospy.ServiceProxy('add_gazebo_box_model',
                                                  Add_Gazebo_Model)
        req = Add_Gazebo_ModelRequest()
        req.model_name.data = _model_name
        req.model_pose = _model_pose
        req.model_reference_frame.data = _model_reference_frame
        resp = add_gazebo_box_model(req)
        rospy.loginfo("loading box succuessfuly")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
def go_to_start_position_client():
    rospy.wait_for_service('go_to_start_position')
    try:
        go_to_start_position_proxy = rospy.ServiceProxy('go_to_start_position',
                                                  Go_To_Start_Position)
        req = Go_To_Start_PositionRequest()
        req.start.data = True
        resp = go_to_start_position_proxy(req)
        rospy.loginfo("go to start position succesfully!")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def go_to_position_client(pose):
    rospy.wait_for_service('go_to_position')
    try:
        go_to_position_proxy = rospy.ServiceProxy('go_to_position',
                                                  Go_To_Position)
        req = Go_To_PositionRequest()
        req.pose = copy.deepcopy(pose)
        resp = go_to_position_proxy(req)
        rospy.loginfo("go to desired position succesfully!")
        return (resp.ik_flag.data, resp.action_flag.data)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def gripper_move_client(is_move_close):
    rospy.wait_for_service('gripper_move')
    try:
        gripper_move_proxy = rospy.ServiceProxy('gripper_move',
                                                Gripper_Move)
        req = Gripper_MoveRequest()
        req.gripper_desired_flag.data = is_move_close
        resp = gripper_move_proxy(req)
        rospy.loginfo("go to desired position succesfully!")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def shutdown():
    rospy.loginfo("Stopping the node...")
    pick_and_place.delete_gazebo_models()
    
def delete_gazebo_models(model_name):
    '''
    this is the function call the gazebo delete service
    '''
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model(model_name)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

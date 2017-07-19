#!/usr/bin/env python
"""
pick and place service server
"""

from arm_move import pick_and_place
from birl_sim_examples.srv import *

import sys
import rospy
import copy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Empty,
)

import ipdb

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
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    #ipdb.set_trace()
    rospy.init_node("pick_n_place_client")
    rospy.wait_for_message("/robot/sim/started", Empty)
    #ipdb.set_trace()
    add_gazebo_model_client(_model_name="box_male",
                            _model_pose=Pose(position=Point(x=0.6, y=0, z=-0.115),
                                            orientation=Quaternion(x=0,y=0,z=0,w=1)),
                            _model_reference_frame="base")
    add_gazebo_model_client(_model_name="box_female",
                            _model_pose=Pose(position=Point(x=0.6, y=0.4, z=-0.115),
                                            orientation=Quaternion(x=0,y=0,z=0,w=1)),
                            _model_reference_frame="base")

    go_to_start_position_client()


    hover_distance = 0.15
    
    pick_object_pose = Pose()
    place_object_pose = Pose()
    #the position of female box
    pick_object_pose.position.x = 0.6 
    pick_object_pose.position.y = 0 
    pick_object_pose.position.z = -0.115 - 0.005
    
    place_object_pose.position.x = 0.6 
    place_object_pose.position.y = -0.2 
    place_object_pose.position.z = -0.115 - 0.005
    #RPY = 0 pi 0
    pick_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
    
    place_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
    _pose = copy.deepcopy(pick_object_pose)
    
    _pose.position.z = pick_object_pose.position.z + hover_distance
    go_to_position_client(pose = _pose)

    _pose.position.z = pick_object_pose.position.z    
    go_to_position_client(pose = _pose)

    _pose.position.z = pick_object_pose.position.z + hover_distance
    go_to_position_client(pose = _pose)

    
    _pose = copy.deepcopy(place_object_pose)
    _pose.position.z = place_object_pose.position.z + hover_distance
    go_to_position_client(pose = _pose)

    _pose.position.z = place_object_pose.position.z    
    go_to_position_client(pose = _pose)

    _pose.position.z = place_object_pose.position.z + hover_distance
    go_to_position_client(pose = _pose) 
    
if __name__ == '__main__':
    sys.exit(main())



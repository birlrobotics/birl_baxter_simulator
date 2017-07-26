#!/usr/bin/env python
"""
pick and place service server
"""

from arm_move import pick_and_place
from birl_sim_examples.srv import *

import sys
import rospy
import copy

from arm_move.srv_client import * 

import ipdb

import random



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
    rospy.init_node("pick_n_place_client")
    rospy.on_shutdown(shutdown)
    rospy.wait_for_message("/robot/sim/started", Empty)
    ipdb.set_trace()
    #add_gazebo_model_client(_model_name="box_male",
    #                        _model_pose=Pose(position=Point(x=0.7, y=0.4, z=-0.115),
    #                                        orientation=Quaternion(x=0,y=0,z=0,w=1)),
    #                        _model_reference_frame="base")
    #add_gazebo_model_client(_model_name="box_female",
    #                        _model_pose=Pose(position=Point(x=0.5, y=-0.4, z=-0.115),
    #                                        orientation=Quaternion(x=0,y=0,z=0,w=1)),
    #                        _model_reference_frame="base")


    while not rospy.is_shutdown():
        go_to_start_position_client()

        hover_distance = 0.15

        pick_object_pose = Pose()
        place_object_pose = Pose()

        #the position of female box
        ik_flag = False
        while not ik_flag:
            pick_object_pose.position.x = float(random.randint(50,70))/100.0 
            pick_object_pose.position.y = float(random.randint(-40,40))/100.0 
            pick_object_pose.position.z = -0.115 - 0.005
                     #RPY = 0 pi 0
            pick_object_pose.orientation = Quaternion(
                x=0.0,
                y=1.0,
                z=0.0,
                w=6.123233995736766e-17)
            ik_flag = pick_and_place.ik_request_check(pick_object_pose,'right')

        ik_flag = False
        while not ik_flag:           
            place_object_pose.position.x = float(random.randint(50,70))/100.0 
            place_object_pose.position.y = float(random.randint(-40,40))/100.0 
            place_object_pose.position.z = -0.115 - 0.005
            place_object_pose.orientation = Quaternion(
                x=0.0,
                y=1.0,
                z=0.0,
                w=6.123233995736766e-17)
            ik_flag = pick_and_place.ik_request_check(place_object_pose,'right')
             
        print "random pick_object_pose(x,y):(%f,%f)" %(pick_object_pose.position.x,
                                                       pick_object_pose.position.y)
        print "random place_object_pose(x,y):(%f,%f)" %(place_object_pose.position.x,
                                                        place_object_pose.position.y)

         
         #RPY = 0 pi 0
        add_gazebo_model_client(_model_name="box_female",
                                _model_pose=Pose(position=Point(x=pick_object_pose.position.x, y=pick_object_pose.position.y, z=-0.115),
                                                 orientation=Quaternion(x=0,y=0,z=0,w=1)),
                                _model_reference_frame="base")

        _pose = copy.deepcopy(pick_object_pose)

        _pose.position.z = pick_object_pose.position.z + hover_distance
        go_to_position_client(pose = _pose)

        _pose.position.z = pick_object_pose.position.z    
        go_to_position_client(pose = _pose)

        gripper_move_client(is_move_close = True)

        _pose.position.z = pick_object_pose.position.z + hover_distance
        go_to_position_client(pose = _pose)

        
        _pose = copy.deepcopy(place_object_pose)
        _pose.position.z = place_object_pose.position.z + hover_distance
        go_to_position_client(pose = _pose)

        _pose.position.z = place_object_pose.position.z    
        go_to_position_client(pose = _pose)

        gripper_move_client(is_move_close = False)

        _pose.position.z = place_object_pose.position.z + hover_distance
        go_to_position_client(pose = _pose)

        delete_gazebo_models(model_name="box_female")

if __name__ == '__main__':
    sys.exit(main())



#!/usr/bin/env python
"""
pick and place service server
"""

from arm_move import pick_and_place

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

from birl_sim_examples.srv import *

import ipdb


def add_gz_model_handle(req):
    """
    The handle for pick and place server when getting the request from 
    service add_gazebo_box_model

    Add_Gazebo_Model.srv:
    # name of model box_female/box_male
    std_msgs/String model_name

    # Pose of the model
    geometry_msgs/Pose model_pose

    # reference frame
    std_msgs/String model_reference_frame

    ---

    # status of loading
    std_msgs/Bool load_status

    """
    _model_name = req.model_name.data
    _model_pose = req.model_pose
    resp=Add_Gazebo_ModelResponse()
    resp.load_status.data = pick_and_place.load_gazebo_models(model_name=_model_name,
                                                              model_pose=_model_pose)
    return resp

def go_to_start_position_handle(req):
    """
    The handle for moving the arm to the start position
    The parameter of starting_joint_angles is set in main() function

    Go_To_Start_Position.srv:
    #Start Flag
    std_msgs/Bool start
    ---
    # finish Flag
    std_msgs/Bool finish

    """
    flag = pnp._guarded_move_to_joint_position(starting_joint_angles)
    pnp.gripper_open()
    rospy.sleep(1.0)
    resp = Go_To_Start_PositionResponse()
    resp.finish.data = flag
    return resp

def go_to_position_handle(req):
    """
    The handle for arm moving to desired position.

    Go_To_Position.srv:
    # go to desired position
    geometry_msgs/Pose pose

    ---

    # IK service result
    std_msgs/Bool ik_flag

    # action result flag
    std_msgs/Bool action_flag
   
    """
    _pose = copy.deepcopy(req.pose)
    joint_angles = pnp.ik_request(_pose)
    resp = Go_To_PositionResponse()
    if joint_angles:
        resp.ik_flag.data = True
        action_flag = pnp._guarded_move_to_joint_position(joint_angles)
    else:
        resp.ik_flag.data = False
        
    resp.action_flag.data = action_flag
    return resp

def gripper_move_handle(req):
    """
    The handle for gripper move Close/Open

    # gripper desired position:  Close->True  Open->False
    std_msgs/Bool gripper_desired_flag

    ---

    # gripper status Close->True  Open->False

    std_msgs/Bool gripper_status_flag

    """
    _gripper_desired_flag = req.gripper_desired_flag.data
    resp = Gripper_MoveResponse()
    if _gripper_desired_flag:
        pnp.gripper_close()
        resp.gripper_status_flag.data = True
    else:
        pnp.gripper_open()
        resp.gripper_status_flag.data = False

    return resp

def pick_and_place_servers():
    #initiating the node
    rospy.init_node('pick_and_place_server')
    #wait for gazebo environment ready!
    rospy.wait_for_message("/robot/sim/started", Empty)
    #create the service servers:
    add_gz_model_server = rospy.Service('add_gazebo_box_model', Add_Gazebo_Model,
                                        add_gz_model_handle)
    rospy.loginfo("starting the add_gazebo_box_model service finished!")
    
    go_to_start_position = rospy.Service('go_to_start_position', Go_To_Start_Position,
                                        go_to_start_position_handle)
    
    rospy.loginfo("starting the go_to_start_position service finished!")

    go_to_position = rospy.Service('go_to_position', Go_To_Position,
                                   go_to_position_handle)
    
    rospy.loginfo("starting the go_to_position service finished!")
    
    gripper_move = rospy.Service('gripper_move', Gripper_Move,
                                 gripper_move_handle)
    
    rospy.loginfo("starting the gripper_move service finished!")    
def main():
    #ipdb.set_trace()
    # initiating the node and service
    pick_and_place_servers()
    # choose the move arm
    limb = 'right'
    # pick and place hover distance
    hover_distance = 0.15 # meters
    # Starting Joint angles for arm, initial state
    global starting_joint_angles
    starting_joint_angles = {'right_w0': -0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': 0.4999997247485215,
                             'right_e0': -0.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': 0.08000397926829805,
                             'right_s1': -0.9999781166910306}
    #initiating the PickAndPlace class
    global pnp
    pnp = pick_and_place.PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    
    block_poses = list()

    #object position with a tranform offset between gripper and male part.
    object_pose = Pose()

    #the position of female box
    object_pose.position.x = 0.6 
    object_pose.position.y = 0 
    object_pose.position.z = -0.115 - 0.005

    #RPY = 0 pi 0
    object_orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
    
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=object_pose.position.x, y=object_pose.position.y, z=object_pose.position.z),
        orientation=object_orientation))
    
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.6, y=-0.2, z=-0.115-0.005),
        orientation=object_orientation))

    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())


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
                                             orientation=Quaternion(x=0, y=0, z=0, w=1)),
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
        rospy.loginfo("loading boxs succuessfuly")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def traj_move_start_pose_client():
    rospy.wait_for_service('traj_move_start_pose')
    try:
        traj_move_start_pose_proxy = rospy.ServiceProxy('traj_move_start_pose',
                                                        Traj_Move_Start_Pose)
        req = Traj_Move_Start_PoseRequest()
        req.start.data = True
        req.time = 6.0
        resp = traj_move_start_pose_proxy(req)
        rospy.loginfo("traj_move_start_pose succesfully!")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def traj_move_pose_client(pose):
    rospy.wait_for_service('traj_move_pose')
    try:
        traj_move_pose_proxy = rospy.ServiceProxy('traj_move_pose',
                                                  Traj_Move_Pose)
        req = Traj_Move_PoseRequest()
        req.dest_pose = copy.deepcopy(pose)
        req.time = 6.0
        resp = traj_move_pose_proxy(req)
        rospy.loginfo("go to desired position succesfully!")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def gripper_move_client(flag):
    rospy.wait_for_service('gripper_move')
    try:
        gripper_move_proxy = rospy.ServiceProxy('gripper_move',
                                                Gripper_Move)
        req = Gripper_MoveRequest()
        req.gripper_desired_flag.data = flag
        resp = gripper_move_proxy(req)
        if flag:
            rospy.loginfo("Gripper close succesfully!")
        else:
            rospy.loginfo("Gripper open succesfully!")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def main():

    rospy.init_node("pick_n_place_joint_trajectory_srv_client")
    rospy.wait_for_message("/robot/sim/started", Empty)

    # Add the BOX in Gazebo
    add_gazebo_model_client(_model_name="box_male",
                            _model_pose=Pose(position=Point(x=0.6, y=0, z=-0.115),
                                             orientation=Quaternion(x=0, y=0, z=0, w=1)),
                            _model_reference_frame="base")
    add_gazebo_model_client(_model_name="box_female",
                            _model_pose=Pose(position=Point(x=0.6, y=0.4, z=-0.115),
                                             orientation=Quaternion(x=0, y=0, z=0, w=1)),
                            _model_reference_frame="base")

    # Move to Start Pose
    traj_move_start_pose_client()

    #########################
    #pick and place pose parameter
    hover_distance = 0.15
    pick_object_pose = Pose()
    place_object_pose = Pose()
    # the position of female box
    pick_object_pose.position.x = 0.6
    pick_object_pose.position.y = 0
    pick_object_pose.position.z = -0.115 - 0.005

    place_object_pose.position.x = 0.6
    place_object_pose.position.y = -0.2
    place_object_pose.position.z = -0.115 - 0.005
    # RPY = 0 pi 0
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
    ##################################


    gripper_move_client(False)

    _pose.position.z = pick_object_pose.position.z + hover_distance
    traj_move_pose_client(pose=_pose)

    _pose.position.z = pick_object_pose.position.z
    traj_move_pose_client(pose=_pose)

    gripper_move_client(True)

    _pose.position.z = pick_object_pose.position.z + hover_distance
    traj_move_pose_client(pose=_pose)

    _pose = copy.deepcopy(place_object_pose)
    _pose.position.z = place_object_pose.position.z + hover_distance
    traj_move_pose_client(pose=_pose)


    _pose.position.z = place_object_pose.position.z
    traj_move_pose_client(pose=_pose)

    gripper_move_client(False)

    _pose.position.z = place_object_pose.position.z + hover_distance
    traj_move_pose_client(pose=_pose)





if __name__ == '__main__':
    sys.exit(main())



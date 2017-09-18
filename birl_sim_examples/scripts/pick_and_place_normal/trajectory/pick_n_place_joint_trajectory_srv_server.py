#!/usr/bin/env python
"""
pick and place joint trajectory service server, move arm to destination

author: ben

date: aug 16, 2017
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
from baxter_core_msgs.msg import EndpointState
from birl_sim_examples.msg import Tag_EndpointPose

import ipdb
import baxter_interface

from arm_move.srv_client import *
from arm_move import srv_action_client

import hardcoded_data



def traj_move_start_pose_handle(req):


    rospy.loginfo('executing Trajectory move to start Pose...')
    global limb
    global traj
    global limb_interface

    _time = req.time
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]

    starting_joint_angles = hardcoded_data.starting_joint_angles

    starting_joint_order_angles = [starting_joint_angles[joint] for joint in hardcoded_data.limb_names]
    traj.clear('right')
    traj.add_point(current_angles, 0.0)
    traj.add_point(starting_joint_order_angles, _time)
    traj.start()
    traj.wait(_time)

    resp = Traj_Move_Start_PoseResponse()
    resp.finish.data = True
    return resp


def traj_move_pose_handle(req):

    rospy.loginfo('executing Trajectory move from current pose to destination Pose..')
    global limb
    global traj
    global limb_interface

    _time = req.time
    _dest_pose = req.dest_pose
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.clear('right')
    traj.add_point(current_angles, 0.0)
    traj.add_pose_point(_dest_pose, _time)
    traj.start()
    traj.wait(_time)

    resp = Traj_Move_PoseResponse()
    resp.finish.data = True
    return resp



def gripper_move_handle(req):

    global traj

    _gripper_desired_flag = req.gripper_desired_flag.data
    resp = Gripper_MoveResponse()
    if _gripper_desired_flag:
        traj.gripper_close()
        resp.gripper_status_flag.data = True
    else:
        traj.gripper_open()
        resp.gripper_status_flag.data = False

    return resp


def add_gz_model_handle(req):

    _model_name = req.model_name.data
    _model_pose = req.model_pose
    resp = Add_Gazebo_ModelResponse()
    resp.load_status.data = pick_and_place.load_gazebo_models(model_name=_model_name,
                                                              model_pose=_model_pose)
    return resp


def pick_and_place_servers():
    # initiating the node
    rospy.init_node('pick_and_place_joint_trajectory_srv_server')
    # wait for gazebo environment ready!
    # rospy.wait_for_message("/robot/sim/started", Empty)
    # create the service servers:
    add_gz_model_server = rospy.Service('add_gazebo_box_model', Add_Gazebo_Model,
                                        add_gz_model_handle)
    # rospy.loginfo("starting the add_gazebo_box_model service finished!")

    traj_move_start_pose = rospy.Service('traj_move_start_pose', Traj_Move_Start_Pose,
                                         traj_move_start_pose_handle)

    # rospy.loginfo("starting the go_to_start_position service finished!")

    traj_move_pose = rospy.Service('traj_move_pose', Traj_Move_Pose,
                                    traj_move_pose_handle)

    # rospy.loginfo("starting the go_to_position service finished!")

    gripper_move = rospy.Service('gripper_move', Gripper_Move,
                                  gripper_move_handle)



    # rospy.loginfo("starting the gripper_move service finished!")


def main():
    global traj
    global limb_interface
    global limb

    # init the node and set up the services
    pick_and_place_servers()

    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)





    rospy.loginfo("All serivices and topics have been set up!")
    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())



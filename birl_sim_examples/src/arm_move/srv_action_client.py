#!/usr/bin/env python
"""
service client module. create a class 
which contains a variety of service client fucntions
"""



import sys
import rospy
import rospkg
import copy
import struct

import actionlib

from arm_move import pick_and_place
from birl_sim_examples.srv import *

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface

class Trajectory(object):
    def __init__(self, limb,verbose = False):
        # set up the action client
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance

        ## check up if the joint_trajectory action server setup
        ## please rosrun baxter_interface joint_trajectory_server
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

        #clear all  JointTrajectoryPoint
        self.clear(limb)
        self._verbose = verbose
        # enable the IK Service
        ik_ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_ns, SolvePositionIK)
        rospy.wait_for_service(ik_ns, 5.0)
        
        self._gripper = baxter_interface.Gripper(limb)
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()


    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        return self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def add_pose_point(self,pose,time):
        angles = self.ik_request(pose)
        if not angles:
            return 0
        else:
            self.add_point(angles,time)
            return 1

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
  
    def ik_request(self,pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

        limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        limb_angles = [limb_joints[joint] for joint in limb_names]
        return limb_angles

    
def delete_gazebo_models():
# This will be called on ROS Exit, deleting Gazebo models
# Do not wait for the Gazebo Delete Model service, since
# Gazebo should already be running. If the service is not
# available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("box_male")
        resp_delete = delete_model("box_female")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def load_gazebo_models(model_name,
                       model_pose=Pose(position=Point(x=0.6, y=0, z=-0.115),
                                      orientation=Quaternion(x=0,y=0,z=0,w=1)),
                       model_reference_frame="base"):
    if model_name == "box_male":
    # Get male box Path
        model_path = rospkg.RosPack().get_path('birl_baxter_description')+"/urdf/box/"
        # Load male box SDF
        box_male_xml = ''
        with open (model_path + "box_male/robots/box_male.URDF", "r") as box_male_file:
            box_male_xml=box_male_file.read().replace('\n', '')
            rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("box_male", box_male_xml, "/",
                                   model_pose, model_reference_frame)
            rospy.loginfo("loading male box succesfully")
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))
            return False


    if model_name == "box_female":
        # get path
        model_path = rospkg.RosPack().get_path('birl_baxter_description')+"/urdf/box/"
        # Load female box  URDF
        box_female_xml = ''
        with open (model_path + "box_female/robots/box_female.URDF", "r") as box_female_file:
            box_female_xml=box_female_file.read().replace('\n', '')
            rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf2 = spawn_urdf2("box_female", box_female_xml, "/",
                                     model_pose, model_reference_frame)
            rospy.loginfo("loading female box succesfully")
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))
            return False
    return True

def hmm_state_switch_client(state):
    rospy.wait_for_service('hmm_state_switch')
    try:
        hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch',
                                                    State_Switch)
        req = State_SwitchRequest()
        req.state = state
        resp = hmm_state_switch_proxy(req)
        if resp.finish.data:
            print "Hmm State switch to %d succesfully" %state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

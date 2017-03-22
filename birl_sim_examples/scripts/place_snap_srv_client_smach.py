#!/usr/bin/env python
"""
pick and place service smach server
"""

from arm_move import pick_and_place
from birl_sim_examples.srv import *

import sys
import rospy
import copy

from arm_move.srv_client import *

import smach
import smach_ros

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import ipdb

import random

import baxter_external_devices

class Go_to_Start_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','Time_Out'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go to Start position...')
        go_to_start_position_client()
        return 'Succeed'
        
class Setting_Start_and_End_Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             output_keys=['pick_object_pose', 'place_object_pose'])
    def execute(self, userdata):
        rospy.loginfo('executing Setting_Start_and_End_Pose... ')
        self.pick_object_pose = Pose()
        self.place_object_pose = Pose()
        
        self.pick_object_pose.position.x = 0.6
        self.pick_object_pose.position.y = -0.4
        self.pick_object_pose.position.z = -0.115 - 0.005
        #RPY = 0 pi 0
        self.pick_object_pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0)
      
        self.place_object_pose.position.x = 0.6
        self.place_object_pose.position.y = -0.2
        self.place_object_pose.position.z = -0.115 - 0.005
        self.place_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)

        userdata.pick_object_pose = copy.deepcopy(self.pick_object_pose)
        userdata.place_object_pose = copy.deepcopy(self.place_object_pose)
        
        return 'Succeed'


class Go_to_Pick_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['pick_object_pose','hover_distance'])
        
    def execute(self, userdata):
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
       
        pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance
        (ik_flag, action_flag) = go_to_position_client(pose = pick_object_pose)
        if not ik_flag:
            return 'IK_Fail'
        elif not action_flag:
            return 'Time_Out'
        else:
            return 'Succeed'

class Go_to_Desired_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['move_pose'])
    def execute(self, userdata):
        move_pose = copy.deepcopy(userdata.move_pose)
        (ik_flag, action_flag) = go_to_position_client(pose = move_pose)
        if not ik_flag:
            return 'IK_Fail'
        elif not action_flag:
            return 'Time_Out'
        else:
            return 'Succeed'
class Keyboard_Control(smach.State):
    def __init__(self): 
        smach.State.__init__(self,
                             input_keys=['pick_object_pose'],
                             output_keys=['move_pose'],
                             outcomes=['Succeed'])
        self.move_scale = 1
        self._init = True
        self.move_pose = Pose()
        
    def execute(self, userdata):
        if self._init:
            self.move_pose = copy.deepcopy(userdata.pick_object_pose)
            self._init = False
            
        def set_pose(direction,delta):
            move_pose = copy.deepcopy(self.move_pose)
            move_scale = self.move_scale
            if direction == 'f':
                move_scale = move_scale + 1
                if move_scale == 11:
                    move_scale = 10
                print move_scale
            elif direction == 'b':
                move_scale = move_scale -1
                if move_scale == 0:
                    move_scale = 1
                print move_scale
            elif direction == 'x':
                move_pose.position.x = move_pose.position.x + delta*move_scale
            elif direction == 'y':
                move_pose.position.y = move_pose.position.y + delta*move_scale
            elif direction == 'z':
                move_pose.position.z = move_pose.position.z + delta*move_scale
            self.move_pose = copy.deepcopy(move_pose)
            userdata.move_pose = copy.deepcopy(move_pose)
            self.move_scale = move_scale
        bindings = {
            '2': (set_pose, ['x',-0.01], "x decrease 0.01!"),
            '8': (set_pose, ['x',0.01], "x increase 0.01!"),
            '4': (set_pose, ['y',-0.01], "y decrease 0.01!"),
            '6': (set_pose, ['y',0.01], "y increase 0.01!"),
            's': (set_pose, ['z',-0.01], "z decrease 0.01!"),
            'w': (set_pose, ['z',0.01], "z increase 0.01!"),
            '1': (set_pose, ['f',1], "Multiply scale * increase 1!"),
            '3': (set_pose, ['b',1], "Multiply scale * decrease 1!")}
        done = False
        while not done and not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:
                if c in bindings:
                    cmd = bindings[c]
                    cmd[0](*cmd[1])
                    print("command: %s" % cmd[2])
                    done = True
        return 'Succeed'

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
 
    sm = smach.StateMachine(outcomes=['Done'])

    sm.userdata.sm_pick_object_pose = Pose()
    sm.userdata.sm_place_object_pose = Pose()
    sm.userdata.sm_hover_distance = 0.15
    sm.userdata.sm_move_pose = Pose()

    with sm:
        smach.StateMachine.add('Go_to_Start_Position',Go_to_Start_Position(),
                               transitions={'Succeed':'Setting_Start_and_End_Pose',
                                            'Time_Out':'Setting_Start_and_End_Pose'})
        smach.StateMachine.add('Setting_Start_and_End_Pose',Setting_Start_and_End_Pose(),
                               transitions={'Succeed':'Go_to_Pick_Hover_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                               'place_object_pose':'sm_place_object_pose'})
                               
        smach.StateMachine.add('Go_to_Pick_Hover_Position',Go_to_Pick_Hover_Position(),
                               transitions={'Succeed':'Keyboard_Control',
                                            'IK_Fail':'Go_to_Pick_Hover_Position',
                                            'Time_Out':'Go_to_Pick_Hover_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance',
                                          'current_pose':'sm_current_pose'})
        smach.StateMachine.add('Keyboard_Control',Keyboard_Control(),
                               transitions={'Succeed':'Go_to_Desired_Position'},
                               remapping={'move_pose':'sm_move_pose',
                                          'pick_object_pose':'sm_pick_object_pose'})
        smach.StateMachine.add('Go_to_Desired_Position',Go_to_Desired_Position(),
                               transitions={'Succeed':'Keyboard_Control',
                                            'IK_Fail':'Go_to_Pick_Hover_Position',
                                            'Time_Out':'Go_to_Pick_Hover_Position'},
                               remapping={'move_pose':'sm_move_pose'})
  
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())

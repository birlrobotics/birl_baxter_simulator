#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun baxter_interface Joint_trajectory_server first!
"""

import baxter_interface
from birl_sim_examples.srv import *

from birl_sim_examples.msg import (
    Tag_MultiModal,
    Hmm_Log
)

import sys
import rospy
import copy

from arm_move.srv_client import *
from arm_move import srv_action_client

import smach
import smach_ros


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import time 

#import ipdb

event_flag = 1

class Go_to_Start_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go to Start position...')
        global limb
        global traj
        global limb_interface
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        starting_joint_angles = {'right_w0': -0.6699952259595108,
                                 'right_w1': 1.030009435085784,
                                 'right_w2': 0.4999997247485215,
                                 'right_e0': -0.189968899785275,
                                 'right_e1': 1.9400238130755056,
                                 'right_s0': 0.08000397926829805,
                                 'right_s1': -0.9999781166910306}
        limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        starting_joint_order_angles = [starting_joint_angles[joint] for joint in limb_names]
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_point(starting_joint_order_angles, 6.0)
        traj.start()
        traj.wait(6.0)
        traj.gripper_open()
        return 'Succeed'
        
class Setting_Start_and_End_Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             output_keys=['pick_object_pose', 'place_object_pose'])
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        rospy.loginfo('executing Setting_Start_and_End_Pose... ')
        self.pick_object_pose = Pose()
        self.place_object_pose = Pose()
        
        self.pick_object_pose.position.x = 0.783433342576
        self.pick_object_pose.position.y = -0.281027705287
        self.pick_object_pose.position.z = -0.0395903973417



        #RPY = 0 pi 0
        self.pick_object_pose.orientation = Quaternion(
            x= -0.0634582357249,
            y= 0.997906913323,
            z= 0.0122551630271,
            w= -0.00215769313191)

        self.place_object_pose = copy.deepcopy(self.pick_object_pose)
      
        self.place_object_pose.position.y = self.place_object_pose.position.y + 0.3

        userdata.pick_object_pose = copy.deepcopy(self.pick_object_pose)
        userdata.place_object_pose = copy.deepcopy(self.place_object_pose)
        
        return 'Succeed'

class Go_to_Pick_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             input_keys=['pick_object_pose','hover_distance'])
        self.state = 1
        
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface

        hmm_state_switch_client(self.state)
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        #pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose.position.z = hover_pick_object_pose.position.z + userdata.hover_distance
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_pick_object_pose, 4.0)
        traj.start()
        traj.wait(4.0)
        traj.gripper_open()
        #rospy.sleep(1)
        return 'Succeed'
    
class Go_to_Pick_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','retry'],
                             input_keys=['pick_object_pose','hover_distance'])
        self.state = 2
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface

        hmm_state_switch_client(self.state)
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose = copy.deepcopy(userdata.pick_object_pose)

        traj.clear('right')
        
        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance
        traj.add_pose_point(hover_pick_object_pose, 0.0)
        
        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance*3/4
        traj.add_pose_point(hover_pick_object_pose, 1.0)
        
        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance*2/4
        traj.add_pose_point(hover_pick_object_pose, 2.0)
        
        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance*1/4
        traj.add_pose_point(hover_pick_object_pose, 3.0)
    

        traj.add_pose_point(pick_object_pose, 4.0)
        traj.start()
        traj.wait(4.0)
        traj.gripper_close()

        
        
        traj.clear('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance*1/4
        traj.add_pose_point(hover_pick_object_pose, 1.0)
        
        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance*2/4
        traj.add_pose_point(hover_pick_object_pose, 2.0)
        
        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance*3/4
        traj.add_pose_point(hover_pick_object_pose, 3.0)

        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance*3/4
        traj.add_pose_point(hover_pick_object_pose, 4.0)
 
        traj.start()

        start_time = time.time()
        duration = 0.0
        while(duration<4.0):
            duration = time.time() - start_time
            if event_flag == 0:
                traj.stop()
                traj.clear('right')
                return 'retry'

        
        
        return 'Succeed'
    
class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        return 'Succeed'

class Go_to_Place_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             input_keys=['place_object_pose','hover_distance'])
        self.state = 3
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface

        
        hmm_state_switch_client(self.state)
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        #place_object_pose = copy.deepcopy(userdata.place_object_pose)
        hover_place_object_pose = copy.deepcopy(userdata.place_object_pose)
        hover_place_object_pose.position.z = hover_place_object_pose.position.z + userdata.hover_distance
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_place_object_pose, 5.0)
        traj.start()
        traj.wait(5.0)
        #rospy.sleep(1)
        return 'Succeed'
    
class Go_to_Place_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             input_keys=['place_object_pose','hover_distance'])
        self.state = 4
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface

        hmm_state_switch_client(self.state)
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        place_object_pose = copy.deepcopy(userdata.place_object_pose)
        hover_place_object_pose = copy.deepcopy(userdata.place_object_pose)

        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        
        hover_place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance*3/4
        traj.add_pose_point(hover_place_object_pose, 1.0)
        
        hover_place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance*2/4
        traj.add_pose_point(hover_place_object_pose, 2.0)
        
        hover_place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance*1/4
        traj.add_pose_point(hover_place_object_pose, 3.0)
    

        traj.add_pose_point(place_object_pose, 4.0)
        traj.start()
        traj.wait(4.0)
        traj.gripper_open()

        
        
        traj.clear('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        hover_place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance*1/4
        traj.add_pose_point(hover_place_object_pose, 1.0)
        
        hover_place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance*2/4
        traj.add_pose_point(hover_place_object_pose, 2.0)
        
        hover_place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance*3/4
        traj.add_pose_point(hover_place_object_pose, 3.0)

        hover_place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance*3/4
        traj.add_pose_point(hover_place_object_pose, 4.0)
 
        traj.start()
        traj.wait(4.0)
        
        return 'Succeed'
    
def callback_hmm(msg):
    global event_flag
    event_flag = msg.event_flag  

        
def shutdown():
    global limb
    global traj
    global lintimb_erface
    rospy.loginfo("Stopping the node...")
    #srv_action_client.delete_gazebo_models()
    traj.clear('right')
    traj.stop()

        
def main():
    rospy.init_node("pick_n_place_joint_trajectory")
    rospy.on_shutdown(shutdown)
    rospy.Subscriber("/hmm_online_result", Hmm_Log, callback_hmm)
    #rospy.wait_for_message("/robot/sim/started", Empty)
    #ipdb.set_trace()
 
    sm = smach.StateMachine(outcomes=['Done'])

    sm.userdata.sm_pick_object_pose = Pose()
    sm.userdata.sm_place_object_pose = Pose()
    sm.userdata.sm_hover_distance = 0.15

    global traj
    global limb_interface
    global limb
    
    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)
    
    with sm:
        smach.StateMachine.add('Go_to_Start_Position',Go_to_Start_Position(),
                               transitions={'Succeed':'Setting_Start_and_End_Pose'})
        smach.StateMachine.add('Setting_Start_and_End_Pose',Setting_Start_and_End_Pose(),
                               transitions={'Succeed':'Go_to_Pick_Hover_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                               'place_object_pose':'sm_place_object_pose'})

        smach.StateMachine.add('Go_to_Pick_Hover_Position',Go_to_Pick_Hover_Position(),
                               transitions={'Succeed':'Go_to_Pick_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Recovery',Recovery(),
                               transitions={'Succeed':'Go_to_Pick_Hover_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})
                               
        smach.StateMachine.add('Go_to_Pick_Position',Go_to_Pick_Position(),
                               transitions={'Succeed':'Go_to_Place_Hover_Position',
                                            'retry':'Recovery'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Go_to_Place_Hover_Position',Go_to_Place_Hover_Position(),
                               transitions={'Succeed':'Go_to_Place_Position'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'hover_distance':'sm_hover_distance'})
                               
        smach.StateMachine.add('Go_to_Place_Position',Go_to_Place_Position(),
                               transitions={'Succeed':'Done'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'hover_distance':'sm_hover_distance'})


  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    hmm_state_switch_client(0)
    sis.start()
    outcome = sm.execute()

    hmm_state_switch_client(0)

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())



#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun baxter_interface Joint_trajectory_server first!
"""

import baxter_interface
from birl_sim_examples.srv import *

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

import ipdb

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
        traj.add_point(starting_joint_order_angles, 5.0)
        traj.start()
        traj.wait(10.0)
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
        
        self.pick_object_pose.position.x = 0.7
        self.pick_object_pose.position.y = -0.4
        self.pick_object_pose.position.z = -0.115
        #RPY = 0 pi 0
        self.pick_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
      
        self.place_object_pose.position.x = 0.7
        self.place_object_pose.position.y = -0.1
        self.place_object_pose.position.z = -0.115
        self.place_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)

        userdata.pick_object_pose = copy.deepcopy(self.pick_object_pose)
        userdata.place_object_pose = copy.deepcopy(self.place_object_pose)
        
        return 'Succeed'

class Add_Box_Gazebo_Model(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             input_keys=['pick_object_pose'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        srv_action_client.delete_gazebo_models()
        srv_action_client.load_gazebo_models(model_name="box_female",
                           model_pose=Pose(position=Point(x=userdata.pick_object_pose.position.x,
                                                          y=userdata.pick_object_pose.position.y,
                                                          z=-0.115),
                                           orientation=Quaternion(x=0,y=0,z=0,w=1)),
                           model_reference_frame="base")
        return 'Succeed'

class Go_to_Pick_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['pick_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose = copy.deepcopy(pick_object_pose)
        hover_pick_object_pose.position.z = hover_pick_object_pose.position.z + userdata.hover_distance
        hover_pick_angles = traj.ik_request(hover_pick_object_pose)
        pick_angles = traj.ik_request(pick_object_pose)
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_point(hover_pick_angles, 5.0)
        traj.add_point(pick_angles, 10.0)
        traj.start()
        traj.wait(15.0)
        traj.gripper_close()
        rospy.sleep(1)
        return 'Succeed'

class Go_to_Place_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['pick_object_pose','place_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance
        place_object_pose = copy.deepcopy(userdata.place_object_pose)
        hover_place_object_pose = copy.deepcopy(place_object_pose)
        hover_place_object_pose.position.z = hover_place_object_pose.position.z + userdata.hover_distance
        hover_pick_angles = traj.ik_request(hover_pick_object_pose)
        hover_place_angles = traj.ik_request(hover_place_object_pose)
        pick_angles = traj.ik_request(pick_object_pose)
        place_angles = traj.ik_request(place_object_pose)
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_point(hover_pick_angles, 5.0)
        traj.add_point(hover_place_angles, 10.0)
        traj.add_point(place_angles, 15.0)
        traj.start()
        traj.wait(20.0)
        traj.gripper_open()
        return 'Succeed'
        
def shutdown():
    global limb
    global traj
    global lintimb_erface
    rospy.loginfo("Stopping the node...")
    srv_action_client.delete_gazebo_models()
    traj.clear('right')
    traj.stop()

        
def main():
    rospy.init_node("pick_n_place_joint_trajectory")
    rospy.on_shutdown(shutdown)
    rospy.wait_for_message("/robot/sim/started", Empty)
    ipdb.set_trace()
 
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
                               transitions={'Succeed':'Add_Box_Gazebo_Model'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                               'place_object_pose':'sm_place_object_pose'})
        smach.StateMachine.add('Add_Box_Gazebo_Model',Add_Box_Gazebo_Model(),
                               transitions={'Succeed':'Go_to_Pick_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose'})
                               
        smach.StateMachine.add('Go_to_Pick_Position',Go_to_Pick_Position(),
                               transitions={'Succeed':'Go_to_Place_Position',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Go_to_Place_Position',Go_to_Place_Position(),
                               transitions={'Succeed':'Done',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())



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

import std_msgs.msg

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import time 

import hardcoded_data


event_flag = 1
execution_history = []



## @brief wait for trajectory goal to be finished, perform preemptive anomaly detection in the meantime. 
## @param trajectory instance 
## @return True if anomaly is detected.
def wait_for_motion_and_detect_anomaly(traj_obj):
    # loop while the motion is not finished
    while not traj.wait(0.00001):
        # anomaly is detected
        if event_flag == 0:
            traj_obj.stop()
            rospy.loginfo("anomaly detected")
            return True

    return False

## @brief record exec history
## @param current_state_name string
## @param current_userdata userdata passed into current state 
## @param depend_on_prev_states True if current state's success depends on previous states 
## @return None
def write_exec_hist(state_instance, current_state_name, current_userdata, depend_on_prev_states):
    import copy
    global execution_history

    saved_userdata = {}
    for k in state_instance._input_keys:
        saved_userdata[k] = copy.deepcopy(current_userdata[k])

    execution_history.append(
        {
            "state_name": current_state_name,
            "saved_userdata": saved_userdata,
            "depend_on_prev_states": depend_on_prev_states
        }
    )

class Go_to_Start_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
    def execute(self, userdata):
        write_exec_hist(self, "Go_to_Start_Position", userdata, False)        

        rospy.loginfo('executing Go to Start position...')
        global limb
        global traj
        global limb_interface
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]

        starting_joint_angles = hardcoded_data.starting_joint_angles

        limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        starting_joint_order_angles = [starting_joint_angles[joint] for joint in limb_names]
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_point(starting_joint_order_angles, 6.0)
        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    
        return 'Succeed'

class Go_to_Pick_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        self.state = 1
        
        
    def execute(self, userdata):
        write_exec_hist(self, "Go_to_Pick_Hover_Position", userdata, False)        

        global limb
        global traj
        global limb_interface

        global mode_no_state_trainsition_report
        if not mode_no_state_trainsition_report:
            hmm_state_switch_client(self.state)
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        hover_pick_object_pose = hardcoded_data.hover_pick_object_pose
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_pick_object_pose, 4.0)
        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    

        #rospy.sleep(1)
        return 'Succeed'
    
class Go_to_Pick_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        self.state = 2
        
    def execute(self, userdata):
        write_exec_hist(self, "Go_to_Pick_Position", userdata, True)        

        global limb
        global traj
        global limb_interface

        global mode_no_state_trainsition_report
        if not mode_no_state_trainsition_report:
            hmm_state_switch_client(self.state)

        
        traj.gripper_open()
        
        # make gripper dive vertically to approach the object
        traj.clear('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        pick_object_pose = hardcoded_data.pick_object_pose
        
        tmp_position = copy.deepcopy(pick_object_pose)
        tmp_position.position.z = pick_object_pose.position.z + hardcoded_data.hover_distance*3/4
        traj.add_pose_point(tmp_position, 1.0)
        
        tmp_position.position.z = pick_object_pose.position.z + hardcoded_data.hover_distance*2/4
        traj.add_pose_point(tmp_position, 2.0)
        
        tmp_position.position.z = pick_object_pose.position.z + hardcoded_data.hover_distance*1/4
        traj.add_pose_point(tmp_position, 3.0)
    
        traj.add_pose_point(pick_object_pose, 4.0)

        rospy.loginfo("Gripper diving...")
        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    

        # grab the object
        traj.gripper_close()

        return 'Succeed'
        
        



    


class Go_to_Place_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        self.state = 3
        
    def execute(self, userdata):
        write_exec_hist(self, "Go_to_Place_Hover_Position", userdata, False)        

        global limb
        global traj
        global limb_interface

        
        global mode_no_state_trainsition_report
        if not mode_no_state_trainsition_report:
            hmm_state_switch_client(self.state)
        
        rospy.loginfo("Gripper lifting...")
        traj.clear('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        hover_pick_object_pose = hardcoded_data.hover_pick_object_pose
        traj.add_pose_point(hover_pick_object_pose, 4.0)
 
        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    


        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        #place_object_pose = copy.deepcopy(userdata.place_object_pose)
        hover_place_object_pose = hardcoded_data.hover_place_object_pose
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_place_object_pose, 5.0)
        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    
        return 'Succeed'
    
class Go_to_Place_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        self.state = 4
        
    def execute(self, userdata):
        write_exec_hist(self, "Go_to_Place_Position", userdata, True)        

        global limb
        global traj
        global limb_interface

        global mode_no_state_trainsition_report
        if not mode_no_state_trainsition_report:
            hmm_state_switch_client(self.state)
        
        traj.clear('right')

        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)
        place_object_pose = hardcoded_data.place_object_pose

        tmp_position = copy.deepcopy(place_object_pose)
        tmp_position.position.z = place_object_pose.position.z + hardcoded_data.hover_distance*3/4
        traj.add_pose_point(tmp_position, 1.0)
        
        tmp_position.position.z = place_object_pose.position.z + hardcoded_data.hover_distance*2/4
        traj.add_pose_point(tmp_position, 2.0)
        
        tmp_position.position.z = place_object_pose.position.z + hardcoded_data.hover_distance*1/4
        traj.add_pose_point(tmp_position, 3.0)
    

        traj.add_pose_point(place_object_pose, 4.0)
        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    
        traj.gripper_open()

        rospy.loginfo("Lifting gripper without a.d.")
        # move the right arm back to hover state... just to keep the amount of state is 4 so we don't add a state for this for now... 
        traj.clear('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)
        hover_place_object_pose = hardcoded_data.hover_place_object_pose
        traj.add_pose_point(hover_place_object_pose, 4.0)
        traj.start()
        traj.wait(4)
        traj.stop()

        return 'Succeed'    
    
class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['RecoveryFailed'])
        
    def execute(self, userdata):
        rospy.loginfo("Enter Recovery State...")
        rospy.loginfo("Hit any key to exit Recovery State...")
        raw_input()
        return 'RecoveryFailed'

def callback_hmm(msg):
    global event_flag
    event_flag = msg.event_flag  

def callback_manual_anomaly_signal(msg):
    global event_flag
    event_flag = 0
        
def shutdown():
    global limb
    global traj
    global lintimb_erface
    rospy.loginfo("Stopping the node...")
    #srv_action_client.delete_gazebo_models()
    traj.clear('right')
    traj.stop()

        
def main():
    global mode_no_state_trainsition_report
    global mode_no_anomaly_detection


    rospy.init_node("pick_n_place_joint_trajectory")
    rospy.on_shutdown(shutdown)
    if not mode_no_anomaly_detection:
        if mode_use_manual_anomaly_signal:
            rospy.Subscriber("/manual_anomaly_signal", std_msgs.msg.String, callback_manual_anomaly_signal)
        else:
            rospy.Subscriber("/hmm_online_result", Hmm_Log, callback_hmm)
 
    sm = smach.StateMachine(outcomes=['TaskFailed', 'TaskSucceed'])

    global traj
    global limb_interface
    global limb
    
    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)
   

    rospy.loginfo('Building state machine...')
    with sm:
        smach.StateMachine.add(
            'Go_to_Start_Position',
            Go_to_Start_Position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_to_Pick_Hover_Position'
            }
        )

        smach.StateMachine.add(
            'Go_to_Pick_Hover_Position',
            Go_to_Pick_Hover_Position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_to_Pick_Position'
            }
        )

        smach.StateMachine.add(
			'Go_to_Pick_Position',
			Go_to_Pick_Position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_to_Place_Hover_Position',
            }
        )

        smach.StateMachine.add(
			'Go_to_Place_Hover_Position',
			Go_to_Place_Hover_Position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_to_Place_Position'
            }
        )
                               
        smach.StateMachine.add(
			'Go_to_Place_Position',
			Go_to_Place_Position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'TaskSucceed'
            }
        )

        smach.StateMachine.add(
			'Recovery',
			Recovery(),
            transitions={
                'RecoveryFailed':'TaskFailed'
            }
        )
                               
    rospy.loginfo('Done...')

    rospy.loginfo('Bring up smach introspection server...')
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
    sis.start()
    rospy.loginfo('Done...')

    if not mode_no_state_trainsition_report:
        hmm_state_switch_client(0)

    rospy.loginfo('Start state machine execution...')
    outcome = sm.execute()
    rospy.loginfo('Done...')

    if not mode_no_state_trainsition_report:
        hmm_state_switch_client(0)

    rospy.spin()
    

if __name__ == '__main__':
    mode_no_state_trainsition_report = True
    mode_no_anomaly_detection = False 
    mode_use_manual_anomaly_signal = True
    sys.exit(main())



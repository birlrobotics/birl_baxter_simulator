#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun dmp dmp_joint_trajectory_action_server.py first!
"""


from dmp.srv import *
import sys
import rospy
from srv_client import *
import dmp_r_joint_trajectory_client
import smach
import smach_ros
import config
from std_msgs.msg import Empty
import os
from birl_sim_examples.srv import *

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

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """

    import cv2
    import cv_bridge
    from sensor_msgs.msg import (
        Image,
    )

    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


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


class Go_to_start_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Go_to_start_position...')
        global dmp0_traj
        dmp0_traj.parse_file(config.go_to_start_position_path)
        
        rospy.loginfo('read recording file Go_to_start_position trajectory')
        dmp0_traj.start()        
        rospy.loginfo('started')
        rospy.loginfo("wait returns %s"%(dmp0_traj.wait(),))
        rospy.loginfo('done wait')
        dmp0_traj.stop()
        rospy.loginfo('succeesfully run start_to_pick_trajectory')    
        return 'Succeed'


        
        
class Go_to_gripper_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Go_to_gripper_position......')
        global dmp1_traj
        dmp1_traj.parse_file(config.go_to_gripper_position_path)
        dmp1_traj.start()
        dmp1_traj.wait()
        rospy.loginfo('succeesfully run Go_to_gripper_position_trajectory')
        return 'Succeed'

class Go_back(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go_back..')
        global dmp2_traj
        dmp2_traj.parse_file(config.go_back_path)
        dmp2_traj.start()
        dmp2_traj.wait()
        return 'Succeed'

class Go_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
        
    def execute(self, userdata):
        rospy.loginfo('executing Go_forward..')
        global dmp3_traj
        dmp3_traj.parse_file(config.go_forward_path)
        dmp3_traj.start()
        dmp3_traj.wait()
        return 'Succeed'
class Go_back_to_start_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go_back_to_start_position...')
        global dmp4_traj
        dmp4_traj.parse_file(config.go_back_to_start_position_path)
        dmp4_traj.start()
        dmp4_traj.wait()       
        return 'Succeed'

class Gripper_open(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  Open gripper...')
        global dmp1_traj
        dmp1_traj.gripper_open()
        rospy.loginfo("gripper open")        
        return 'Succeed'


class Gripper_close(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed', 'NeedRecovery'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  close gripper...')
        global dmp1_traj
        dmp1_traj.gripper_close()
        rospy.loginfo("gripper close")        
        return 'Succeed'

class Recovery(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
        
    def execute(self, userdata):
        global event_flag
        global execution_history
        global sm
        global mode_no_state_trainsition_report

        rospy.loginfo("Enter Recovery State...")
        rospy.loginfo("Block anomlay detection")
        event_flag = -1

        send_image(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'red.jpg'))

        history_to_reexecute = None 
        while True:
            if len(execution_history) == 0:
                rospy.loginfo("no execution_history found")
            elif execution_history[-1]['depend_on_prev_states']:
                execution_history.pop()
            else:
                history_to_reexecute = execution_history[-1]
                break

        if history_to_reexecute is None:
            return 'RecoveryFailed'

        state_name = history_to_reexecute['state_name']

        state_instance = sm._states[state_name]
        state_transitions = sm._transitions[state_name]
        rospy.loginfo("Gonna call %s's execute with empty userdata"%(state_name,))
        if not mode_no_state_trainsition_report:
            hmm_state_switch_client(0)
            mode_no_state_trainsition_report = True
            state_outcome = state_instance.execute({}) 
            mode_no_state_trainsition_report = False
        else:
            state_outcome = state_instance.execute({}) 
            
        next_state = state_transitions[state_outcome]
        rospy.loginfo('Gonna reenter %s'%(next_state,))


        send_image(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'green.jpg'))

        rospy.loginfo("Unblock anomlay detection")
        event_flag = 1
        return 'Reenter_'+next_state


def callback_hmm(msg):
    global event_flag
    # if event_flag is not blocked by Recovery state
    if event_flag != -1:
        event_flag = msg.event_flag  

def callback_manual_anomaly_signal(msg):
    global event_flag
    if event_flag != -1:
        event_flag = 0

        
def main():
    global mode_no_state_trainsition_report
    global mode_no_anomaly_detection
    global sm

    global dmp0_traj
    global dmp1_traj
    global dmp2_traj
    global dmp3_traj
    global dmp4_traj


    rospy.init_node("open_drawer_joint_trajectory")
    if not mode_no_anomaly_detection:
        import std_msgs.msg
        if mode_use_manual_anomaly_signal:
            rospy.Subscriber("/manual_anomaly_signal", std_msgs.msg.String, callback_manual_anomaly_signal)
        else:
            from birl_sim_examples.msg import (
                Hmm_Log
            )
            rospy.Subscriber("/hmm_online_result", Hmm_Log, callback_hmm)

    sm = smach.StateMachine(outcomes=['TaskFailed', 'TaskSucceed'])

    
    dmp0_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp1_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp2_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp3_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp4_traj = dmp_r_joint_trajectory_client.Trajectory()
    with sm:

        smach.StateMachine.add(
            'Go_to_start_position', 
            Go_to_start_position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_to_gripper_position'
            }
        )
        
        smach.StateMachine.add(
            'Go_to_gripper_position',
            Go_to_gripper_position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Gripper_close'
            }
        )
                               
        smach.StateMachine.add(
            'Gripper_close',
            Gripper_close(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_back'
            }
        )
        
        smach.StateMachine.add(
            'Go_back',
            Go_back(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Gripper_open'
            }
        )
        
        smach.StateMachine.add(
            'Gripper_open',
            Gripper_open(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_forward'
            }
        )
        
        smach.StateMachine.add(
            'Go_forward',
            Go_forward(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'Go_back_to_start_position'
            }
        )
        
        smach.StateMachine.add(
            'Go_back_to_start_position',
            Go_back_to_start_position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Succeed':'TaskSucceed'
            }
        )


        # build Recovery states automatically
        recovery_outcomes = ['RecoveryFailed']
        recovery_state_transitions = {
            'RecoveryFailed':'TaskFailed'
        }
        for added_state in sm._states:
            recovery_outcomes.append('Reenter_'+added_state)
            recovery_state_transitions['Reenter_'+added_state] = added_state

        smach.StateMachine.add(
			'Recovery',
			Recovery(outcomes=recovery_outcomes),
            transitions=recovery_state_transitions
        )


    send_image(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'green.jpg'))


    if not mode_no_state_trainsition_report:
        hmm_state_switch_client(0)
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    if not mode_no_state_trainsition_report:
        hmm_state_switch_client(0)

    rospy.spin()


if __name__ == '__main__':
    mode_no_state_trainsition_report = False
    mode_no_anomaly_detection = False 
    mode_use_manual_anomaly_signal = False 
    sm = None
    sys.exit(main())




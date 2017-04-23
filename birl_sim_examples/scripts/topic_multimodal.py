#!/usr/bin/env python
"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy
import ipdb

import rospy
from std_msgs.msg import (
    Empty
)

from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from birl_sim_examples.msg import Tag_MultiModal
from birl_sim_examples.srv import (
    State_Switch,
    State_SwitchResponse
)



def callback_endpoint_state(endpoint_state):
    global tag_multimodal
    tag_multimodal.endpoint_state = endpoint_state

def callback_joint_state(joint_state):
    global tag_multimodal
    tag_multimodal.joint_state = joint_state

def callback_wrench_stamped(wrench_stamped):
    global tag_multimodal
    tag_multimodal.wrench_stamped = wrench_stamped

    
def state_switch_handle(req):
    """
    # change the hidden state of data, service msg type:
    int32 state
    ---
    # finish flag
    std_msgs/Bool finish
    """
    global hmm_state
    hmm_state = req.state
    print "state is changed to %d" %req.state
    resp = State_SwitchResponse()
    resp.finish.data = True
    return resp
    

def main():
    #ipdb.set_trace()
    global hmm_state
    global tag_multimodal
    tag_multimodal = Tag_MultiModal()
    hmm_state = 0
    
    rospy.init_node("topic_multimodal", anonymous=True)
    # Simulation waiting start up
    rospy.wait_for_message("/robot/sim/started", Empty)
    # set up Subscribers
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, callback_endpoint_state)
    rospy.Subscriber("/robot/joint_states", JointState, callback_joint_state)
    rospy.Subscriber("/wrench/filtered/right", WrenchStamped, callback_wrench_stamped)
    # set up publisher
    pub = rospy.Publisher("/tag_multimodal",Tag_MultiModal, queue_size=10)
    # set up service
    state_switch = rospy.Service('hmm_state_switch', State_Switch, state_switch_handle)
    # set up the publishing rate
    r = rospy.Rate(50)
    print "Topic /tag_multimodal publish rate: 50hz"
    while not rospy.is_shutdown():
        pub.publish(tag_multimodal)
        r.sleep()

if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python
"""
Baxter RSDK Inverse Kinematics Pick and Place Demo

This script is about data collection.

Will do 2 things here:

1. combine 3 channels of data sources into one: endpoints, joint states and wrench -> tag_multimodal
2. run a service that will change a global variable called HMM_state in this script on request.
1.1 append HMM_state to tag_multimodal publication




"""
import argparse
import struct
import sys
import copy
import ipdb

import rospy
from std_msgs.msg import (
    Empty,
    Header
)

import copy

from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from birl_sim_examples.msg import Tag_MultiModal
from birl_sim_examples.srv import (
    State_Switch,
    State_SwitchResponse
)

my_header = Header()

def callback_endpoint_state(endpoint_state):
    global tag_multimodal
    global my_header
    tag_multimodal.endpoint_state = endpoint_state
    my_header = copy.deepcopy(endpoint_state.header)

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
    global my_header
    tag_multimodal = Tag_MultiModal()
    hmm_state = 0
    publishing_rate = 100
    
    rospy.init_node("topic_multimodal", anonymous=True)
    # Simulation waiting start up
    #rospy.wait_for_message("/robot/sim/started", Empty)
    # set up Subscribers
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, callback_endpoint_state)
    rospy.Subscriber("/robot/joint_states", JointState, callback_joint_state)
    rospy.Subscriber("/wrench/filter", WrenchStamped, callback_wrench_stamped)
    # set up publisher
    pub = rospy.Publisher("/tag_multimodal",Tag_MultiModal, queue_size=10)
    # set up service
    state_switch = rospy.Service('hmm_state_switch', State_Switch, state_switch_handle)
    # set up the publishing rate
    r = rospy.Rate(publishing_rate)
    print "Topic /tag_multimodal publish rate: %d hz"%publishing_rate
    print "Topic /robot/limb/right/endpoint_state publish rate: 100hz"
    print "Topic /robot/joint_states publish rate: 120hz"
    print "Topic /wrench/filter publish rate: 200hz"
    
    while not rospy.is_shutdown():
        tag_multimodal.tag = hmm_state
        tag_multimodal.header = copy.deepcopy(my_header)
        pub.publish(tag_multimodal)
        r.sleep()

if __name__ == '__main__':
    sys.exit(main())

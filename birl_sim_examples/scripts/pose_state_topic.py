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
from birl_sim_examples.msg import Tag_EndpointPose
from birl_sim_examples.srv import (
    State_Switch,
    State_SwitchResponse
)



def callback(endpoint_state):
    tag_endpointpose = Tag_EndpointPose()
    tag_endpointpose.header = endpoint_state.header
    tag_endpointpose.pose = endpoint_state.pose
    tag_endpointpose.tag = hmm_state
    pub.publish(tag_endpointpose)

    
def state_switch_handle(req):
    """
    # change the hidden state of data
    int32 state

    ---
    
    # finish flag

    std_msgs/Bool finish

    """
    global hmm_state
    hmm_state = req.state
    print "state is %d" %req.state
    resp = State_SwitchResponse()
    resp.finish.data = True
    return resp
    

def main():
    ipdb.set_trace()
    global pub
    global hmm_state
    hmm_state = 0
    rospy.init_node("pose_hidden_state_topic", anonymous=True)
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, callback)
    pub = rospy.Publisher("/tag_endpointpose",Tag_EndpointPose, queue_size=10)
    state_switch = rospy.Service('hmm_state_switch', State_Switch, state_switch_handle)
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())

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


from geometry_msgs.msg import WrenchStamped 
from birl_sim_examples.msg import Tag_Wrench
from birl_sim_examples.srv import (
    State_Switch,
    State_SwitchResponse
)


def callback(data):
    global hmm_state
    global pub
    tag_wrench = Tag_Wrench()
    tag_wrench.wrench_stamped = copy.deepcopy(data)
    tag_wrench.tag = hmm_state
    pub.publish(tag_wrench)

    
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
    global wrench_stamp
    global hmm_state
    hmm_state = 0
    rospy.init_node("tag_wrench_topic", anonymous=True)
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.Subscriber("/wrench/filtered/right", WrenchStamped, callback)
    pub = rospy.Publisher("/tag_wrench",Tag_Wrench, queue_size=10)
    state_switch = rospy.Service('hmm_state_switch', State_Switch, state_switch_handle)
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())

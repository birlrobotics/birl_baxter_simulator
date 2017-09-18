#!/usr/bin/env python
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

import rosbag



sub_endpose_topic = '/robot/limb/right/endpoint_state'
sub_joint_state_topic = "/robot/joint_states"
sub_wrench_topic  =  "/wrench/filtered/right"

pub_topic = "/tag_multimodal"

service_topic = 'hmm_state_switch'


hmm_state = 0
publishing_rate = 100



shared_header = None
shared_endpoint_state = None
def callback_endpoint_state(endpoint_state):
    global shared_header
    global shared_endpoint_state 
    shared_header = endpoint_state.header
    shared_endpoint_state = endpoint_state

shared_joint_state = None
def callback_joint_state(joint_state):
    global shared_joint_state
    shared_joint_state = joint_state

shared_wrench_stamped = None
def callback_wrench_stamped(wrench_stamped):
    global shared_wrench_stamped
    shared_wrench_stamped = wrench_stamped


    
hmm_state = None

def state_switch_handle(req):
    global hmm_state
    hmm_state = req.state
    print "state is changed to %d" %req.state
    resp = State_SwitchResponse()
    resp.finish.data = True
    return resp
    

def main():
    global hmm_state
    global shared_header
    global shared_endpoint_state 
    global shared_joint_state
    global shared_wrench_stamped
    global publishing_rate
    global hmm_state

    Success_Info = True


    rospy.init_node("topic_multimodal", anonymous=True)

    # subscribe to these topics
    rospy.Subscriber(sub_endpose_topic, EndpointState, callback_endpoint_state)
    rospy.Subscriber(sub_joint_state_topic, JointState, callback_joint_state)
    rospy.Subscriber(sub_wrench_topic, WrenchStamped, callback_wrench_stamped)


    # set up the publisher
    pub = rospy.Publisher(pub_topic,Tag_MultiModal, queue_size=10)


    # set up the tag service server
    state_switch = rospy.Service(service_topic, State_Switch, state_switch_handle)

    r = rospy.Rate(publishing_rate)
    
    while not rospy.is_shutdown():
        if shared_header is None:
            rospy.loginfo("waiting for shared_header")
        elif shared_endpoint_state is None:
            rospy.loginfo("waiting for endpoint_state")
        elif shared_joint_state is None:
            rospy.loginfo("waiting for joint_state is missing")
        elif shared_wrench_stamped is None:
            rospy.loginfo("waiting for shared_wrench_stamped")
        else:
            tag_multimodal = Tag_MultiModal()
            tag_multimodal.tag = hmm_state
            tag_multimodal.header = copy.deepcopy(shared_header)
            tag_multimodal.endpoint_state = copy.deepcopy(shared_endpoint_state)
            tag_multimodal.joint_state = copy.deepcopy(shared_joint_state)
            tag_multimodal.wrench_stamped = copy.deepcopy(shared_wrench_stamped)
            if Success_Info:
                rospy.loginfo("Topics has been recieved and published successfully!")
                rospy.loginfo("Subscribe to topics:\n [1]%-20s \n [2]%-20s \n [3]%-20s \n ", sub_endpose_topic,sub_joint_state_topic,sub_wrench_topic)
                rospy.loginfo('Publish topic: \n [1] %s', pub_topic)
                rospy.loginfo('Service Server: \n [1] %s', service_topic)
                Success_Info = False
            try:
                pub.publish(tag_multimodal)
            except:
                ipdb.set_trace()
                rospy.loginfo("The node has expired, please check error!")

        r.sleep()

if __name__ == '__main__':
    sys.exit(main())















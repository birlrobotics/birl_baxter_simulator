#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun birl_sim_examples topic_multimodal first
"""

import rosbag

import rospy

import ipdb


from birl_sim_examples.msg import Tag_MultiModal

import sys
import os
import hardcoded_data
import rosbag_pandas

import copy

IS_BAG_RECORDING = False

IS_BAG_TO_CSV = False

TOPIC_START_FLAG = False


bag_index = 1

bag = None


def callback_multimodal(msg):

    global bag_index
    global bag
    global IS_BAG_RECORDING
    global IS_BAG_TO_CSV
    global TOPIC_START_FLAG
    TOPIC_START_FLAG = True

    # when the tag begin change from 0 to 1, record the bag
    if (msg.tag != 0) and (IS_BAG_RECORDING == False):
        if not os.path.isdir(os.path.join(hardcoded_data.bag_save_path,str(bag_index))):
            os.makedirs(os.path.join(hardcoded_data.bag_save_path,str(bag_index)))
        bag = rosbag.Bag(os.path.join(hardcoded_data.bag_save_path, str(bag_index), 'tag_multimodal.bag'), 'w')
        bag.write('/tag_multimodal', msg)
        IS_BAG_RECORDING = True

    if (msg.tag != 0) and (IS_BAG_RECORDING == True):
        bag.write('/tag_multimodal', msg)

    if (msg.tag == 0) and (IS_BAG_RECORDING == True):
    # when the tag begin change from not 0 to 0, save the bag
        bag.close()
        bag_index += 1
        IS_BAG_RECORDING =False
        rospy.loginfo("Bag No %d recording is done.",bag_index-1)
        IS_BAG_TO_CSV = True


def main():

    global IS_BAG_TO_CSV
    global bag_index

    INFO_FLAG = True

    # ipdb.set_trace()
    rospy.init_node("topic_recorder", anonymous=True)
    rospy.Subscriber("/tag_multimodal", Tag_MultiModal, callback_multimodal)


    if not os.path.isdir(hardcoded_data.bag_save_path):
        os.makedirs(hardcoded_data.bag_save_path)

    csv_index = 1
    #ipdb.set_trace()
    while not rospy.is_shutdown():
        if TOPIC_START_FLAG and INFO_FLAG:
            print "Start to subscribe to topic /tag_multimodal"
            INFO_FLAG = False
        if IS_BAG_TO_CSV:
            #ipdb.set_trace()
            csv_index = bag_index -1
            dataframe = rosbag_pandas.bag_to_dataframe(os.path.join(hardcoded_data.bag_save_path, str(csv_index), 'tag_multimodal.bag'))
            df = dataframe[['tag_multimodal__endpoint_state_pose_position_x',
                           'tag_multimodal__endpoint_state_pose_position_y',
                           'tag_multimodal__endpoint_state_pose_position_z',
                           'tag_multimodal__endpoint_state_pose_orientation_x',
                           'tag_multimodal__endpoint_state_pose_orientation_y',
                           'tag_multimodal__endpoint_state_pose_orientation_z',
                           'tag_multimodal__endpoint_state_pose_orientation_w',
                           'tag_multimodal__wrench_stamped_wrench_force_x',
                           'tag_multimodal__wrench_stamped_wrench_force_y',
                           'tag_multimodal__wrench_stamped_wrench_force_z',
                           'tag_multimodal__wrench_stamped_wrench_torque_x',
                           'tag_multimodal__wrench_stamped_wrench_torque_y',
                           'tag_multimodal__wrench_stamped_wrench_torque_z',
                           'tag_multimodal__tag']]
            df.columns = [  '.endpoint_state.pose.position.x',
                            '.endpoint_state.pose.position.y',
                            '.endpoint_state.pose.position.z',
                            '.endpoint_state.pose.orientation.x',
                            '.endpoint_state.pose.orientation.y',
                            '.endpoint_state.pose.orientation.z',
                            '.endpoint_state.pose.orientation.w',
                            '.wrench_stamped.wrench.force.x',
                            '.wrench_stamped.wrench.force.y',
                            '.wrench_stamped.wrench.force.z',
                            '.wrench_stamped.wrench.torque.x',
                            '.wrench_stamped.wrench.torque.y',
                            '.wrench_stamped.wrench.torque.z',
                            '.tag'
                          ]



            df.to_csv(os.path.join(hardcoded_data.bag_save_path, str(csv_index), str(csv_index)+'-tag_multimodal.csv'))
            rospy.loginfo("csv transform finished !!")
            IS_BAG_TO_CSV = False


if __name__ == '__main__':
    sys.exit(main())

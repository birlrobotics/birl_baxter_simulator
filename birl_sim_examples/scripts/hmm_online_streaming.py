#!/usr/bin/env python
import sys
import pandas as pd
import numpy as np
#from matplotlib import pyplot as plt
from hmmlearn.hmm import *
from sklearn.externals import joblib

from birl_sim_examples.msg import (
    Tag_EndpointPose,
    Hmm_Log
)

import os

import rospy
import ipdb

def callback(data):
     global tag_endpointpose
     # global df
     global topic_subscribe_flag
     # print "df shape %d %d" %df.shape
     global df
     tag_endpointpose.header = data.header
     tag_endpointpose.pose = data.pose
     tag_endpointpose.tag = data.tag
     df_append_data = {'x_position':[data.pose.position.x],
                       'y_position':[data.pose.position.y],
                       'z_position':[data.pose.position.z],
                       'x_orientation':[data.pose.orientation.x],
                       'y_orientation':[data.pose.orientation.y],
                       'z_orientation':[data.pose.orientation.z],
                       'w_orientation':[data.pose.orientation.w],
                       'tag':[data.tag]}
     df_append = pd.DataFrame(df_append_data, columns=['x_position','y_position','z_position','x_orientation','y_orientation','z_orientation','w_orientation','tag'])
     df = df.append(df_append, ignore_index = True)    
     topic_subscribe_flag = True
     #print "df shape %d %d" %df.shape
     #print "x:%f y:%f z:%f tag:%d" %(df.values[0,0],df_append.values[0,1],df_append.values[0,2],df_append.values[0,7])

def main():
    global tag_endpointpose
    global pub
    global X_1
    global df
    global topic_subscribe_flag
    #ipdb.set_trace()
    topic_subscribe_flag = False
    tag_endpointpose = Tag_EndpointPose()
    rospy.init_node('hmm_online_streaming', anonymous=True)
    rospy.Subscriber("/tag_endpointpose",Tag_EndpointPose, callback)
    pub = rospy.Publisher("/hmm_log",Hmm_Log, queue_size=10)
    
    model_1 = joblib.load("/home/ben/ML_data/pick_n_place_smach_tag/01/model/model_s1.pkl")
    df_data = {'x_position':[0.0],
               'y_position':[0.0],
               'z_position':[0.0],
               'x_orientation':[0.0],
               'y_orientation':[0.0],
               'z_orientation':[0.0],
               'w_orientation':[0.0],
               'tag':[0]}
    df = pd.DataFrame(df_data,columns=['x_position','y_position','z_position','x_orientation','y_orientation','z_orientation','w_orientation','tag'])

    log = 0.0
    hmm_log = Hmm_Log()
    r = rospy.Rate(200)
    while not rospy.is_shutdown():
         hmm_log.log.data = log
         pub.publish(hmm_log)
         index,column = df.shape
         r.sleep()
         if df.values[index-1,7] == 1:
              if topic_subscribe_flag:
                   X_1 = df.values[df.values[:,7] ==1]
                   X_1_ = X_1[:,:7]
                   log, Z = model_1.decode(X_1_)
                   topic_subscribe_flag = False

#    plt.title("Subtask1-trained HMM Model log-function ")
#    plt1 = plt.plot(O, log_array_1, label="Subtask 1",linewidth='3.0')
#    plt2 = plt.plot(O, log_array_2, label="Subtask 2",linewidth='3.0')
#    plt.legend(loc='lower left', frameon=True)
#    plt.show()
    return 0

if __name__ == '__main__':
    sys.exit(main())










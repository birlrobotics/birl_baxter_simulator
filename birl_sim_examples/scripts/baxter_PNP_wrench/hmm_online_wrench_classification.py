#!/usr/bin/env python
import sys
import os
import pandas as pd
import numpy as np
from hmmlearn.hmm import *
from sklearn.externals import joblib
import ipdb
from math import (
    log,
    exp
)
from sklearn.preprocessing import (
    scale,
    normalize
)

#######-----ros module----##########
import rospy
from std_msgs.msg import (
    Empty,
    Header
)

from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from birl_sim_examples.msg import (
    Tag_MultiModal,
    Hmm_Classification
)
from birl_sim_examples.srv import (
    State_Switch,
    State_SwitchResponse
)

import threading
mylock = threading.RLock()


data_arr = np.array([0])
hmm_state = 0
data_index = 0
df = pd.DataFrame()
header = Header()

success_path = "/home/ben/ML_data/REAL_BAXTER_PICK_N_PLACE_5_18/success"

model_save_path = "/home/ben/ML_data/REAL_BAXTER_PICK_N_PLACE_5_18/model/wrench"

figure_save_path = "/home/ben/ML_data/REAL_BAXTER_PICK_N_PLACE_5_18/figure/wrench"

class ROSThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)     

    def callback_multimodal(self,data):
        mylock.acquire()
        global hmm_state
        global data_arr
        global data_index
        global df
        global header
        hmm_state = data.tag
        header = data.wrench_stamped.header

        df_append_data = {'.wrench_stamped.wrench.force.x':[data.wrench_stamped.wrench.force.x],
                          '.wrench_stamped.wrench.force.y':[data.wrench_stamped.wrench.force.y],
                          '.wrench_stamped.wrench.force.z':[data.wrench_stamped.wrench.force.z],
                          '.wrench_stamped.wrench.torque.x':[data.wrench_stamped.wrench.torque.x],
                          '.wrench_stamped.wrench.torque.y':[data.wrench_stamped.wrench.torque.y],
                          '.wrench_stamped.wrench.torque.z':[data.wrench_stamped.wrench.torque.z],
                          '.tag':[data.tag]}
        df_append = pd.DataFrame(df_append_data, columns=['.wrench_stamped.wrench.force.x',
                                                          '.wrench_stamped.wrench.force.y',
                                                          '.wrench_stamped.wrench.force.z',
                                                          '.wrench_stamped.wrench.torque.x',
                                                          '.wrench_stamped.wrench.torque.y',
                                                          '.wrench_stamped.wrench.torque.z',
                                                          '.tag'])   
        df = df.append(df_append, ignore_index = True)
        df = df.fillna(method='ffill')
        data_arr = df.values[df.values[:,-1] ==hmm_state]
        data_arr = data_arr[:,:-1]
        data_index = data_arr.shape[0]
        mylock.release()

    def run(self):
        # set up Subscribers
        rospy.Subscriber("/tag_multimodal", Tag_MultiModal, self.callback_multimodal)
        print "Topic /tag_multimodal publish rate: 100 hz"
        print "Topic /robot/limb/right/endpoint_state publish rate: 100hz"
        print "Topic /robot/joint_states publish rate: 120hz"
        print "Topic /wrench/filter publish rate: 200hz"        

        while not rospy.is_shutdown():
            rospy.spin()

class HMMThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self) 
        #n_state = 10
        #n_iteraton = 100
        #covariance_type_string = 'diag'
        #preprocessing_scaling = False
        #preprocessing_normalize = False
        #data_feature = 6

        self.model_1 = joblib.load(model_save_path+"/multisequence_model/model_s1.pkl")
        self.model_2 = joblib.load(model_save_path+"/multisequence_model/model_s2.pkl")
        self.model_3 = joblib.load(model_save_path+"/multisequence_model/model_s3.pkl")
        self.model_4 = joblib.load(model_save_path+"/multisequence_model/model_s4.pkl")
        self.expected_log_1 = joblib.load(model_save_path+'/multisequence_model/expected_log.pkl')[0]
        self.expected_log_2 = joblib.load(model_save_path+'/multisequence_model/expected_log.pkl')[1]
        self.expected_log_3 = joblib.load(model_save_path+'/multisequence_model/expected_log.pkl')[2]
        self.expected_log_4 = joblib.load(model_save_path+'/multisequence_model/expected_log.pkl')[3]
        self.threshold_1 = joblib.load(model_save_path+'/multisequence_model/threshold.pkl')[0]
        self.threshold_2 = joblib.load(model_save_path+'/multisequence_model/threshold.pkl')[1]
        self.threshold_3 = joblib.load(model_save_path+'/multisequence_model/threshold.pkl')[2]
        self.threshold_4 = joblib.load(model_save_path+'/multisequence_model/threshold.pkl')[3]
    



    def run(self):
        #ipdb.set_trace()
        global data_arr
        global hmm_state
        global data_index
        global header
        hmm_classification = Hmm_Classification()
        publishing_rate = 50
        r = rospy.Rate(publishing_rate)
        pub = rospy.Publisher("/hmm_online_classifcation", Hmm_Classification, queue_size=10)
        while not rospy.is_shutdown():
            if not hmm_state == 0:
                try:
                    hmm_classification.state_1.data = self.model_1.score(data_arr)              
                    print "%d"%(data_index)
                except:
                    rospy.logerr("the data shape is %d ",data_index)

                try:
                    hmm_classification.state_2.data = self.model_2.score(data_arr)              
                    print "%d"%(data_index)
                except:
                    rospy.logerr("the data shape is %d ",data_index)
                try:
                    hmm_classification.state_3.data = self.model_3.score(data_arr)              
                    print "%d"%(data_index)
                except:
                    rospy.logerr("the data shape is %d ",data_index)
                try:
                    hmm_classification.state_4.data = self.model_4.score(data_arr)              
                    print "%d"%(data_index)
                except:
                    rospy.logerr("the data shape is %d ",data_index)

            hmm_classification.header = header
            pub.publish(hmm_classification)
            r.sleep()

        return 0

    
def main():
    rospy.init_node("hmm_online_parser", anonymous=True)
    thread1 = ROSThread()  
    thread2 = HMMThread()
    thread1.setDaemon(True)
    thread2.setDaemon(True)
    thread1.start()  
    thread2.start()
    while not rospy.is_shutdown():
        rospy.spin()
    return 0
    
    

if __name__ == '__main__':
    sys.exit(main())

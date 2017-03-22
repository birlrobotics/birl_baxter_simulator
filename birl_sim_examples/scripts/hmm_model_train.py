#!/usr/bin/env python
import sys
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from hmmlearn.hmm import *
from sklearn.externals import joblib
import ipdb



def main():
    ipdb.set_trace()

    n_state = 3
    
    df = pd.read_csv('/home/ben/ML_data/pick_n_place_smach_tag/01/tag_endpointpose.csv', sep=',')
    X_1 = df.values[df.values[:,12] ==1]
    index_1,column_1 = X_1.shape
    X_2 = df.values[df.values[:,12] ==2]
    index_2,column_2 = X_2.shape
    X_3 = df.values[df.values[:,12] ==3]
    index_3,column_3 = X_3.shape
    X_4 = df.values[df.values[:,12] ==4]
    index_4,column_4 = X_4.shape
    X_5 = df.values[df.values[:,12] ==5]
    index_5,column_5 = X_5.shape

    index = min(index_1,index_2,index_3,index_4,index_5)

    X_1_ = X_1[:index,5:12]
    X_2_ = X_2[:index,5:12]
    X_3_ = X_3[:index,5:12]
    X_4_ = X_4[:index,5:12]
    X_5_ = X_5[:index,5:12]


    Data = [X_1_,X_2_,X_3_,X_4_,X_5_]
        
    start_prob = np.zeros(3)
    start_prob[0] = 1
    model_1 =GaussianHMM(n_components=n_state, covariance_type="full",
                         params="mct", init_params="cmt", n_iter=1000)
    model_1.startprob_ = start_prob
    model_1 = model_1.fit(Data[0])

    model_2 =GaussianHMM(n_components=n_state, covariance_type="full",
                         params="mct", init_params="cmt", n_iter=1000)
    model_2.startprob_ = start_prob
    model_2 = model_2.fit(Data[1])

    model_3 =GaussianHMM(n_components=n_state, covariance_type="full",
                         params="mct", init_params="cmt", n_iter=1000)
    model_3.startprob_ = start_prob
    model_3 = model_3.fit(Data[2])

    model_4 =GaussianHMM(n_components=n_state, covariance_type="full",
                         params="mct", init_params="cmt", n_iter=1000)
    model_4.startprob_ = start_prob
    model_4 = model_4.fit(Data[3])

    model_5 =GaussianHMM(n_components=3, covariance_type="full",
                         params="mct", init_params="cmt", n_iter=1000)
    model_5.startprob_ = start_prob
    model_5 = model_5.fit(Data[4])

    # save the models
    joblib.dump(model_1,"/home/ben/ML_data/pick_n_place_smach_tag/01/model/model_s1.pkl")
    joblib.dump(model_2,"/home/ben/ML_data/pick_n_place_smach_tag/01/model/model_s2.pkl")
    joblib.dump(model_3,"/home/ben/ML_data/pick_n_place_smach_tag/01/model/model_s3.pkl")
    joblib.dump(model_4,"/home/ben/ML_data/pick_n_place_smach_tag/01/model/model_s4.pkl")
    joblib.dump(model_5,"/home/ben/ML_data/pick_n_place_smach_tag/01/model/model_s5.pkl")

    log_1_data = []
    log_2_data = []
    log_3_data = []
    log_4_data = []
    log_5_data = []
    for j in range(4):
        log_1 =[]
        log_2 =[]
        log_3 =[]
        log_4 =[]
        log_5 =[]
        O = []
        for i in range(2,index):
            O.append(i)
            log, Z = model_1.decode(Data[j][:i])
            log_1.append(log)
            log, Z = model_2.decode(Data[j][:i])
            log_2.append(log)
            log, Z = model_3.decode(Data[j][:i])
            log_3.append(log)
            log, Z = model_4.decode(Data[j][:i])
            log_4.append(log)
            #log, Z = model_5.decode(X_1_[:i])
            #log_5.append(log)
        log_1_data.append(log_1)
        log_2_data.append(log_2)
        log_3_data.append(log_3)
        log_4_data.append(log_4)
        log_5_data.append(log_5)

    plt1 = plt.figure(1,figsize=(40,30), dpi=80)
    #plt.subplot(2,1,1)
    plt.title("Subtask 1 -trained HMM Model log-function ")
    # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
    #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
    #plt.ylim(Min*1.1, Max*1.1)
    plt.plot(O, log_1_data[0][:], label="Subtask 1",linewidth='3.0')
    plt.plot(O, log_2_data[0][:], label="Subtask 2",linewidth='3.0')
    plt.plot(O, log_3_data[0][:], label="Subtask 3",linewidth='3.0')
    plt.plot(O, log_4_data[0][:], label="Subtask 4",linewidth='3.0')
            # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
    plt.legend(loc='lower left', frameon=True)
    plt.savefig("Subtask1.eps", format="eps")
    
    plt.figure(2,figsize=(40,30), dpi=100)
    
    #plt.subplot(2,1,2)
    plt.title("Subtask 2 -trained HMM Model log-function ")
    # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
    #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
    #plt.ylim(Min*1.1, Max*1.1)
    plt.plot(O, log_1_data[1][:], label="Subtask 1",linewidth='3.0')
    plt.plot(O, log_2_data[1][:], label="Subtask 2",linewidth='3.0')
    plt.plot(O, log_3_data[1][:], label="Subtask 3",linewidth='3.0')
    plt.plot(O, log_4_data[1][:], label="Subtask 4",linewidth='3.0')
            # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
    plt.legend(loc='lower left', frameon=True)
    plt.savefig("Subtask2.eps", format="eps")
    
    plt.figure(3,figsize=(40,30), dpi=80)
   # plt.subplot(2,1,3)
    plt.title("Subtask 3 -trained HMM Model log-function ")
    # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
    #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
    #plt.ylim(Min*1.1, Max*1.1)
    plt.plot(O, log_1_data[2][:], label="Subtask 1",linewidth='3.0')
    plt.plot(O, log_2_data[2][:], label="Subtask 2",linewidth='3.0')
    plt.plot(O, log_3_data[2][:], label="Subtask 3",linewidth='3.0')
    plt.plot(O, log_4_data[2][:], label="Subtask 4",linewidth='3.0')
            # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
    plt.legend(loc='lower left', frameon=True)
    plt.savefig("Subtask3.eps", format="eps")
    
    plt.figure(4,figsize=(40,30), dpi=80)
    #plt.subplot(2,1,4)
    plt.title("Subtask 4 -trained HMM Model log-function ")
    # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
    #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
    #plt.ylim(Min*1.1, Max*1.1)
    plt.plot(O, log_1_data[3][:], label="Subtask 1",linewidth='3.0')
    plt.plot(O, log_2_data[3][:], label="Subtask 2",linewidth='3.0')
    plt.plot(O, log_3_data[3][:], label="Subtask 3",linewidth='3.0')
    plt.plot(O, log_4_data[3][:], label="Subtask 4",linewidth='3.0')
            # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
    plt.legend(loc='lower left', frameon=True)
    plt.savefig("Subtask4.eps", format="eps")
    plt.show()
    return 0

if __name__ == '__main__':
    sys.exit(main())

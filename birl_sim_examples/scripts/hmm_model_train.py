#!/usr/bin/env python
import sys
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from hmmlearn.hmm import *
from sklearn.externals import joblib
import ipdb



def matplot_list(list_data):
    ax = plt.subplot(111)
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.spines['bottom'].set_position(('data',0))
    plt.grid(True)
    i = 0
    for data in list_data:
        i = i + 1
        index = np.asarray(data).shape
        O = (np.arange(index[0])).tolist()
        plt.plot(O, data, label="Subtask"+str(i),linewidth='3.0')
    plt.legend(loc='lower left', frameon=True)


def main():
    #ipdb.set_trace()

    n_state = 2

    #ipdb.set_trace()
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

    index = [index_1,index_2,index_3,index_4,index_5]

    X_1_ = X_1[:index_1,5:12]
    X_2_ = X_2[:index_2,5:12]
    X_3_ = X_3[:index_3,5:12]
    X_4_ = X_4[:index_4,5:12]
    X_5_ = X_5[:index_5,5:12]


    Data = [X_1_,X_2_,X_3_,X_4_,X_5_]
        
    start_prob = np.zeros(n_state)
    start_prob[0] = 1
    model_1 =GaussianHMM(n_components=n_state, covariance_type="diag",
                         params="mct", init_params="cmt", n_iter=1000)
    model_1.startprob_ = start_prob
    model_1 = model_1.fit(Data[0])

    model_2 =GaussianHMM(n_components=n_state, covariance_type="diag",
                         params="mct", init_params="cmt", n_iter=1000)
    model_2.startprob_ = start_prob
    model_2 = model_2.fit(Data[1])

    model_3 =GaussianHMM(n_components=n_state, covariance_type="diag",
                         params="mct", init_params="cmt", n_iter=1000)
    model_3.startprob_ = start_prob
    model_3 = model_3.fit(Data[2])

    model_4 =GaussianHMM(n_components=n_state, covariance_type="diag",
                         params="mct", init_params="cmt", n_iter=1000)
    model_4.startprob_ = start_prob
    model_4 = model_4.fit(Data[3])

    model_5 =GaussianHMM(n_components=2, covariance_type="diag",
                         params="mct", init_params="cmt", n_iter=1000)
    model_5.startprob_ = [1,0]
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
    log_1_sum_data = [0,0]
    log_2_sum_data = [0,0]
    log_3_sum_data = [0,0]
    log_4_sum_data = [0,0]
    log_5_sum_data = [0,0]
    
    ix_list = [index_1,index_2,index_3,index_4,index_5]
    for j in range(5):
        log_1 =[]
        log_2 =[]
        log_3 =[]
        log_4 =[]
        log_5 =[]
        for i in range(1,ix_list[j]):
            log, Z = model_1.decode(Data[j][:i])
            log_1.append(log)
            if i == 2:
                log_1_sum_data.append(log)
            else:
                log_1_sum_data.append(log+log_1_sum_data[-1])
            
            log, Z = model_2.decode(Data[j][:i])
            log_2.append(log)
            if i == 2:
                log_2_sum_data.append(log)
            else:
                log_2_sum_data.append(log+log_2_sum_data[-1])
            
            log, Z = model_3.decode(Data[j][:i])
            log_3.append(log)
            if i == 2:
                log_3_sum_data.append(log)
            else:
                log_3_sum_data.append(log+log_3_sum_data[-1])
            
            log, Z = model_4.decode(Data[j][:i])
            log_4.append(log)
            if i == 2:
                log_4_sum_data.append(log)
            else:
                log_4_sum_data.append(log+log_4_sum_data[-1])
            
            log, Z = model_5.decode(Data[j][:i])
            log_5.append(log)
            if i == 2:
                log_5_sum_data.append(log)
            else:
                log_5_sum_data.append(log+log_5_sum_data[-1])
        log_1_data.append(log_1)
        log_2_data.append(log_2)
        log_3_data.append(log_3)
        log_4_data.append(log_4)
        log_5_data.append(log_5)

    task_log_data = []
    for i in range(5):
        task_log_data.append([log_1_data[i],log_2_data[i],log_3_data[i],log_4_data[i],log_5_data[i]])

    task_log_sum_data = [log_1_sum_data,log_2_sum_data,log_3_sum_data,log_4_sum_data,log_5_sum_data]
        
    plt.figure(1,figsize=(40,30), dpi=80)
    plt.title("Subtask 1 -trained HMM Model log-function ")
    matplot_list(task_log_data[0])
    plt.savefig("Subtask1.eps", format="eps")

    plt.figure(2,figsize=(40,30), dpi=80)
    plt.title("Subtask 2 -trained HMM Model log-function ")
    matplot_list(task_log_data[1])
    plt.savefig("Subtask2.eps", format="eps")

    plt.figure(3,figsize=(40,30), dpi=80)
    plt.title("Subtask 3 -trained HMM Model log-function ")
    matplot_list(task_log_data[2])
    plt.savefig("Subtask3.eps", format="eps")    

    plt.figure(4,figsize=(40,30), dpi=80)
    plt.title("Subtask 4 -trained HMM Model log-function ")
    matplot_list(task_log_data[3])
    plt.savefig("Subtask4.eps", format="eps")

    plt.figure(5,figsize=(40,30), dpi=80)
    plt.title("Subtask 5 -trained HMM Model log-function ")
    matplot_list(task_log_data[4])
    plt.savefig("Subtask5.eps", format="eps")

    
    plt.figure(6,figsize=(40,30), dpi=80)
    # ax = plt.subplot(111)
    # ax.spines['right'].set_color('none')
    # ax.spines['top'].set_color('none')
    # ax.xaxis.set_ticks_position('bottom')
    # ax.spines['bottom'].set_position(('data',0))
    # ax.yaxis.set_ticks_position('left')
    # ax.spines['left'].set_position(('data',0))
    plt.title("Full Data HMM Model log-sum-function ")
    #plt.grid(True)
    matplot_list(task_log_sum_data)
    index_cum = 2
    for i in range(5):
        index_cum = index[i] + index_cum -1
        plt.plot([index_cum, index_cum], [-1*10**8, 5*10**7], color ='grey', linewidth=2,linestyle="--")
        print "subtask data index:%d"%index_cum
    plt.savefig("Full_sum_log.eps", format="eps")

    
   #  plt.figure(2,figsize=(40,30), dpi=100)
    
   #  #plt.subplot(2,1,2)
   #  plt.title("Subtask 2 -trained HMM Model log-function ")
   #  # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
   #  #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
   #  #plt.ylim(Min*1.1, Max*1.1)
   #  plt.plot(O, log_1_data[1][:], label="Subtask 1",linewidth='3.0')
   #  plt.plot(O, log_2_data[1][:], label="Subtask 2",linewidth='3.0')
   #  plt.plot(O, log_3_data[1][:], label="Subtask 3",linewidth='3.0')
   #  plt.plot(O, log_4_data[1][:], label="Subtask 4",linewidth='3.0')
   #          # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
   #  plt.legend(loc='lower left', frameon=True)
   #  plt.savefig("Subtask2.eps", format="eps")
    
   #  plt.figure(3,figsize=(40,30), dpi=80)
   # # plt.subplot(2,1,3)
   #  plt.title("Subtask 3 -trained HMM Model log-function ")
   #  # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
   #  #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
   #  #plt.ylim(Min*1.1, Max*1.1)
   #  plt.plot(O, log_1_data[2][:], label="Subtask 1",linewidth='3.0')
   #  plt.plot(O, log_2_data[2][:], label="Subtask 2",linewidth='3.0')
   #  plt.plot(O, log_3_data[2][:], label="Subtask 3",linewidth='3.0')
   #  plt.plot(O, log_4_data[2][:], label="Subtask 4",linewidth='3.0')
   #          # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
   #  plt.legend(loc='lower left', frameon=True)
   #  plt.savefig("Subtask3.eps", format="eps")
    
   #  plt.figure(4,figsize=(40,30), dpi=80)
   #  #plt.subplot(2,1,4)
   #  plt.title("Subtask 4 -trained HMM Model log-function ")
   #  # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
   #  #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
   #  #plt.ylim(Min*1.1, Max*1.1)
   #  plt.plot(O, log_1_data[3][:], label="Subtask 1",linewidth='3.0')
   #  plt.plot(O, log_2_data[3][:], label="Subtask 2",linewidth='3.0')
   #  plt.plot(O, log_3_data[3][:], label="Subtask 3",linewidth='3.0')
   #  plt.plot(O, log_4_data[3][:], label="Subtask 4",linewidth='3.0')
   #          # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
   #  plt.legend(loc='lower left', frameon=True)
   #  plt.savefig("Subtask4.eps", format="eps")


   #  O1 = np.arange()

   #  plt.figure(4,figsize=(40,30), dpi=80)
   #  #plt.subplot(2,1,4)
   #  plt.title("Subtask 4 -trained HMM Model log-function ")
   #  # Max = max(max(log_1),max(log_2),max(log_3),max(log_4),max(log_5))
   #  #  Min = min(min(log_1),min(log_2),min(log_3),min(log_4),min(log_5))
   #  #plt.ylim(Min*1.1, Max*1.1)
   #  plt.plot(O, log_1_data[3][:], label="Subtask 1",linewidth='3.0')
   #  plt.plot(O, log_2_data[3][:], label="Subtask 2",linewidth='3.0')
   #  plt.plot(O, log_3_data[3][:], label="Subtask 3",linewidth='3.0')
   #  plt.plot(O, log_4_data[3][:], label="Subtask 4",linewidth='3.0')
   #          # plt5 = plt.plot(O, log_5, label="Subtask 5",linewidth='3.0')
   #  plt.legend(loc='lower left', frameon=True)
   #  plt.savefig("Subtask4.eps", format="eps")

    
    plt.show()
    return 0

if __name__ == '__main__':
    sys.exit(main())

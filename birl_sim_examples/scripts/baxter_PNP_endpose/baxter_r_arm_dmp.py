#!/usr/bin/env python
import roslib;
import sys
import rospy
import numpy as np
import pandas as pd 
from dmp.srv import *
from dmp.msg import *
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState




class Dmp(object):
    def __init__(self):
         self.q_s0 = None
         self.q_s1 = None
         self.q_e0 = None
         self.q_e1 = None
         self.q_w0 = None
         self.q_w1 = None
         self.q_w2 = None
  
    def callback(self,data):    
        self.q_s0 = data.command[0]
        self.q_s1 = data.command[1]
        self.q_e0 = data.command[2]
        self.q_e1= data.command[3]
        self.q_w0= data.command[4]
        self.q_w1= data.command[5]
        self.q_w2= data.command[6]
        rospy.loginfo("q_s0 %s", data.command[0])
        rospy.loginfo("q_s1 %s", data.command[1])
        rospy.loginfo("q_e0 %s", data.command[2])
        rospy.loginfo("q_e1 %s", data.command[3])
        rospy.loginfo("q_w0 %s", data.command[4])
        rospy.loginfo("q_w1 %s", data.command[5])
        rospy.loginfo("q_w2 %s", data.command[6])
    
    def setpoint_callback(self,data):
        self.j_s0 = data.position[9]
        self.j_s0 = data.position[10]
        self.j_s0 = data.position[11]
        self.j_s0 = data.position[12]
        self.j_s0 = data.position[13]
        self.j_s0 = data.position[14]
        self.j_s0 = data.position[15]
        self.j_s0 = data.position[16]
        
          
#Learn a DMP from demonstration data
    def makeLFDRequest(self,dims, traj, dt, K_gain,
                       D_gain, num_bases):
        demotraj = DMPTraj()
    
        for i in range(len(traj)):
            pt = DMPPoint();
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
    
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
    
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"
    
        return resp;


#Set a DMP as active for planning
    def makeSetActiveRequest(self,dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


#Generate a plan from a DMP
    def makePlanRequest(self,x_0, x_dot_0, t_0, goal, goal_thresh,
                        seg_length, tau, dt, integrate_iter):
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                       seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"
    
        return resp;


def main(record_trajectory_path, generalized_dmp_trajectory_path, starting_angles):
    dmp = Dmp()
    
    # read file
    train_set = pd.read_csv(record_trajectory_path)
    


    train_len = len(train_set)
    resample_t = np.linspace(train_set.values[0,0],train_set.values[-1,0],train_len)
    joint0_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,9])
    joint1_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,10])
    joint2_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,11])
    joint3_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,12])
    joint4_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,13])
    joint5_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,14])
    joint6_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,15])
    
    traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
    for i in range(train_len):
        traj[i] = [joint0_data[i],joint1_data[i],joint2_data[i],joint3_data[i],joint4_data[i],joint5_data[i],joint6_data[i]]

    

    #Create a DMP from a 7-D trajectory
    dims = 7
    dt = 0.01
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 200

    resp = dmp.makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    dmp.makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = starting_angles
#    x_0 = [0.498222186231,-0.631997082801,-0.0291949014446,1.03494845382,0.0677285306576,1.22626257012,-0.35325073918]
    
    x_dot_0 = [0.4, 0.4, 0.4, 0.4, 0.4, 0.0, 0.4]
    
    t_0 = train_set.values[0,0] # better to choose the starting time in the record file

    goal =[ joint0_data[-1],joint1_data[-1],
    joint2_data[-1], joint3_data[-1],joint4_data[-1],joint5_data[-1], joint6_data[-1]         ]
    
    goal_thresh = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]   
    
    seg_length = -1          #Plan until convergence to goal
    
    tau = 1 * resp.tau       #Desired plan should take twice as long as demo #let see change to 1
#    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1
    
    plan = dmp.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)
    
    
    
####################################### finish dmp #################################################################    
    
####################################### plot the curve generated by dmp #################################################################
    
    Column0_plan = [0.0]*len(plan.plan.times)
    Column1_plan = [0.0]*len(plan.plan.times)
    Column2_plan = [0.0]*len(plan.plan.times)
    Column3_plan = [0.0]*len(plan.plan.times)
    Column4_plan = [0.0]*len(plan.plan.times)
    Column5_plan = [0.0]*len(plan.plan.times)
    Column6_plan = [0.0]*len(plan.plan.times)
    for i in range(len(plan.plan.times)):    
        Column0_plan[i] = plan.plan.points[i].positions[0]
        Column1_plan[i] = plan.plan.points[i].positions[1]
        Column2_plan[i] = plan.plan.points[i].positions[2]
        Column3_plan[i] = plan.plan.points[i].positions[3]
        Column4_plan[i] = plan.plan.points[i].positions[4]
        Column5_plan[i] = plan.plan.points[i].positions[5]
        Column6_plan[i] = plan.plan.points[i].positions[6]
        
    resample_t0 = np.linspace(0.01,plan.plan.times[-1], train_len)
    joint0_data_plan = np.interp(resample_t0, plan.plan.times, Column0_plan)
    joint1_data_plan = np.interp(resample_t0, plan.plan.times, Column1_plan)
    joint2_data_plan = np.interp(resample_t0, plan.plan.times, Column2_plan)
    joint3_data_plan = np.interp(resample_t0, plan.plan.times, Column3_plan)
    joint4_data_plan = np.interp(resample_t0, plan.plan.times, Column4_plan)
    joint5_data_plan = np.interp(resample_t0, plan.plan.times, Column5_plan)
    joint6_data_plan = np.interp(resample_t0, plan.plan.times, Column6_plan)
##########  record the plan trajectory 
    WriteFileDir = generalized_dmp_trajectory_path   ## the path of generated dmp traj
    plan_len = len(plan.plan.times)
    f = open(WriteFileDir,'w')
    f.write('time,')
    f.write('right_s0,')
    f.write('right_s1,')
    f.write('right_e0,')
    f.write('right_e1,')
    f.write('right_w0,')
    f.write('right_w1,')
    f.write('right_w2\n')
        
    for i in range(train_len):
        f.write("%f," % (resample_t[i],))
        f.write(str(joint0_data_plan[i])+','+str(joint1_data_plan[i])+','+str(joint2_data_plan[i])+','
        +str(joint3_data_plan[i])+','+str(joint4_data_plan[i])+','+str(joint5_data_plan[i])+','+str(joint6_data_plan[i])
        +'\n')        
    f.close()

if __name__ == '__main__':
    record_trajectory_path = '/home/sklaw/Desktop/experiment/ros/indigo/baxter_ws/src/birl_baxter_dmp/dmp/open_drawer_datasets/go_to_start_position1.txt'
    generalized_dmp_trajectory_path = 'tmp.txt'
    starting_angles = [0.498222186231,-0.631997082801,-0.0291949014446,1.03494845382,0.0677285306576,1.22626257012,-0.35325073918]
    main(record_trajectory_path, generalized_dmp_trajectory_path, starting_angles)

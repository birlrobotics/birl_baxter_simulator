#!/usr/bin/env python
"""
pick and place  smach service client
"""

from arm_move import pick_and_place
from birl_sim_examples.srv import *

import sys
import rospy
import copy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Empty,
)

from smach import *
from smach_ros import *

from smach_msgs.msg import *

import ipdb
      

def main():
    """pick_n_place_smach_client smach example

    A smach client example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    """
    #ipdb.set_trace()
    #initiating the node
    rospy.init_node("pick_n_place_smach_client")
    #wait for simulation evironment set up
    rospy.wait_for_message("/robot/sim/started", Empty)

    #set the paramemeter of pick and place object pose
    pick_object_pose = Pose()
    place_object_pose = Pose()
    pick_object_pose.position.x = 0.6 
    pick_object_pose.position.y = 0 
    pick_object_pose.position.z = -0.115 - 0.005
    
    place_object_pose.position.x = 0.6 
    place_object_pose.position.y = -0.2 
    place_object_pose.position.z = -0.115 - 0.005
    
    pick_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
    
    place_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
    _pose = copy.deepcopy(pick_object_pose)

    # state machine set up
    sm = StateMachine(outcomes=['succeeded','aborted','preempted','done'])
    with sm:
        # setting up all the service response and request call back functions
        def add_gazebo_male_model_client_response_cb(userdata, response):
            userdata.foo_var_out = 'foo!'
            return 'succeeded'
        def add_gazebo_male_model_client_request_cb(userdata,request):
            req = Add_Gazebo_ModelRequest()
            req.model_name.data = "box_male"
            req.model_pose = Pose(position=Point(x=0.6, y=0, z=-0.115),
                                  orientation=Quaternion(x=0,y=0,z=0,w=1))
            req.model_reference_frame.data = "base"
            return req
        
        def add_gazebo_female_model_client_response_cb(userdata, response):
            userdata.foo_var_out = 'foo!'
            return 'succeeded'
        def add_gazebo_female_model_client_request_cb(userdata,request):
            req = Add_Gazebo_ModelRequest()
            req.model_name.data = "box_female"
            req.model_pose = Pose(position=Point(x=0.6, y=0.4, z=-0.115),
                                  orientation=Quaternion(x=0,y=0,z=0,w=1))
            req.model_reference_frame.data = "base"
            return req

        def go_to_start_position_client_request_cb(userdata, request):
            req = Go_To_Start_PositionRequest()
            req.start.data = True
            return req
        def go_to_start_position_client_response_cb(userdata, response):
            return 'succeeded'

        def go_to_hover_pick_position_client_request_cb(userdata, request):
            pick_object_pose = Pose()
            hover_distance = 0.15
            pick_object_pose.position.x = 0.6 
            pick_object_pose.position.y = 0 
            pick_object_pose.position.z = -0.115 - 0.005 + hover_distance
            pick_object_pose.orientation = Quaternion(
                x=0.0,
                y=1.0,
                z=0.0,
                w=6.123233995736766e-17)
            req = Go_To_PositionRequest()
            req.pose = pick_object_pose
            return req
        def go_to_hover_pick_position_client_response_cb(userdata, response):
            return 'succeeded'
        
        def go_to_pick_position_client_request_cb(userdata, request):
            pick_object_pose = Pose()
            hover_distance = 0.15
            pick_object_pose.position.x = 0.6 
            pick_object_pose.position.y = 0 
            pick_object_pose.position.z = -0.115 - 0.005 
            pick_object_pose.orientation = Quaternion(
                x=0.0,
                y=1.0,
                z=0.0,
                w=6.123233995736766e-17)
            req = Go_To_PositionRequest()
            req.pose = pick_object_pose
            return req
        def go_to_pick_position_client_response_cb(userdata, response):
            return 'succeeded'

        def gripper_move_open_client_request_cb(userdata, request):
            req = Gripper_MoveRequest()
            req.gripper_desired_flag.data = False
            return req
        def gripper_move_open_client_response_cb(userdata, response):
            return 'succeeded'        
        
        def gripper_move_close_client_request_cb(userdata, request):
            req = Gripper_MoveRequest()
            req.gripper_desired_flag.data = True
            return req
        def gripper_move_close_client_response_cb(userdata, response):
            return 'succeeded'

        # initiating all the nest of smach 
        StateMachine.add('Creating_box_model',
                         ServiceState('add_gazebo_box_model',
                                      Add_Gazebo_Model,
                                      request_cb=add_gazebo_male_model_client_request_cb,
                                      response_cb=add_gazebo_male_model_client_response_cb,
                                      output_keys=['foo_var_out']),
                         remapping={'foo_var_out':'sm_var'},
                         transitions={'succeeded':'Creating_female_box_model'})

        StateMachine.add('Creating_female_box_model',
                         ServiceState('add_gazebo_box_model',
                                      Add_Gazebo_Model,
                                      request_cb=add_gazebo_female_model_client_request_cb,
                                      response_cb=add_gazebo_female_model_client_response_cb,
                                      output_keys=['foo_var_out']),
                         remapping={'foo_var_out':'sm_var'},
                         transitions={'succeeded':'Go_To_Start_Position'})
            
        StateMachine.add('Go_To_Start_Position',
                         ServiceState('go_to_start_position',
                                      Go_To_Start_Position,
                                      request_cb=go_to_start_position_client_request_cb,
                                      response_cb=go_to_start_position_client_response_cb,
                                      output_keys=['foo_var_out']),
                         remapping={'foo_var_out':'sm_var'},
                         transitions={'succeeded':'Go_To_Hover_Pick_Position'})
        StateMachine.add('Go_To_Hover_Pick_Position',
                         ServiceState('go_to_position',
                                      Go_To_Position,
                                      request_cb=go_to_hover_pick_position_client_request_cb,
                                      response_cb=go_to_hover_pick_position_client_response_cb,
                                      output_keys=['foo_var_out']),
                         remapping={'foo_var_out':'sm_var'},
                         transitions={'succeeded':'Go_To_Pick_Position'})
        StateMachine.add('Go_To_Pick_Position',
                         ServiceState('go_to_position',
                                      Go_To_Position,
                                      request_cb=go_to_pick_position_client_request_cb,
                                      response_cb=go_to_pick_position_client_response_cb,
                                      output_keys=['foo_var_out']),
                         remapping={'foo_var_out':'sm_var'},
                         transitions={'succeeded':'Go_To_Pick_Position_Gripper_Open'})
        StateMachine.add('Go_To_Pick_Position_Gripper_Open',
                         ServiceState('gripper_move',
                                      Gripper_Move,
                                      request_cb=gripper_move_open_client_request_cb,
                                      response_cb=gripper_move_open_client_response_cb,
                                      output_keys=['foo_var_out']),
                         remapping={'foo_var_out':'sm_var'},
                         transitions={'succeeded':'Go_To_Pick_Position_Gripper_Close'})        
        StateMachine.add('Go_To_Pick_Position_Gripper_Close',
                         ServiceState('gripper_move',
                                      Gripper_Move,
                                      request_cb=gripper_move_close_client_request_cb,
                                      response_cb=gripper_move_close_client_response_cb,
                                      output_keys=['foo_var_out']),
                         remapping={'foo_var_out':'sm_var'},
                         transitions={'succeeded':'Go_To_Hover_Pick_Position'})
        
    #using smach_view to visualize the nest graph
    #This requires starting the smach_view node
    sis = IntrospectionServer('My_pick_n_place_smach_client',sm, '/SM_ROOT')
    sis.start()

    #wait for view start
    rospy.sleep(3.0)
    #excute the smach service 
    outcome = sm.execute()
    
    rospy.loginfo("OUTCOME: "+outcome)

    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    sys.exit(main())



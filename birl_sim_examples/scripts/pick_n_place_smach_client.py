#/usr/bin/env python
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

def add_gazebo_model_client(_model_name,
                            _model_pose=Pose(position=Point(x=0.6, y=0, z=-0.115),
                                            orientation=Quaternion(x=0,y=0,z=0,w=1)),
                            _model_reference_frame="base"):
    rospy.wait_for_service('add_gazebo_box_model')
    try:
        add_gazebo_box_model = rospy.ServiceProxy('add_gazebo_box_model',
                                                  Add_Gazebo_Model)
        req = Add_Gazebo_ModelRequest()
        req.model_name.data = _model_name
        req.model_pose = _model_pose
        req.model_reference_frame.data = _model_reference_frame
        resp = add_gazebo_box_model(req)
        rospy.loginfo("loading box succuessfuly")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
def go_to_start_position_client():
    rospy.wait_for_service('go_to_start_position')
    try:
        go_to_start_position_proxy = rospy.ServiceProxy('go_to_start_position',
                                                  Go_To_Start_Position)
        req = Go_To_Start_PositionRequest()
        req.start.data = True
        resp = go_to_start_position_proxy(req)
        rospy.loginfo("go to start position succesfully!")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def go_to_position_client(pose):
    rospy.wait_for_service('go_to_position')
    try:
        go_to_position_proxy = rospy.ServiceProxy('go_to_position',
                                                  Go_To_Position)
        req = Go_To_PositionRequest()
        req.pose = copy.deepcopy(pose)
        resp = go_to_position_proxy(req)
        rospy.loginfo("go to desired position succesfully!")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e          

def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    #ipdb.set_trace()
    rospy.init_node("pick_n_place_smach_client")
    rospy.wait_for_message("/robot/sim/started", Empty)
    ipdb.set_trace()

 
    pick_object_pose = Pose()
    place_object_pose = Pose()
    #the position of female box
    pick_object_pose.position.x = 0.6 
    pick_object_pose.position.y = 0 
    pick_object_pose.position.z = -0.115 - 0.005
    
    place_object_pose.position.x = 0.6 
    place_object_pose.position.y = -0.2 
    place_object_pose.position.z = -0.115 - 0.005
    #RPY = 0 pi 0
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



    sm = StateMachine(outcomes=['succeeded','aborted','preempted','done'])
    with sm:
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
                         transitions={'succeeded':'Go_To_Hover_Pick_Position'})
    #add smach viewer to view
    sis = IntrospectionServer('My_server',sm, '/SM_ROOT')
    sis.start()

    #waitfor view start
    rospy.sleep(3.0)
    #excute the service 
    outcome = sm.execute()

    rospy.loginfo("OUTCOME: "+outcome)

    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    sys.exit(main())



#!/usr/bin/env python
"""
pick and place service server
"""

from arm_move import pick_and_place

import sys
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Empty,
)
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
    rospy.init_node("pa_box")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    # Remove models from the scene on shutdown
   # rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)
    pick_and_place.load_gazebo_models(model_name="box_male",
                                      model_pose=Pose(position=Point(x=0.6, y=0, z=-0.115),
                                                      orientation=Quaternion(x=0,y=0,z=0,w=1)),
                                     model_reference_frame="base")
    pick_and_place.load_gazebo_models(model_name="box_female",
                                      model_pose=Pose(position=Point(x=0.6, y=0.4, z=-0.115),
                                                      orientation=Quaternion(x=0,y=0,z=0,w=1)),
                                      model_reference_frame="base")
    limb = 'right'
    hover_distance = 0.15 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_w0': -0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': 0.4999997247485215,
                             'right_e0': -0.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': 0.08000397926829805,
                             'right_s1': -0.9999781166910306}
    pnp = pick_and_place.PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    
    block_poses = list()

    #object position with a tranform offset between gripper and male part.
    object_pose = Pose()

    #the position of female box
    object_pose.position.x = 0.6 
    object_pose.position.y = 0 
    object_pose.position.z = -0.115 - 0.005

    #RPY = 0 pi 0
    object_orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
    
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=object_pose.position.x, y=object_pose.position.y, z=object_pose.position.z),
        orientation=object_orientation))
    
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.6, y=-0.2, z=-0.115-0.005),
        orientation=object_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        print("\nPicking...")
        pnp.pick(block_poses[idx])
        print("\nPlacing...")
        idx = (idx+1) % len(block_poses)
        pnp.place(block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())



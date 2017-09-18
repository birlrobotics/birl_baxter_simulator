from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

import copy

import os
import datetime


home = os.path.expanduser("~")
bag_save_path = os.path.join(home,"ML_DATA\Sim")

hover_distance = 0.15

place_pick_y_diff = 0.3

user_data = {
    'gazebo_model_pose':    Pose(Point(x=0.6,
                                  y=-0.4,
                                  z=-0.115),
                            Quaternion(x=0.0,
                                       y=0.0,
                                       z=0.0,
                                       w=1)),

    'right_start_joint_angles' :    {'right_w0': -0.6699952259595108,
                                    'right_w1': 1.030009435085784,
                                    'right_w2': 0.4999997247485215,
                                    'right_e0': -0.189968899785275,
                                    'right_e1': 1.9400238130755056,
                                    'right_s0': 0.08000397926829805,
                                    'right_s1': -0.9999781166910306},

    'pick_goal_pose' : Pose(Point(x=0.6,
                                  y=-0.4,
                                  z=-0.12),
                            Quaternion(x=0.0,
                                       y=1.0,
                                       z=0.0,
                                       w=6.123233995736766e-17)),
    'place_goal_pose': Pose(Point(x=0.6,
                                 y=-0.1,
                                 z=-0.12),
                            Quaternion(x=0.0,
                                       y=1.0,
                                       z=0.0,
                                       w=6.123233995736766e-17)),
    'hover_distance' : hover_distance,

    'place_hover_goal_pose': Pose(Point(x=0.6,
                                      y=-0.1,
                                      z=-0.12 + hover_distance),
                                  Quaternion(x=0.0,
                                             y=1.0,
                                             z=0.0,
                                             w=6.123233995736766e-17)),

    'pick_hover_goal_pose': Pose(Point(x=0.6,
                                      y=-0.4,
                                      z=-0.12 + hover_distance),
                                 Quaternion(x=0.0,
                                            y=1.0,
                                            z=0.0,
                                            w=6.123233995736766e-17))
}


# def create_new_path(path = 'default'):
#     if path is not 'default':
#         if os.path.isdir(path):
#             logerr('%s is already exist!', path)
#         else:
#             home = os.path.expanduser("~")
#             date = datetime.datetime.now()
#             tail_dir = date.year + '.' + date.month + '.' date.day
#             mid_dir = 'ML_DATA\Sim_Baxter_PNP'
#             os.mkdir(os.path.join(home,'ML_DATA'))



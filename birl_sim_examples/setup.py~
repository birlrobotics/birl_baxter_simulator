#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['arm_move','srv_client','srv_action_client']
d['package_dir'] = {'': 'src'}

setup(**d)

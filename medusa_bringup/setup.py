#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['medusa_bringup_algorithms', 'medusa_bringup_ros'],
 package_dir={'medusa_bringup_algorithms': 'src/medusa_bringup_algorithms', 'medusa_bringup_ros': 'src/medusa_bringup_ros'}
)

setup(**d)

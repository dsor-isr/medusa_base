#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['medusa_identification_algorithms', 'medusa_identification_ros'],
 package_dir={'medusa_identification_algorithms': 'src/medusa_identification_algorithms', 'medusa_identification_ros': 'src/medusa_identification_ros'}
)

setup(**d)

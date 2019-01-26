#!/usr/bin/env python

#TODO can I delete this file

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['imu_kalman'],
    package_dir = {'': 'src'}
)

setup(**setup_args)

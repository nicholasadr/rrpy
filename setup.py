#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rr_orpy', 'rr_ros', 'rr_task'],
    package_dir={'': 'src'})

setup(**d)

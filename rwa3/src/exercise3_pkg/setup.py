#!/usr/bin/env python

# DO NOT USE
# python setup.py install

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=['exercise3_pkg'],
    package_dir={'': 'scripts'}
)

setup(**setup_args)

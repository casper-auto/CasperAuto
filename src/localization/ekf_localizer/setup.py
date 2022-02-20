"""
Setup for ekf_localizer
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ekf_localizer'],
    package_dir={'': 'src'}
)

setup(**d)

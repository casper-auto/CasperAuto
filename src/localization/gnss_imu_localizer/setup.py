"""
Setup for gnss_imu_localizer
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gnss_imu_localizer'],
    package_dir={'': 'src'}
)

setup(**d)

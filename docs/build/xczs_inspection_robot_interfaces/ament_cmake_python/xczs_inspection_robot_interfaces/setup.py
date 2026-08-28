from setuptools import find_packages
from setuptools import setup

setup(
    name='xczs_inspection_robot_interfaces',
    version='1.0.0',
    packages=find_packages(
        include=('xczs_inspection_robot_interfaces', 'xczs_inspection_robot_interfaces.*')),
)

from setuptools import find_packages
from setuptools import setup

setup(
    name='xczs_inspection_robot_control',
    version='1.0.0',
    packages=find_packages(
        include=('xczs_inspection_robot_control', 'xczs_inspection_robot_control.*')),
)

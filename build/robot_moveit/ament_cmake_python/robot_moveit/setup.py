from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_moveit',
    version='0.3.0',
    packages=find_packages(
        include=('robot_moveit', 'robot_moveit.*')),
)

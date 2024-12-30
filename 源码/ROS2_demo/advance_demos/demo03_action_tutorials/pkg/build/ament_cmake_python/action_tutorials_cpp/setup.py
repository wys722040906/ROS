from setuptools import find_packages
from setuptools import setup

setup(
    name='action_tutorials_cpp',
    version='0.35.1',
    packages=find_packages(
        include=('action_tutorials_cpp', 'action_tutorials_cpp.*')),
)

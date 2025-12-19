from setuptools import find_packages
from setuptools import setup

setup(
    name='solution_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('solution_interfaces', 'solution_interfaces.*')),
)

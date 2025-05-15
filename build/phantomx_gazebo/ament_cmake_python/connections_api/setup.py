from setuptools import find_packages
from setuptools import setup

setup(
    name='connections_api',
    version='0.0.0',
    packages=find_packages(
        include=('connections_api', 'connections_api.*')),
)

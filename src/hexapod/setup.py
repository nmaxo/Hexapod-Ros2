from setuptools import setup
import os
from glob import glob

package_name = 'hexapod'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Hexapod robot control package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_node = hexapod.test_hexapod_direct:main',
            'subscribe_node = hexapod.test_subscribe_msg:main',
            'fsr_test_node  = hexapod.fsr_test_node:main',
            'one_leg_coord = hexapod.one_leg_coordinator:main',
            'leg_controller = hexapod.leg_controller:main',
            'coordinator = hexapod.coordinator:main'
        ],
    },
)

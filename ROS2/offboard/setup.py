from setuptools import setup
from glob import glob
import os

package_name = 'ros2_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto-generated',
    maintainer_email='unknown@example.com',
    description='ROS2 port of offboard pbvs node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pbvs_node = ros2_offboard.pbvs_node:main',
            'offb_node = ros2_offboard.offb_node:main'
        ],
    },
)

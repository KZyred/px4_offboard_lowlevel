from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'px4_offboard_lowlevel'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/controller/*.yaml')),
        (os.path.join('share', package_name), glob('config/exp/*.yaml')),
        (os.path.join('share', package_name), glob('config/sitl/*.yaml')),
        (os.path.join('share', package_name), glob('config/uav_parameters/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KZyred',
    maintainer_email='dung839224@gmail.com',
    description='Low-level control of PX4 Multi-rotor vehicles in Offboard mode (python)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller_node = px4_offboard_lowlevel.controller_node:main",
            "circle_trajectory_node = px4_offboard_lowlevel.trajectory_publishers.circle_trajectory_node:main"
        ],
    },
)

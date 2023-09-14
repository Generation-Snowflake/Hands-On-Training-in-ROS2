from setuptools import setup
import os
from glob import glob
package_name = 'mecanum_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share',package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='com-27x',
    maintainer_email='tanawit_7@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_pose = mecanum_navigation.galactic_goal_pose:main',
            'multi_goal_pose = mecanum_navigation.multi_waypoint:main',
            'move_robot = mecanum_navigation.move_robot_using_twist:main',
        ],
    },
)
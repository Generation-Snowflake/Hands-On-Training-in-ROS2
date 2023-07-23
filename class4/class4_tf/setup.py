from setuptools import setup
import os
from glob import glob

package_name = 'class4_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cstick',
    maintainer_email='cstick987@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assignment1 = class4_tf.assignment1:main',
            'broadcaster = class4_tf.broadcaster:main'
        ],
    },
)

data_files=[

    (os.path.join('share', package_name, 'launch'), 
     glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
],
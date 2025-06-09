from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarmswimros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='saverio.iacoponi@gmail.com',
    description='Bridging between UW Swarm Simulator and ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarmswim = swarmswimros.swarmswim:main',
            'demo1 = swarmswimros.demo1:main',
        ],
    },
)

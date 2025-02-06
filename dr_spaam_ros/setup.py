from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dr_spaam_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@robots.ox.ac.uk',
    description='ROS interface for DR-SPAAM detector',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dr_spaam_ros = dr_spaam_ros.dr_spaam_ros:main'
        ],
    },
)
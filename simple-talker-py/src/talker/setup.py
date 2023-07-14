# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

from setuptools import setup
import os
from glob import glob
package_name = 'talker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boschrexroth',
    maintainer_email="github@boschrexroth.com",
    description='The package demonstrate a simple ROS2 talker (publisher)',
    license='MIT Licence',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'publisher = talker.main_minimal_publisher:main',
        ],
},
)

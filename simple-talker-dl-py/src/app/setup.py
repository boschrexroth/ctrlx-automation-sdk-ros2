# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

from setuptools import setup

package_name = 'app'

setup(
    name=package_name,
    version='2.2.0',
    packages=[package_name, 'ctrlx_datalayer', 'ros2'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boschrexroth',
    maintainer_email='github@boschrexroth.com',
    description='Reads avalue from the ctrlX Data Layer and publishes it as ROS 2 message',
    license='MIT Licence',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'app = app.main:main',
        ],
},
)

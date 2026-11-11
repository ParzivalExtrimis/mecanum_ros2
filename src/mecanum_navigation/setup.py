# Copyright 2026 Aryan Hegde
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mecanum_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        # The maps, their keepout masks and zones, and any pose graph present (the .posegraph
        # and .data files are not in git; see maps/.gitignore).
        (os.path.join('share', package_name, 'maps'),
         [f for f in glob('maps/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hegde-aryan',
    maintainer_email='claude.raseksha@gmail.com',
    description='Nav2 configuration, maps and keepout masks for the mecanum base.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'range_relay = mecanum_navigation.range_relay:main',
            'cloud_thinner = mecanum_navigation.cloud_thinner:main',
        ],
    },
)

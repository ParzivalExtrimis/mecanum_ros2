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

import os

from setuptools import find_packages, setup

package_name = 'mecanum_panel'


def web_files():
    """Install web/ as it is, keeping its folders."""
    out = []
    for folder, _, files in os.walk('web'):
        if files:
            out.append((os.path.join('share', package_name, folder),
                        [os.path.join(folder, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        *web_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hegde-aryan',
    maintainer_email='claude.raseksha@gmail.com',
    description='Web pages of the booth demo: the robot control panel, the TV and a '
                'development dashboard, with their static file server.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panel_server = mecanum_panel.panel_server:main',
        ],
    },
)

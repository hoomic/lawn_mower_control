from setuptools import setup

import os
from glob import glob

package_name = 'lawn_mower_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhooi',
    maintainer_email='mhooi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'test_stereo_control.py = lawn_mower_control.test_stereo_control:main',
          'stereo_mapper = lawn_mower_control.stereo_mapper:main',
          'manual_control = lawn_mower_control.manual_control:main',
          'zone_mapper = lawn_mower_control.zone_mapper:main',
          'path_planner = lawn_mower_control.path_planner:main',
          'navigation = lawn_mower_control.navigation:main',
          'pose_publisher = lawn_mower_control.pose_publisher:main',
        ],
    },
)

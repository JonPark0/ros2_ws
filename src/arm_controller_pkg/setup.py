import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scr',
    maintainer_email='scr@todo.todo',
    description='ARM controller package',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'cargo_manager_node = arm_controller_pkg.cargo_manager_node:main',
            'gripper_node = arm_controller_pkg.gripper_node:main',
            'amr_robot_node = arm_controller_pkg.amr_robot_node:main',
            'manual_command_node = arm_controller_pkg.manual_command_node:main',
        ],
    },
)

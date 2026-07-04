from glob import glob

from setuptools import find_packages, setup


package_name = 'eai_task_server'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/orders', glob('orders/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='marco@example.com',
    description='Simple task publisher for EAI scenarios.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_publisher = eai_task_server.task_publisher:main',
            'task_listener = eai_task_server.task_listener:main',
            'task_complexity_publisher = eai_task_server.task_complexity_publisher:main',
            'entry_server_0702 = eai_task_server.entry_server_0702:main',
            'beginner_server_0702 = eai_task_server.beginner_server_0702:main',
            'manual_order_server = eai_task_server.manual_order_server:main',
        ],
    },
)

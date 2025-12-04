from setuptools import setup

package_name = 'toy_robot_simulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viki',
    maintainer_email='you@example.com',
    description='Toy robot simulator for ROS2 Jazzy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = toy_robot_simulator.robot_node:main',
            'command_client = toy_robot_simulator.command_client:main',
        ],
    },
)


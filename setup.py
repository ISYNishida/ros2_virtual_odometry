from setuptools import setup

package_name = 'virtual_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='your@email.com',
    description='Virtual odometry node for ROS2 (cmd_vel → odom).',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_odometry_node = virtual_odometry.virtual_odometry_node:main',
            'odom_analyzer_node = virtual_odometry.odom_analyzer_node:main',
        ],
    },
)


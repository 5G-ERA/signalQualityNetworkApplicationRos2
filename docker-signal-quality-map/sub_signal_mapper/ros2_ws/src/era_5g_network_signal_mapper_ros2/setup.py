from setuptools import setup
import os
from glob import glob

package_name = 'era_5g_network_signal_mapper_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m',
    maintainer_email='m@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "signal_mapper = era_5g_network_signal_mapper_ros2.signal_mapper:main",
            "sub_signal_mapper = era_5g_network_signal_mapper_ros2.sub_signal_mapper:main",
            "costmap_translate = era_5g_network_signal_mapper_ros2.costmap_translate:main",
            "colour_pub = era_5g_network_signal_mapper_ros2.colour_pub:main",
            "publisher = era_5g_network_signal_mapper_ros2.publisher:main",
            "get_data = era_5g_network_signal_mapper_ros2.get_data:main",
            "current_color = era_5g_network_signal_mapper_ros2.current_color:main",
            "colours_red_green_yellow = era_5g_network_signal_mapper_ros2.colours_red_green_yellow:main",
            "colours_multy = era_5g_network_signal_mapper_ros2.colours_multy:main",
            "current_color_confidence = era_5g_network_signal_mapper_ros2.current_color_confidence:main"
        ],
    },
)

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('era_5g_network_signal_mapper_ros2'),
        'config',
        'params.yaml'
        )

    # Publish a pcl2 around the robot position at time t.
    signal_mapper_node = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="signal_mapper",
        name="pcl2_pub_example",
        parameters=[config]
    )

    # Publish a concatenated (historical) pcl2 from the signal_mapper publisher pcl2.
    sub_signal_mapper_node = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="sub_signal_mapper",
        name="pcl2_semantic_mapper",
        parameters=[config]
    )

    # Publish a concatenated (historical) pcl2 from the signal_mapper publisher pcl2.
    costmap_translate_node = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="sub_signal_mapper",
        name="pcl2_to_costmap",
        parameters=[config]
    )

    # Spin up semantic map server.
    semantic_map_service_server_node = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="service_server_semantic_map",
        name="semantic_map_server",
        parameters=[config]
    )

    # Spin up semantic map server.
    switchover_server = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="switchover_service_server",
        name="swithover",
        parameters=[config]
    )
    
    ld.add_action(signal_mapper_node)
    ld.add_action(sub_signal_mapper_node)
    ld.add_action(costmap_translate_node)
    ld.add_action(semantic_map_service_server_node)
    ld.add_action(switchover_server)

    return ld

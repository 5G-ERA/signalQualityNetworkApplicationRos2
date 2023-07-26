#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Publish a pcl2 around the robot position at time t.
    signal_mapper_node = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="signal_mapper",
        name="pcl2_pub_example",
        parameters=[
            {'base_link':'robot/base_link'},
            {'semantic_map_frame':'semantic_map'}

        ]
    )

    # Publish a concatenated (historical) pcl2 from the signal_mapper publisher pcl2.
    sub_signal_mapper_node = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="sub_signal_mapper",
        name="pcl2_semantic_mapper",
        parameters=[
            {'map_frame':'map'},
            {'semantic_map_frame':'semantic_map'}

        ]
    )

    # Publish a concatenated (historical) pcl2 from the signal_mapper publisher pcl2.
    costmap_translate_node = Node(
        package="era_5g_network_signal_mapper_ros2",
        executable="sub_signal_mapper",
        name="pcl2_to_costmap",
        parameters=[
            {'map_frame':'map'},
            {'map_topic':'/map'},
            {'map_metadata_topic':'/robot/map_metadata'}

        ]
    )
    
    ld.add_action(signal_mapper_node)
    ld.add_action(sub_signal_mapper_node)
    #ld.add_action(costmap_translate_node)

    return ld

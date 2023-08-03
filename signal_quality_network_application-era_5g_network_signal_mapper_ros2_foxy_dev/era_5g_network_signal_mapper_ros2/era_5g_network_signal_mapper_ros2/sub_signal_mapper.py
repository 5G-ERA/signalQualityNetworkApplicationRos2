#!/usr/bin/env python3

import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg as std_msgs
import sensor_msgs_py.point_cloud2 as pcl2
import time
from rclpy.duration import Duration
import era_5g_network_signal_mapper_ros2.tf2_custom as tf2_c
import era_5g_network_signal_mapper_ros2.declare_param as paramm


class SubSignalMapper(Node):
    
        def __init__(self):
            super().__init__("pcl2_semantic_mapper")

            self.tf_buffer = tf2_ros.Buffer() 
            self.listener = tf2_ros.TransformListener(self.tf_buffer,self)

            # Retreive parameters from ENV/ launch/params.yaml file or set default values automaticaly
            self.semantic_map_frame = paramm.param_set_string(self,"my_semantic_map_frame", "semantic_map")
            self.map_frame = paramm.param_set_string(self,"my_map_frame", "map")

            # Create publisher of semantic_pcl
            self.semantic_pcl_pub = self.create_publisher(PointCloud2, '/semantic_pcl',1)

            # Historical pcl2
            merged_cloud = []            

            # Function to construct a pointcloud2 object
            def construct_pointCloud2(points):
                
                # ROS DATATYPE 
                ros_dtype = PointField.FLOAT32
                
                # The PointCloud2 message also has a header which specifies which 
                # coordinate frame it is represented in.
                header = std_msgs.Header()

                # The fields specify what the bytes represents. The first 4 bytes 
                # represents the x-coordinate, the next 4 the y-coordinate, etc.
                fields = [
                    PointField(name='x', offset=0, datatype=ros_dtype, count=1),
                    PointField(name='y', offset=4, datatype=ros_dtype, count=1),
                    PointField(name='z', offset=8, datatype=ros_dtype, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
                ]
                pcl_msg = pcl2.create_cloud(header, fields, points)
                pcl_msg.header.stamp = self.get_clock().now().to_msg()
                pcl_msg.header.frame_id = self.map_frame
                return pcl_msg
            

            def sub_callback(pcl):   
                
                self.get_logger().info(f'1', once=True)

                source_frame = self.map_frame
                target_frame = self.semantic_map_frame
                timeout = Duration(seconds=10)  # Create a duration of 4 seconds                

                transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time(), timeout)
                self.get_logger().info(f'2', once=True) 

                try:
                    transformed_cloud = tf2_c.do_transform_cloud(pcl, transform)
                except Exception as e:
                    pass

                # Get the points from the pcl2.

                gen = pcl2.read_points(transformed_cloud, skip_nans=True)
                int_data = list(gen)
                
                temp_list = []
                
                for element in int_data:
                    mini = []
                    for x in element:
                        
                        mini.append(x)
                    temp_list.append(mini)

                # Append latest pointcloud to merged_cloud
                for y in temp_list:
                    merged_cloud.append(y)

                # Recreate the merged pointcloud
                map_pcl = construct_pointCloud2(merged_cloud) 
                self.semantic_pcl_pub.publish(map_pcl)
                self.get_logger().info("Publishing map_pcl", once=True)

                
            self.semantic_pcl_sub = self.create_subscription(PointCloud2, "/current_semantic_pcl",sub_callback, qos_profile=1)


def main(args=None):
    rclpy.init(args=args)
    node = SubSignalMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

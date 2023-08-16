#!/usr/bin/env python3

import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg as std_msgs
import sensor_msgs_py.point_cloud2 as pcl2
from rclpy.duration import Duration
import era_5g_network_signal_mapper_ros2.tf2_custom as tf2_c
import era_5g_network_signal_mapper_ros2.declare_param as paramm


class SubSignalMapper(Node):
    
        def __init__(self):
            super().__init__("pcl2_semantic_mapper")
            # Declare a tf_buffer which will store known frames and offers a ROS service, "tf_frames",
            # which responds to client requests with a response containing a tf2_msgs
            self.tf_buffer = tf2_ros.Buffer() 
            # Here, we create a tf2_ros.TransformListener object. Once the listener is created,
            # it starts receiving tf2 transformations over the wire, and buffers them for up to "timeout" declared "sub_callback(pcl)".
            self.listener = tf2_ros.TransformListener(self.tf_buffer,self)

            # Retreive parameters from param / launch or params.yaml file / ENV or set default values automaticaly
            self.get_logger().info("Retriving params sub_signal_mapper", once=True)
            self.map_frame = paramm.param_set_string(self,'my_map_frame', 'map')
            self.semantic_map_frame = paramm.param_set_string(self,'my_semantic_map_frame', 'semantic_map')

            self.semantic_pcl_pub = self.create_publisher(PointCloud2, '/semantic_pcl',10)

            # Historical pcl2
            merged_cloud = []
            
            # Function to construct a pointcloud2 object 
            # which will represent the collections of current clouds 
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
            
            # Creating sub_callback which will receive current cloud of the robot "pcl"
            def sub_callback(pcl):
                
                # the amount of time to wait until fail receiving both frames
                timeout = Duration(seconds=10)
                # Waiting until transfrom of map_frame and semantic_map_frame is available
                transform = self.tf_buffer.lookup_transform('map', 'semantic_map', rclpy.time.Time(), timeout)
                
                # Atempt to transform frame "map" to "semantic_map" and add cloud received from "pcl"
                try:
                    transformed_cloud = tf2_c.do_transform_cloud(pcl, transform)

                except Exception as ex:
                    self.get_logger().info(str(ex))

                # Get the points from the pcl2.
                self.get_logger().info("Publishing transformed_cloud", once=True)
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

                # Publishing a map created from collection of current semantic cloud of the robot
                # Rate of pubication is rate of "semantic_pcl_sub" minus time of waiting 
                # frames for transformation from "self.tf_buffer.lookup_transform...."
                self.semantic_pcl_pub.publish(map_pcl)

            # Subscribing to "/current_semantic_pcl" for receiving current semantic cloud of the robot
            self.semantic_pcl_sub = self.create_subscription(PointCloud2, "/current_semantic_pcl",sub_callback, qos_profile=1,)

def main():
    rclpy.init()
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

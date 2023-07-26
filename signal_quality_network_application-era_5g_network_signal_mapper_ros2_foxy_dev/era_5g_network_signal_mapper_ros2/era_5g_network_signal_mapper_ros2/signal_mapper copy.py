#!/usr/bin/env python3
import numpy as np
import rclpy
import std_msgs.msg as std_msgs
import struct
import tf2_ros
import sensor_msgs_py.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class PCL2PublisherNode(Node):

    def __init__(self):
        
        super().__init__("pcl2_pub_example")

        # Declare parameters
        self.declare_parameter("base_link", rclpy.Parameter.Type.STRING)
        self.declare_parameter("semantic_map_frame", rclpy.Parameter.Type.STRING)

        # Retreive parameters from launch/yaml file 
        self.robot_base_frame = self.get_parameter("base_link").get_parameter_value().string_value
        self.robot_base_frame_param = self.get_parameter("base_link")._name

        self.semantic_map_frame = self.get_parameter("semantic_map_frame").get_parameter_value().string_value

        #self.tf_broadcaster = TransformBroadcaster(self)

        global r, g, b, height, lenght, lamba
        r = 124 # 
        g = 252 #
        b = 0   #

        height = 0.3
        lenght = 0.3
        lamba = 0.05

        # Log parameters
        self.get_logger().info("Parameter " + self.robot_base_frame_param + " has value " +  self.robot_base_frame)

        # ROS node definition
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.current_pcl_pub = self.create_publisher(PointCloud2, '/current_semantic_pcl', 10)
        
        self.pcl_colour_subscriber = self.create_subscription(String, "/pcl_colour",self.signal_color_callback,1)
        self.get_logger().info("Initializing semantic pcl2 mapper...")

        #Create a timer that will publish every timer_period (constant defined bellow) pcl_msg and send tf_broadcaster.sendTransform
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pointcloud_while_broadcasting)

    
    # Set pcl colour callback
    def signal_color_callback(self, msg):
        self.get_logger().info(msg.data)
        global r, g, b
        if msg.data == "GREEN":
            r = 124
            g = 252
            b = 0
        elif msg.data == "RED":
            r = 255
            g = 0
            b = 0
        elif msg.data == "BLUE":
            r = 128
            g = 0
            b = 0

                		
    def publish_pointcloud_while_broadcasting(self):
            
            t = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.robot_base_frame  # self.robot_base_frame 
            t.child_frame_id = self.semantic_map_frame  # self.semantic_map_frame
            # Robot only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            # For the same reason, robot can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            #q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

            ##############
            '''
            t_fix = TransformStamped()
            t_fix.header.stamp = self.get_clock().now().to_msg()
            t_fix.header.frame_id = self.semantic_map_frame
            t_fix.child_frame_id = 'map'
            # Robot only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            t_fix.transform.translation.x = 0.0
            t_fix.transform.translation.y = 0.0
            t_fix.transform.translation.z = 0.0

            # For the same reason, robot can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            #q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
            t_fix.transform.rotation.x = 0.0
            t_fix.transform.rotation.y = 0.0
            t_fix.transform.rotation.z = 0.0
            t_fix.transform.rotation.w = 1.0

            # Send the transformation
            self.tf_broadcaster.sendTransform(t_fix)
            '''


            self.create_simple_pointcloud()


    def create_simple_pointcloud(self):

        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
        print(rgb)

        cloud_points = []
        for x in np.arange(0,height,lamba):
            for y in np.arange(-lenght,lenght,lamba):
                cloud_points.append([x, y, 0.0, rgb])
                cloud_points.append([-x, y, 0.0, rgb])

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

        pcl_msg = pcl2.create_cloud(header, fields, cloud_points)
        pcl_msg.header.stamp = self.get_clock().now().to_msg()
        pcl_msg.header.frame_id = self.semantic_map_frame
        self.current_pcl_pub.publish(pcl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PCL2PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

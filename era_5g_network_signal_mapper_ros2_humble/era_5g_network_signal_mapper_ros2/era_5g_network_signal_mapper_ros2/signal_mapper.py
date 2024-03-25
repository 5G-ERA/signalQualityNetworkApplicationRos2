#!/usr/bin/env python3

import rclpy
import numpy as np
import std_msgs.msg as std_msgs
import struct
import tf2_ros
import sensor_msgs_py.point_cloud2 as pcl2
import era_5g_network_signal_mapper_ros2.declare_param as paramm
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
import time


class FramePublisher(Node):

    def __init__(self):
        super().__init__('robot_tf2_frame_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Retreive parameters from param / launch or params.yaml file / ENV or set default values automaticaly
        self.get_logger().info('Retriving params signal_mapper', once=True)
        self.robot_base_frame = paramm.param_set_string(self,'my_base_link', 'base_footprint')
        self.map_frame = paramm.param_set_string(self,'my_map_frame', 'map')
        self.semantic_map_frame = paramm.param_set_string(self,'my_semantic_map_frame', 'semantic_map')
        self.topic_amcl = paramm.param_set_string(self,'robot_topic_position', '/amcl_pose')
        
        # Variables to compose a colour for representing pointcolour
        global r, g, b, confidence, amcl_pose_msg
        r = 124 # 124
        g = 252 # 252
        b = 0   # 0
        confidence = 0.0

        # Boundingbox of the pcl2 publish around the robot
        global height, lenght, lamba, ka

        ka = 0.0
        height = 0.3
        lenght = 0.3
        lamba = 0.05

        # Creating publisher for which will publish cloud formed from pointclouds at current position of robot
        self.current_pcl_pub = self.create_publisher(PointCloud2, '/current_semantic_pcl', 10)
        # Creating subscriber which will receive information of signal strength as colour   
        self.pcl_colour_subscriber = self.create_subscription(String, "/pcl_colour",self.signal_color_callback,1)

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                self.topic_amcl,
                                                self._amclPoseCallback,
                                                amcl_pose_qos)
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        #time.sleep(2)
        #self.timer = self.create_timer(1.0, self.on_timer)
    # 
    def on_timer(self):
        try:
            self.get_logger().info('Sending Position of robot', once=True)
            #global amcl_pose_msg
            self.send_transformation_of_frames2(self.amcl_pose_msg)
            #self.create_simple_pointcloud()

        except TransformException as ex:
            self.get_logger().info(str(ex))

    def _amclPoseCallback(self, msg):
        #global amcl_pose_msg
        self.amcl_pose_msg = msg
        self.initial_pose_received = True
        self.send_transformation_of_frames2(self.amcl_pose_msg)
        #self.create_simple_pointcloud()
        #return

    # Transform frame received from "lookup_transform"; in our case we transform robot_base_frame to map_frame
   
    def send_transformation_of_frames2(self,current_pose:PoseWithCovarianceStamped):
            
            t = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.semantic_map_frame
            # Robot only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            t.transform.translation.x =  current_pose._pose.pose.position.x
            t.transform.translation.y = current_pose._pose.pose.position.y
            t.transform.translation.z = current_pose._pose.pose.position.z

            # For the same reason, robot can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            t.transform.rotation.x = current_pose._pose.pose.orientation.x
            t.transform.rotation.y = current_pose._pose.pose.orientation.y
            t.transform.rotation.z = current_pose._pose.pose.orientation.z
            t.transform.rotation.w = 1.0

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

            self.create_simple_pointcloud()

    
    def create_simple_pointcloud(self, point_size=2):
            global confidence
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            confidence = confidence + 1.0
            if (confidence > 99):
                confidence = 0.0
            # print(confidence)
            cloud_points = []
            # Add a single point at the center of the point cloud with the specified size
            cloud_points.append([0.0, 0.0, 0.0, rgb, confidence, point_size]) #
            # ROS DATATYPE
            ros_dtype = PointField.FLOAT32
            # The point cloud also has a header specifying which coordinate frame it's represented in
            header = std_msgs.Header()
            # The fields specify what the bytes represent. The first 4 bytes represent the x-coordinate,
            # the next 4 the y-coordinate, etc.
            fields = [
                PointField(name='x', offset=0, datatype=ros_dtype, count=1),
                PointField(name='y', offset=4, datatype=ros_dtype, count=1),
                PointField(name='z', offset=8, datatype=ros_dtype, count=1),
                # The 4th set of bytes represents the color of the point cloud
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='confidence', offset=16, datatype=PointField.FLOAT32, count=1),
                # Add a field for the point size
                PointField(name='size', offset=20, datatype=PointField.FLOAT32, count=1)
            ]
            # Create a point cloud and store it in pcl_msg
            pcl_msg = pcl2.create_cloud(header, fields, cloud_points)
            pcl_msg.header.stamp = self.get_clock().now().to_msg()
            pcl_msg.header.frame_id = self.semantic_map_frame
            # Publish the created point cloud at the current robot position
            self.current_pcl_pub.publish(pcl_msg)

    # Set pcl colour callback; setting values of "r" "g" "b" accordingly to message received 
    # from "pcl_colour_subscriber" which represent signal strength
    def signal_color_callback2(self, msg):


        self.get_logger().info(str(msg.data) + " COLOR REQUESTED TO CHANGE!!!")

        global r, g, b
        if msg.data == "GREEN":
            self.get_logger().info("CHANGE TO GREEN")
            r = 255
            g = 255
            b = 0
            self.send_transformation_of_frames2(self.amcl_pose_msg)
            #self.create_simple_pointcloud()
        elif msg.data == "RED":
            self.get_logger().info("CHANGE TO RED")
            r = 255
            g = 0
            b = 0
            self.send_transformation_of_frames2(self.amcl_pose_msg)
            #self.create_simple_pointcloud()
        elif msg.data == "BLUE":
            self.get_logger().info("CHANGE TO BLUE")
            r = 0
            g = 0
            b = 255
            self.send_transformation_of_frames2(self.amcl_pose_msg)
            #self.create_simple_pointcloud()
        elif msg.data == "YELLOW":
            self.get_logger().info("CHANGE TO YELLOW")
            r = 124
            g = 252
            b = 0
            

            self.send_transformation_of_frames2(self.amcl_pose_msg)
            #self.create_simple_pointcloud()
        elif msg.data == "ORANGE":
            self.get_logger().info("CHANGE TO ORANGE")
            r = 255
            g = 192
            b = 0
            self.send_transformation_of_frames2(self.amcl_pose_msg)
            #self.create_simple_pointcloud()

    # Set pcl colour callback; setting values of "r" "g" "b" accordingly to message received 
    # from "pcl_colour_subscriber" which represent signal strength
    def signal_color_callback(self, msg):

        self.get_logger().info(str(msg.data) + " COLOR REQUESTED TO CHANGE!!!")

        global r, g, b
            
        if msg.data == "GREEN": #colour code green 1: 65535.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 0
            g = 255
            b = 255
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "YELLOW": #colour code yellow 2: 8190976.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 124
            g = 252
            b = 0
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "RED":  #colour code red 3: 7902720.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 120 #blue 
            g = 150
            b = 0
            self.send_transformation_of_frames2(self.amcl_pose_msg)
            
        if msg.data == "BLUE2":#blue
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 150 #blue 
            g = 0
            b = 200
            self.send_transformation_of_frames2(self.amcl_pose_msg)            
        if msg.data == "BLUE": #colour code blue 4: 8532735.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 130 #blue 
            g = 50
            b = 255 
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "PURPLE": #colour code purple 5: 16711935.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 255
            g = 0
            b = 255
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "PINK": #colour code pink 6: 7864520.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 120 #pink 
            g = 0
            b = 200
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "TEAL": #colour code teal 7: 8421504.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 128
            g = 128
            b = 128
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "ORANGE": #colour code orange 8: 7910400.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 120 #blue 
            g = 180
            b = 0    
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "CYAN": #colour code cyan 9: 16777215.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 255
            g = 255
            b = 255
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "MAROON": #colour code maroon 10: 3294790.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 50 #maroon 
            g = 70
            b = 70
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "LIME": #colour code lime 11: 5926430.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 90 #lime 
            g = 110
            b = 30 
            self.send_transformation_of_frames2(self.amcl_pose_msg)        
        if msg.data == "BROWN": #colour code brown 12: 4633620.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 70 #brown 
            g = 180
            b = 20
            self.send_transformation_of_frames2(self.amcl_pose_msg)        
        if msg.data == "BLACK": #colour code black 13: 8388608.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 128
            g = 0
            b = 0
            self.send_transformation_of_frames2(self.amcl_pose_msg)

        if msg.data == "RED_DARK": #colour code red_dark 14: 16711680.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 255
            g = 0
            b = 0
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "BLUE_DARK": #colour code blue_dark 15: 8388736.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 128
            g = 0
            b = 128
            self.send_transformation_of_frames2(self.amcl_pose_msg)
        if msg.data == "GREEN_DARK": #colour code green_dark 16: 32896.0
            self.get_logger().info(f"CHANGE TO {msg.data}")
            r = 0
            g = 128
            b = 128
            self.send_transformation_of_frames2(self.amcl_pose_msg)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
import numpy as np
import std_msgs.msg as std_msgs
import struct
import tf2_ros
import sensor_msgs_py.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
import era_5g_network_signal_mapper_ros2.declare_param as paramm
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


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
        
        # Variables to compose a colour for representing pointcolour
        global r, g, b, confidence
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
                                                '/amcl_pose',
                                                self._amclPoseCallback,
                                                amcl_pose_qos)
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.on_timer)
    # 
    def on_timer(self):
            try:
                #self.model_pose_sub
                print("here!!here")
                global pcl2_message
                #print(d)
                self.send_transformation_of_frames2(pcl2_message)
                self.create_simple_pointcloud()

            except TransformException as ex:
                    self.get_logger().info(str(ex))

    def _amclPoseCallback(self, msg):
        global pcl2_message
        pcl2_message = msg
        self.initial_pose_received = True
        return

    # Transform frame received from "lookup_transform"; in our case we transform robot_base_frame to map_frame
   
    def send_transformation_of_frames2(self,trans2:PoseWithCovarianceStamped):
            
            t = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.semantic_map_frame
            # Robot only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            t.transform.translation.x =  trans2._pose.pose.position.x #trans.transform.translation.x
            t.transform.translation.y = trans2._pose.pose.position.y #trans.transform.translation.y
            t.transform.translation.z = trans2._pose.pose.position.z #trans.transform.translation.z

            # For the same reason, robot can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            t.transform.rotation.x = trans2._pose.pose.orientation.x #trans.transform.rotation.x
            t.transform.rotation.y = trans2._pose.pose.orientation.y#trans.transform.rotation.y
            t.transform.rotation.z = trans2._pose.pose.orientation.z#trans.transform.rotation.z
            t.transform.rotation.w = 1.0

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

    # Set pcl colour callback; setting values of "r" "g" "b" accordingly to message received 
    # from "pcl_colour_subscriber" which represent signal strength
    def signal_color_callback(self, msg):


        self.get_logger().info(str(msg.data) + " COLOR REQUESTED TO CHANGE!!!")

        global r, g, b
        if msg.data == "GREEN":
            self.get_logger().info("CHANGE TO GREEN")
            r = 124
            g = 252
            b = 0
        elif msg.data == "RED":
            self.get_logger().info("CHANGE TO RED")
            r = 255
            g = 0
            b = 0
        elif msg.data == "BLUE":
            self.get_logger().info("CHANGE TO BLUE")
            r = 0
            g = 0
            b = 255
        elif msg.data == "YELLOW":
            self.get_logger().info("CHANGE TO YELLOW")
            r = 255
            g = 255
            b = 0
        elif msg.data == "ORANGE":
            self.get_logger().info("CHANGE TO ORANGE")
            r = 255
            g = 165
            b = 0

    def create_simple_pointcloud(self):
        global confidence

        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
        confidence = confidence + 1.0
        if (confidence > 99):
            confidence = 0.0
        print(confidence)

        cloud_points = []

        # Creating point clouds of the current position of the robot 
        for x in np.arange(0,height,lamba):
            for y in np.arange(-lenght,lenght,lamba):
                #cloud_points.append([x, y, 0.0, rgb])
                #cloud_points.append([-x, y, 0.0, rgb]) 
                
                cloud_points.append([x, y, 0.0, rgb, confidence])
                cloud_points.append([-x, y, 0.0, rgb, confidence])       


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
            # The 4th set of bytes represent colour of pointcloud
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='confidence', offset=16, datatype=PointField.FLOAT32, count=1)
        ]
        # creating a cloud and store in pcl_msg
        pcl_msg = pcl2.create_cloud(header, fields, cloud_points)
        pcl_msg.header.stamp = self.get_clock().now().to_msg()
        pcl_msg.header.frame_id = self.semantic_map_frame

        # Publishing cloud created at the current possition of the robot
        self.current_pcl_pub.publish(pcl_msg)


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

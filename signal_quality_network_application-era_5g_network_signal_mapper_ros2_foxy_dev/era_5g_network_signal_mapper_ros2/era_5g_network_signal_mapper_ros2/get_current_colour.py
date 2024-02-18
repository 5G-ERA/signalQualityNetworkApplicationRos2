#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pcl2
from std_msgs.msg import Float32
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
import time

#
# This node will create confidence layer
#
class get_current_colour(Node):

    def __init__(self):
        super().__init__('pub_color')

        #global x, y, z, pcl_global

                # Boundingbox of the pcl2 publish around the robot
        #global height, lenght, lamba, ka

        #ka = 0.0
        #height = 0.3
        #lenght = 0.3
        #lamba = 0.05
        
        # Initialize variables
        self.center_x = 0.0
        self.center_y = 0.0
        self.z = 0.0
        self.pcl_global = None
        self.height = 0.3
        self.length = 0.3
        self.lamba = 0.05

        
        self.pub = self.create_publisher(String, '/current_color', 10)
        
               
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                '/amcl_pose',
                                                self._amclPoseCallback,
                                                amcl_pose_qos)
        
        # Subscribing to "/current_semantic_pcl" for receiving current semantic cloud of the robot
        self.semantic_pcl_sub = self.create_subscription(PointCloud2, "/semantic_pcl",self.pcl2_callback, qos_profile=1,)
            

    def _amclPoseCallback(self, msg:PoseWithCovarianceStamped):
        #global pcl2_message, x, y, z
        self.pcl2_message = msg

        self.center_x = msg.pose.pose.position.x
        self.center_y = msg.pose.pose.position.y
        #z = msg.pose.pose.position.z        

        self.initial_pose_received = True
        self.debug_func()
        self.get_color_from_historical_pcl()
        return
    
     
    # Creating pcl2_callback which will receive historical pointcloud of the robot "pcl"
    def pcl2_callback(self,pcl:PointCloud2):
        
        #self.get_logger().info(str(pcl) + " Publishing historical pcl", once=True)
        #global pcl_global

        self.pcl_global = pcl        
        

        # TODO: get colour from historical pcl

        # TODO: get confidence from historical pcl

        pass
    

    def get_color_from_historical_pcl(self):
        
        #array[3000 points => (x,y)]
        #foreach point in array
        try:
                

            point_generator = pcl2.read_points(self.pcl_global, field_names=("x", "y", "z"), skip_nans=True)
            # Iterateing each point in pointcloud
            for p_in in point_generator:
                #x, y, z = p_in
                point_x = p_in[0]
                point_y = p_in[1]
                #z = p_in[2]
                #print(f"Coordinates: x={x}, y={y}, z={z}")
                #print(f"Coordinates: x={point_x}, y={point_y}")

                #print(f"Coordinates Center robot: x={self.center_x}, y={self.center_y}")
                

                bounding_box_bottom_x = self.center_x - self.height/2
                bounding_box_top_x = self.center_x + self.height/2

                bounding_box_bottom_y = self.center_y - self.height/2
                bounding_box_top_y = self.center_y + self.height/2
                if( bounding_box_bottom_x <= point_x) and (point_x >= bounding_box_top_x):
                    if( bounding_box_bottom_y <= point_y) and (point_y >= bounding_box_top_y):
                        print("point inside bounding box")
                        print(f"Coordinates: x={str(point_x)}, y={str(point_y)}")
                        print(f"Coordinates Center robot: x={str(self.center_x)}, y={str(self.center_y)}")

                # if bounding_box_bottom.x  < point.x  and point.x > bounding_box_top.x 
                # if bounding_box_bottom.y  < point.y  and point.x > bounding_box_top.y  
                #self.height = 0.3
                #self.length = 0.3
        except:
            pass
        





        # check if is inside in bounding box with point




        # if point.x >

        # TODO: if (1 or more points == red and pose < bounding box)  => Red
                # esle if (no red and 1 or more points == yellow) => Yellow
                # else if (no red and no yellow and one or more green ) => Green
                # else => no colour
        # TODO: 
        # TODO: 
        # TODO: 
        pass

    def debug_func(self):
        #global pcl_global, x, y, height, lenght, lamba
        
        print("x")
        print( self.center_x)
        print("")
        print("y")
        print(self.center_y)
        print("")
        print( "height")
        print(self.height)
        print("")
        print("lenght")
        print(self.length)
        print("")
        print("lamba")
        print(self.lamba)
        print("")
        print("pcl_global:")
        #print(self.pcl_global)
        pass

def main():
    rclpy.init()
    node = get_current_colour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

#
# This node will create confidence layer
#
class get_data_from_pcl2(Node):

    def __init__(self):
        super().__init__('pub_color')

        global x, y, z, pcl_global, height, lenght
        
        self.pub = self.create_publisher(String, '/current_color', 10)
        self.pub = self.create_publisher(Float32, '/current_confidence', 10)
       
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
        self.semantic_pcl_sub = self.create_subscription(PointCloud2, "/current_semantic_pcl",self.pcl2_callback, qos_profile=1,)

    def _amclPoseCallback(self, msg:PoseWithCovarianceStamped):
        global pcl2_message, x, y, z
        pcl2_message = msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        #z = msg.pose.pose.position.z        

        self.initial_pose_received = True
        return
    
     
    # Creating pcl2_callback which will receive historical pointcloud of the robot "pcl"
    def pcl2_callback(self,pcl:PointCloud2):
        
        #self.get_logger().info(str(pcl) + " Publishing historical pcl", once=True)
        global pcl_global

        pcl_global = pcl

        
        

        # TODO: get colour from historical pcl

        # TODO: get confidence from historical pcl

        pass

    def get_color_from_historical_pcl():
        pass

    def get_confidence_from_historical_pcl():
        pass
    '''    
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
    '''

def main():
    rclpy.init()
    node = get_data_from_pcl2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

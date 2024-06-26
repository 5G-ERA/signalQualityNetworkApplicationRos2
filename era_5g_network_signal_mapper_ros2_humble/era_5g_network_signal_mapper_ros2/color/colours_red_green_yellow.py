#!/usr/bin/env python3

'''
Ros2 Node that will publish under the topic /pcl_colour from user input keyboard as number in result will be published string name of the colour only Red, Green and Yellow;
'''


import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class cloud_coloring(Node):

    def __init__(self):
        super().__init__('pub_color')
        
        self.pub = self.create_publisher(String, '/pcl_colour', 10)
        while True:
                
                self.get_logger().info("\n- Publish pcl colour -", once=True)
                self.get_logger().info("   1. Publish RED", once=True)
                self.get_logger().info("   2. Publish GREEN", once=True)
                self.get_logger().info("   4. Publish YELLOW -", once=True)
                self.get_logger().info("   99. Exit", once=True)
                colour = input('Input: ')

                if str(colour) == "1":
                    msg = String()
                    msg.data = "RED"
                    print("RED")
                    self.pub.publish(msg)
                    self.get_logger().info("Publish RED", once=True)

                elif str(colour) == "2":
                    msg = String()
                    msg.data = "GREEN"
                    print("GREEN")
                    self.pub.publish(msg)
                    self.get_logger().info("Publish GREEN", once=True)                    

                elif str(colour) == "3":
                    print("YELLOW")
                    msg = String()
                    msg.data = "YELLOW"
                    self.pub.publish(msg)
                    self.get_logger().info("Publish YELLOW", once=True)

                    
                elif str(colour) == "99":
                    exit(0)

def main():
    rclpy.init()
    node = cloud_coloring()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

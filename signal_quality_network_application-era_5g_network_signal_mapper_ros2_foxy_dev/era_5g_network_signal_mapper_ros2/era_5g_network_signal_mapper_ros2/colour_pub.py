#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import os


class cloud_coloring(Node):

    def __init__(self):
        super().__init__('pub_color')

        
        self.pub = self.create_publisher(String, '/pcl_colour', 10)
        while True:
                self.get_logger().info("\n- Publish pcl colour -", once=True)
                self.get_logger().info("   1. Publish RED", once=True)
                self.get_logger().info("   2. Publish GREEN", once=True)
                self.get_logger().info("   3. Publish BLUE -", once=True)
                self.get_logger().info("   4. Publish YELLOW -", once=True)
                self.get_logger().info("   5. Publish ORANGE -", once=True)
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
                    print("BLUE")
                    msg = String()
                    msg.data = "BLUE"
                    self.pub.publish(msg)
                    self.get_logger().info("Publish BLUE", once=True)

                elif str(colour) == "4":
                    print("YELLOW")
                    msg = String()
                    msg.data = "YELLOW"
                    self.pub.publish(msg)
                    self.get_logger().info("Publish YELLOW", once=True)

                elif str(colour) == "5":
                    print("ORANGE")
                    msg = String()
                    msg.data = "ORANGE"
                    self.pub.publish(msg)
                    self.get_logger().info("Publish ORANGE", once=True)
                   

                elif str(colour) == "99":
                    exit(0)
            


        

        

def main():
    rclpy.init()
    node = cloud_coloring()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

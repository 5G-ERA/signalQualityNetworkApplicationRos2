#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class cloud_coloring(Node):

    def __init__(self):
        super().__init__('pub_color')
        
        self.pub = self.create_publisher(String, '/pcl_colour', 10)
        while True:
                
                self.get_logger().info("\n- Publish pcl colour -", once=True)
                self.get_logger().info("   1. Publish BLACK", once=True)
                self.get_logger().info("   2. Publish WHITE", once=True)
                self.get_logger().info("   3. Publish RED", once=True)
                self.get_logger().info("   4. Publish LIME", once=True)
                self.get_logger().info("   5. Publish BLUE", once=True)
                self.get_logger().info("   6. Publish YELLOW", once=True)
                self.get_logger().info("   7. Publish CYAN", once=True)
                self.get_logger().info("   8. Publish MAGENTA", once=True)
                self.get_logger().info("   9. Publish MAROON", once=True)
                self.get_logger().info("   10. Publish OLIVE", once=True)
                self.get_logger().info("   11. Publish GREEN", once=True)
                self.get_logger().info("   12. Publish PURPLE", once=True)
                self.get_logger().info("   13. Publish TEAL", once=True)
                self.get_logger().info("   14. Publish NAVY", once=True)
                self.get_logger().info("   99. Exit", once=True)
                colour = input('Input: ')

                if str(colour) == "1":
                    msg = String()
                    msg.data = "BLACK"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "2":
                    msg = String()
                    msg.data = "WHITE"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "3":
                    msg = String()
                    msg.data = "RED"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "4":
                    msg = String()
                    msg.data = "LIME"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "5":
                    msg = String()
                    msg.data = "BLUE"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "6":
                    msg = String()
                    msg.data = "YELLOW"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "7":
                    msg = String()
                    msg.data = "CYAN"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "8":
                    msg = String()
                    msg.data = "MAGENTA"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "9":
                    msg = String()
                    msg.data = "MAROON"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "10":
                    msg = String()
                    msg.data = "OLIVE"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "11":
                    msg = String()
                    msg.data = "GREEN"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "12":
                    msg = String()
                    msg.data = "PURPLE"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "13":
                    msg = String()
                    msg.data = "TEAL"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "14":
                    msg = String()
                    msg.data = "NAVY"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)
                    
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

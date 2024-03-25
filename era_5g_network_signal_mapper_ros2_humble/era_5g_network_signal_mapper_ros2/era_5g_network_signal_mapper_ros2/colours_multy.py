#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class cloud_coloring(Node):

    def __init__(self):
        super().__init__('pub_color')

        '''
                #colour code green 1: 65535.0
                #colour code yellow 2: 8190976.0
                #colour code red 3: 7902720.0
                #colour code blue 4: 8532735.0
                #colour code purple 5: 16711935.0
                #colour code pink 6: 7864520.0
                #colour code teal 7: 8421504.0
                #colour code orange 8: 7910400.0
                #colour code cyan 9: 16777215.0
                #colour code maroon 10: 3294790.0
                #colour code lime 11: 5926430.0
                #colour code brown 12: 4633620.0
                #colour code black 13: 8388608.0
                #colour code red_dark 14: 16711680.0
                #colour code blue_dark 15: 8388736.0
                #colour code green_dark 16: 32896.0
        #'''
        
        self.pub = self.create_publisher(String, '/pcl_colour', 10)
        while True:
                
                self.get_logger().info("\n- Publish pcl colour -", once=True)       
                self.get_logger().info("   1. Publish GREEN", once=True)            #colour code green 1: 65535.0
                self.get_logger().info("   2. Publish YELLOW", once=True)           #colour code yellow 2: 8190976.0
                self.get_logger().info("   3. Publish RED", once=True)              #colour code red 3: 7902720.0
                self.get_logger().info("   4. Publish BLUE", once=True)             #colour code blue 4: 8532735.0
                self.get_logger().info("   5. Publish PURPLE", once=True)           #colour code purple 5: 16711935.0
                self.get_logger().info("   6. Publish PINK", once=True)             #colour code pink 6: 7864520.0
                self.get_logger().info("   7. Publish TEAL", once=True)             #colour code teal 7: 8421504.0
                self.get_logger().info("   8. Publish ORANGE", once=True)           #colour code orange 8: 7910400.0
                self.get_logger().info("   9. Publish CYAN", once=True)             #colour code cyan 9: 16777215.0
                self.get_logger().info("   10. Publish MAROON", once=True)          #colour code maroon 10: 3294790.0
                self.get_logger().info("   11. Publish LIME", once=True)            #colour code lime 11: 5926430.0
                self.get_logger().info("   12. Publish BROWN", once=True)           #colour code brown 12: 4633620.0
                self.get_logger().info("   13. Publish BLACK", once=True)           #colour code black 13: 8388608.0
                self.get_logger().info("   14. Publish RED_DARK", once=True)        #colour code red_dark 14: 16711680.0
                self.get_logger().info("   15. Publish BLUE_DARK", once=True)       #colour code blue_dark 15: 8388736.0
                self.get_logger().info("   16. Publish GREEN_DARK", once=True)      #colour code green_dark 16: 32896.0
                self.get_logger().info("   99. Exit", once=True)
                colour = input('Input: ')

                '''
                        
                        
                        
                        
                        
                        
                        
                        
                #'''

                if str(colour) == "1": #colour code green 1: 65535.0
                    msg = String()
                    msg.data = "GREEN"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "2": #colour code yellow 2: 8190976.0
                    msg = String()
                    msg.data = "YELLOW"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "3": #colour code red 3: 7902720.0
                    msg = String()
                    msg.data = "RED"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "4": #colour code blue 4: 8532735.0
                    msg = String()
                    msg.data = "BLUE"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "5": #colour code purple 5: 16711935.0
                    msg = String()
                    msg.data = "PURPLE"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "6": #colour code pink 6: 7864520.0
                    msg = String()
                    msg.data = "PINK"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "7": #colour code teal 7: 8421504.0
                    msg = String()
                    msg.data = "TEAL"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "8": #colour code orange 8: 7910400.0
                    msg = String()
                    msg.data = "ORANGE"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "9": #colour code cyan 9: 16777215.0
                    msg = String()
                    msg.data = "CYAN"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "10": #colour code maroon 10: 3294790.0
                    msg = String()
                    msg.data = "MAROON"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "11": #colour code lime 11: 5926430.0
                    msg = String()
                    msg.data = "LIME"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "12": #colour code brown 12: 4633620.0
                    msg = String()
                    msg.data = "BROWN"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "13": #colour code black 13: 8388608.0
                    msg = String()
                    msg.data = "BLACK"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "14": #colour code red_dark 14: 16711680.0
                    msg = String()
                    msg.data = "RED_DARK"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "15": #colour code blue_dark 15: 8388736.0
                    msg = String()
                    msg.data = "BLUE_DARK"
                    print(f"{msg.data}")
                    self.pub.publish(msg)
                    self.get_logger().info(f"Publish {msg.data}", once=True)

                elif str(colour) == "16": #colour code green_dark 16: 32896.0
                    msg = String()
                    msg.data = "GREEN_DARK"
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


'''
                #colour code green 1: 65535.0
                #colour code yellow 2: 8190976.0
                #colour code red 3: 7902720.0
                #colour code blue 4: 8532735.0
                #colour code purple 5: 16711935.0
                #colour code pink 6: 7864520.0
                #colour code teal 7: 8421504.0
                #colour code orange 8: 7910400.0
                #colour code cyan 9: 16777215.0
                #colour code maroon 10: 3294790.0
                #colour code lime 11: 5926430.0
                #colour code brown 12: 4633620.0
                #colour code black 13: 8388608.0
                #colour code red_dark 14: 16711680.0
                #colour code blue_dark 15: 8388736.0
                #colour code green_dark 16: 32896.0

        
                        
                self.get_logger().info("\n- Publish pcl colour -", once=True)
                self.get_logger().info("   1. Publish GREEN", once=True)
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
                self.get_logger().info("   15. Publish PINK", once=True)
                self.get_logger().info("   16. Publish ORANGE", once=True)
                self.get_logger().info("   99. Exit", once=True)
                colour = input('Input: ')
#'''


'''
        if msg.data == "GREEN":#green 1
        if msg.data == "YELLOW":#yellow 2
        if msg.data == "RED":#red 3     
        if msg.data == "BLUE":#blue 4
        if msg.data == "PURPLE":#purple 5
        if msg.data == "PINK":#pink 6
        if msg.data == "TEAL":# teal 7
        if msg.data == "ORANGE":#orange 8
        if msg.data == "CYAN":#cyan 9
        if msg.data == "MAROON":#maroon 10
        if msg.data == "LIME":# lime 11    
        if msg.data == "BROWN":#brown 12
        if msg.data == "BLAK":#black 13
        if msg.data == "RED_DARK":#red_dark 14
        if msg.data == "BLUE_DARK":#blue_dark 15
        if msg.data == "GREEN_DARK":#green_dark 16
#'''
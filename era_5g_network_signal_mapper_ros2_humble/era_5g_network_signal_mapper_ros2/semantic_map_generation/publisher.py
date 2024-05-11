#!/usr/bin/env python3

'''
ROS Node to get router RSPR values and translte into ROS topic of type String for color.
TODO: parameters to be included for flexible change as input from ENV, params.yaml or default values.
'''


import rclpy
from std_msgs.msg import String
from influxdb import InfluxDBClient
from rclpy.node import Node
import time


class influx_coloring(Node):
    def __init__(self):
        super().__init__('influx_color')
        self.pub = self.create_publisher(String, 'pcl_colour', 10)

        #Setup database
        client = InfluxDBClient('192.168.X.XXX', 8086, 'loremipsum', 'loremipsum', 'openwrt')
        connection_success = True
        #if client.switch_database('openwrt'):
            #check if connection with database is successful
        #    connection_success = True
        #else:
        #    client.close()
        #    print("Connection Error")

        def handle_keyboard():
            if connection_success:
                try:                  
                    
                    
                    while True:
                        self.get_logger().info("inside while", once=False)
                        #result = client.query('SELECT mean("RSRP") as rsrp from gsmctl where time < now()-1s;')
                        result = client.query('SELECT last("RSRP") as rsrp from gsmctl;')
                
                
                        signal_strength = "Red"
                        points = result.get_points()
                        for item in points:
                            rsrp = (item['rsrp'])

                        self.get_logger().info("rsrp: "+str(rsrp), once=False)
                        #    rsrq =(item['last_1'])

                        # check signal and printing lowest signal between 2 parameters RSRP and RSRQ
                        # Green Strong Signal
                        # Yellow Good Signal
                        # Orange Poor Signal
                        # Red No Signal
                        signal_strength = String()
                        self.get_logger().info("signal_value: "+ str(rsrp))
                        crsrp=abs(rsrp)
                        red_value=100.0
                        green_value=95.00
                        if  crsrp > red_value:
                            signal_strength.data = "RED"
                        elif  (crsrp <= red_value) and (crsrp >= green_value):
                            signal_strength.data = "YELLOW"
                        elif crsrp < green_value:
                            signal_strength.data = "GREEN"
                        self.get_logger().info("signal_data: "+ signal_strength.data)

                        
                        #rospy.loginfo(signal_strength)
                        self.pub.publish(signal_strength)
                        time.sleep(0.5)
                        #rate.sleep()
                except Exception as e:
                    print(e)
                    self.get_logger().info(e, once=False)
                
        try:
            handle_keyboard()
        except Exception as e:
                self.get_logger().info(e, once=False)


def main():
    rclpy.init()
    node = influx_coloring()
    #rate = node.create_rate(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

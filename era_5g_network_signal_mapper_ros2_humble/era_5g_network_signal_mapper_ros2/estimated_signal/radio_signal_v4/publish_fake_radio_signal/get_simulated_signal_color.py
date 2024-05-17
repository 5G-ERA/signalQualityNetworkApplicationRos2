'''
ROS2 Node that publishes signal color based on its positions
from radio antena signal (full map simulated)
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import struct
import numpy as np
class RadioSignalColorDetector(Node):
    def __init__(self):
        super().__init__('radio_signal_color_detector')
        # Declare robot radius parameter
        self.declare_parameter('robot_radius', 0.1)
        self.robot_radius = self.get_parameter('robot_radius').value
        # Subscribers
        self.create_subscription(PointCloud2, '/radio_antenna_signal', self.pointcloud_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        # Publisher
        #self.color_publisher = self.create_publisher(String, '/current_signal_color', 10)
        self.color_publisher = self.create_publisher(String, '/pcl_colour', 10)
        self.point_data = None
    def pointcloud_callback(self, msg):
        # Extract points from PointCloud2 message
        self.point_data = self.convert_pointcloud2_to_array(msg)
        self.get_logger().info('Received point cloud data.')
    def pose_callback(self, msg):
        if self.point_data is None:
            self.get_logger().info('Waiting for point cloud data...')
            return
        # Get robot's current position
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        # Determine the color at the robot's position
        color = self.get_color_at_position(robot_x, robot_y)
        if color:
            color_str = f'R={color[0]}, G={color[1]}, B={color[2]}'
            r = color[0]
            g = color[1]
            b = color[2]
            print(r)
            color_to_pub = "NONE"
            if (r==0) and (g==255) and (b==0):
                color_to_pub = "GREEN"
            if (r==255) and (g==255) and (b==0):
                color_to_pub = "YELLOW"
            if (r==255) and (g==0) and (b==0):
                color_to_pub = "RED"
            self.get_logger().info(f'Current color at robot position: x= {robot_x} y= {robot_y} ')
            self.color_publisher.publish(String(data=color_to_pub))
        else:
            self.get_logger().info('No color detected at robot position.')
            self.color_publisher.publish(String(data='No signal'))
    def convert_pointcloud2_to_array(self, msg):
        # Convert PointCloud2 message to a list of points with colors
        fmt = 'fffI'  # x, y, z, rgb
        point_step = struct.calcsize(fmt)
        points = []
        for i in range(0, len(msg.data), point_step):
            x, y, z, rgb = struct.unpack_from(fmt, msg.data, i)
            r = (rgb >> 16) & 0xFF
            g = (rgb >> 8) & 0xFF
            b = rgb & 0xFF
            points.append((x, y, z, r, g, b))
        return points
    def get_color_at_position(self, x, y):
        # Find the closest point in the point cloud to the robot's position
        closest_point = None
        min_distance = float('inf')
        for point in self.point_data:
            px, py, pz, r, g, b = point
            distance = np.sqrt((px - x)**2 + (py - y)**2)
            if distance < min_distance and distance <= self.robot_radius:
                min_distance = distance
                closest_point = (r, g, b)

        if closest_point:
            return closest_point  # Returning the color as a tuple (r, g, b)
        return None
def main(args=None):
    rclpy.init(args=args)
    node = RadioSignalColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
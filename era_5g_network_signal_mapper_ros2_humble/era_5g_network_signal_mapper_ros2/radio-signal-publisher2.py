import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.radio_antenna_signal = self.create_publisher(PointCloud2, '/radio_antenna_signal', 10)

        # Parameters for the distances between layers and the location of the antenna source
        self.declare_parameter('green_to_yellow', 1.0)
        self.declare_parameter('yellow_to_red', 2.0)
        self.declare_parameter('antenna_x', 0.0)
        self.declare_parameter('antenna_y', 0.0)

        green_to_yellow = self.get_parameter('green_to_yellow').value
        yellow_to_red = self.get_parameter('yellow_to_red').value
        antenna_x = self.get_parameter('antenna_x').value
        antenna_y = self.get_parameter('antenna_y').value

        # Create the circle points with layers
        points = self.create_colored_circle(green_to_yellow, yellow_to_red, antenna_x, antenna_y)

        # Convert the points to PointCloud2 message
        pointcloud_msg = self.create_pointcloud2(points)

        # Publish the PointCloud2 message
        self.publish_pointcloud2(pointcloud_msg)

    def create_colored_circle(self, green_to_yellow, yellow_to_red, center_x, center_y):
        # Define the layers and their radii
        layers = [
            (0, green_to_yellow, (0, 255, 0)), # Green layer
            (green_to_yellow, green_to_yellow + yellow_to_red, (255, 255, 0)), # Yellow layer
            (green_to_yellow + yellow_to_red, green_to_yellow + 2 * yellow_to_red, (255, 0, 0)) # Red layer
        ]

        points = []
        for inner_radius, outer_radius, color in layers:
            for r in np.arange(inner_radius, outer_radius, 0.1): # Adjust step size as needed
                for theta in np.arange(0, 2 * np.pi, np.pi / 180): # Full circle
                    x = center_x + r * np.cos(theta)
                    y = center_y + r * np.sin(theta)
                    points.append((x, y, 0, *color))
        return points

    def create_pointcloud2(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        point_data = []
        for point in points:
            x, y, z, r, g, b = point
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            point_data.append(struct.pack('fffI', x, y, z, rgb))

        pointcloud2 = PointCloud2()
        pointcloud2.header = header
        pointcloud2.height = 1
        pointcloud2.width = len(points)
        pointcloud2.fields = fields
        pointcloud2.is_bigendian = False
        pointcloud2.point_step = 16
        pointcloud2.row_step = pointcloud2.point_step * pointcloud2.width
        pointcloud2.data = b''.join(point_data)
        pointcloud2.is_dense = True

        return pointcloud2

    def publish_pointcloud2(self, pointcloud_msg):
        self.radio_antenna_signal.publish(pointcloud_msg)
        self.get_logger().info('Publishing PointCloud2 message with colored circle layers.')

def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

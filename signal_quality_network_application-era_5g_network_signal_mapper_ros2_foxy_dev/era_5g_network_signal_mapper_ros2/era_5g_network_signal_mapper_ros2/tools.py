import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class PointCloudSaverNode(Node):
    def __init__(self):
        super().__init__('pointcloud_saver_node')

        self.create_subscription(PointCloud2, '/semantic_pcl', self.pointcloud_callback, 1)
        self.save_service = self.create_service(Trigger, 'save_pointcloud', self.save_pointcloud_callback)
        self.load_and_publish_service = self.create_service(Trigger, 'load_and_publish_pointcloud', self.load_and_publish_pointcloud_callback)
        self.publisher_ = self.create_publisher(PointCloud2, '/loaded_pointcloud', 1)

        self.pointcloud_data = None

    def pointcloud_callback(self, msg):
        # Save the PointCloud2 data when it's received
        self.pointcloud_data = msg

    def save_pointcloud_callback(self, request, response):
        if self.pointcloud_data is not None:
            # Extract point cloud data
            points = pc2.read_points(self.pointcloud_data, field_names=("x", "y", "z", "rgb"), skip_nans=True)

            # Convert points to a list for easy manipulation
            point_list = list(points)

            # Save the PointCloud2 to a text file
            self.save_pointcloud_to_file(point_list, 'pointcloud_data.txt')

            response.success = True
            response.message = 'PointCloud2 saved to file.'
        else:
            response.success = False
            response.message = 'No PointCloud2 data available.'

        return response

    def load_and_publish_pointcloud_callback(self, request, response):
        # Load PointCloud2 data from the file
        loaded_pointcloud_data = self.load_pointcloud_from_file('pointcloud_data.txt')

        if loaded_pointcloud_data is not None:
            # Publish the loaded PointCloud2 data
            self.publisher_.publish(loaded_pointcloud_data)

            response.success = True
            response.message = 'PointCloud2 loaded and published.'
        else:
            response.success = False
            response.message = 'Failed to load PointCloud2 data from file.'

        return response

    def save_pointcloud_to_file(self, point_list, filename):
        with open(filename, 'w') as file:
            # Write the header
            file.write("# x y z r g b\n")

            # Write each point to the file
            for point in point_list:
                x, y, z, rgb = point
                r = (rgb >> 16) & 0xff
                g = (rgb >> 8) & 0xff
                b = rgb & 0xff
                file.write(f"{x} {y} {z} {r} {g} {b}\n")

    def load_pointcloud_from_file(self, filename):
        try:
            # Read PointCloud2 data from the file
            loaded_points = np.loadtxt(filename, comments="#", delimiter=" ", dtype=float)
        except Exception as e:
            self.get_logger().error(f"Error loading PointCloud2 from file: {e}")
            return None

        # Create a PointCloud2 message and fill it with the loaded data
        header = self.pointcloud_data.header if self.pointcloud_data else None
        loaded_pointcloud = pc2.create_cloud_xyz32(header, loaded_points[:, :3])

        return loaded_pointcloud

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver_node = PointCloudSaverNode()
    rclpy.spin(pointcloud_saver_node)
    pointcloud_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

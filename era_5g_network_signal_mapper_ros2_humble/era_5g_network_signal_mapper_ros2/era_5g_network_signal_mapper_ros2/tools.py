import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import *
import sensor_msgs_py.point_cloud2 as pcl2
import numpy as np
import std_msgs.msg as std_msgs

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
            points = pcl2.read_points(self.pointcloud_data, field_names=("x", "y", "z", "rgb"), skip_nans=True)

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
                file.write(f"{x} {y} {z} {rgb}".rstrip()+'\n')

    def load_pointcloud_from_file(self, filename):
        try:
            # Read PointCloud2 data from the file
            loaded_points = np.loadtxt(filename, comments="#", delimiter=" ", dtype=float)
            if len(loaded_points) == 0:
                self.get_logger().warning("Loaded PointCloud2 data is empty.")
                return None
        except Exception as e:
            self.get_logger().error(f"Error loading PointCloud2 from file: {e}")
            return None
         
        header = std_msgs.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        # Create a PointCloud2 message and fill it with the loaded data
        #header = self.pointcloud_data.header if self.pointcloud_data else None
                # ROS DATATYPE 
        ros_dtype = PointField.FLOAT32
        fields = [
            PointField(name='x', offset=0, datatype=ros_dtype, count=1),
            PointField(name='y', offset=4, datatype=ros_dtype, count=1),
            PointField(name='z', offset=8, datatype=ros_dtype, count=1),
            # The 4th set of bytes represent colour of pointcloud
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        loaded_pointcloud = pcl2.create_cloud(header, fields, loaded_points)
        return loaded_pointcloud

    def load_pointcloud_from_file2(self, filename):
        try:
            # Read PointCloud2 data from the file
            loaded_points = np.loadtxt(filename, comments="#", delimiter=" ", dtype=float)
        except Exception as e:
            self.get_logger().error(f"Error loading PointCloud2 from file: {e}")
            return None

        # Create a PointCloud2 message and fill it with the loaded data
        header = self.pointcloud_data.header if self.pointcloud_data else None
        loaded_pointcloud = pcl2.create_cloud_xyz32(header, loaded_points[:, :3])

        return loaded_pointcloud
    

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver_node = PointCloudSaverNode()
    rclpy.spin(pointcloud_saver_node)
    pointcloud_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

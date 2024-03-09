import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
#import sensor_msgs.point_cloud2 as pc2
import sensor_msgs_py.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        self.create_subscription(PointCloud2, '/loaded_pointcloud', self.pointcloud_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(String, '/current_color', 10)
        self.semanic_pcl = None

        self.robot_position = None

    def pointcloud_callback(self, msg):

        self.semanic_pcl = msg
        

    def pose_callback(self, msg):
        # Extract robot position from PoseWithCovarianceStamped message
        self.robot_position = msg.pose.pose.position
                # Extract color information from PointCloud2 message
        detected_color = self.detect_color_in_bbox(self.semanic_pcl)

        # Publish the detected color to '/current_color'
        color_msg = String()
        color_msg.data = detected_color
        self.publisher_.publish(color_msg)


    def detect_color_in_bbox(self, pointcloud_msg):
        if self.robot_position is None:
            # No robot position available, cannot perform color detection
            return 'None'

        # Bounding box dimensions
        bbox_length = 0.3
        bbox_height = 0.3

        # Robot position
        robot_x = self.robot_position.x
        robot_y = self.robot_position.y

        # Extract point cloud data
        points = pcl2.read_points(pointcloud_msg, field_names=("x", "y", "z", "rgb", "confidence", "size"), skip_nans=True)

        for point in points:
            # Check if point is within the bounding box
            if (robot_x - bbox_length/2 < point[0] < robot_x + bbox_length/2) and \
               (robot_y - bbox_length/2 < point[1] < robot_y + bbox_length/2):

                # Extract RGB values from the point cloud
                rgb = point[3]
                print (str(rgb))
                if (16711680.0 == rgb):
                    return 'red'
                if (8190976.0 == rgb):
                    return 'yellow'
                if (16776960.0 == rgb):
                    return 'green'

        # No color found within the bounding box
        return 'None'

def main(args=None):
    rclpy.init(args=args)
    color_detector_node = ColorDetectorNode()
    rclpy.spin(color_detector_node)
    color_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

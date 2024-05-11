import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

'''
Ros2 Node that will publish the current color where the robot is located. It needs as input the semantic map as topic /loaded_pointcloud.
In predefined box where robot is located it is searching for points,
if first point is detected in the predefined color range it returns given collor as string in topic /current_color
TODO: Include other logics for getting colours such as return color which has most points in the area or other implementations
'''

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
                if (65535.0 == rgb):
                    return 'GREEN' #colour code green 1: 65535.0
                if (8190976.0 == rgb):
                    return 'YELLOW'#colour code yellow 2: 8190976.0
                if (7902720.0 == rgb):
                    return 'RED'#colour code red 3: 7902720.0
                if (8532735.0 == rgb):
                    return 'BLUE'#colour code blue 4: 8532735.0
                if (16711935.0 == rgb):
                    return 'PURPLE'#colour code purple 5: 16711935.0
                if (7864520.0 == rgb):
                    return 'PINK'#colour code pink 6: 7864520.0
                if (8421504.0 == rgb):
                    return 'TEAL'#colour code teal 7: 8421504.0
                if (7910400.0 == rgb):
                    return 'ORANGE'#colour code orange 8: 7910400.0
                if (16777215.0 == rgb):
                    return 'CYAN'#colour code cyan 9: 16777215.0
                if (3294790.0 == rgb):
                    return 'MAROON'#colour code maroon 10: 3294790.0
                if (5926430.0 == rgb):
                    return 'LIME'#colour code lime 11: 5926430.0
                if (4633620.0 == rgb):
                    return 'BROWN'#colour code brown 12: 4633620.0
                if (8388608.0 == rgb):
                    return 'BLACK'#colour code black 13: 8388608.0
                if (16711680.0 == rgb):
                    return 'RED_DARK'#colour code red_dark 14: 16711680.0
                if (8388736.0 == rgb):
                    return 'BLUE_DARK'#colour code blue_dark 15: 8388736.0
                if (32896.0 == rgb):
                    return 'GREEN_DARK'#colour code green_dark 16: 32896.0
                
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

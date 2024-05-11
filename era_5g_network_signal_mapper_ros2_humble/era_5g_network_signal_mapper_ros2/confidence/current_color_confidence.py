import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import math

'''
Ros2 Node that will updating confidence value in pcl data structure from standart deviation formula.
TODO: Finish implementation as it is not reflecting the desired outcome.
'''

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        self.create_subscription(PointCloud2, '/loaded_pointcloud', self.pointcloud_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(String, '/current_color', 10)
        self.standart_conficence = 0.5

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
        points = pcl2.read_points(pointcloud_msg, field_names=("x", "y", "z", "rgb", "confidence","size"), skip_nans=True)

        calculated_points = []

        green = 0
        yellow = 0
        red = 0
        total_samples = 0

        

        for point in points:
            # Check if point is within the bounding box
            if (robot_x - bbox_length/2 < point[0] < robot_x + bbox_length/2) and \
               (robot_y - bbox_length/2 < point[1] < robot_y + bbox_length/2):
                

                # Extract RGB values from the point cloud
                rgb = point[3]
                
                print (str(rgb))
                if (16711680.0 == rgb):
                    calculated_points.append(point)
                    total_samples = total_samples + 1                    
                    red = red + 1
                if (8190976.0 == rgb):
                    calculated_points.append(point)
                    total_samples = total_samples + 1
                    yellow = yellow + 1
                if (16776960.0 == rgb):
                    calculated_points.append(point)
                    total_samples = total_samples + 1
                    green = green + 1
                
        

        # if no sample found we exit our program
        if total_samples == 0:
            print("Nothin was found")
            return 'None'
        
        # if is only one sample standart confidence wil be applyed
        if total_samples== 1:
            #'''
            for point in points:
                # Check if point is within the bounding box
                if (robot_x - bbox_length/2 < point[0] < robot_x + bbox_length/2) and \
                (robot_y - bbox_length/2 < point[1] < robot_y + bbox_length/2):
                    point[4]=self.standart_conficence
                    
                    print("One sample was found")
            #'''

        else:

            print("total_samples")
            print(total_samples)
            print("green")
            print(green)
            print("yellow")
            print(yellow)
            print("red")
            print(red)

            meadian = (green * 1 + yellow * 2 + red * 3) / total_samples
            print("meadian")
            print(meadian)
            top_value = 0

            for point in calculated_points:
                rgb = point[3]
                if (16711680.0 == rgb):
                    top_value = top_value + ((3 - meadian) * (3 - meadian))
                    print("red topvalue")
                    print(top_value)
                if (8190976.0 == rgb):
                    top_value =  top_value + ((2 - meadian) * (2 - meadian))
                    print("yellow topvalue")
                    print(top_value)
                if (16776960.0 == rgb):
                    print("green topvalue")
                    print(top_value)
                    top_value =  top_value + ((1 - meadian) * (1 - meadian))
            
            print
            print("top_value")
            print(top_value)

            sqrt1=math.sqrt(top_value)
            print("sqrt1")
            print(sqrt1)

            omega = sqrt1/total_samples
            print("confidece = ")
            print(omega)

            print("robot_x")            
            print(robot_x)            
            print("robot_y")            
            print(robot_y)

            print("total_samples")
            print(total_samples)

            #'''
            for point in points:
                # Check if point is within the bounding box
                if (robot_x - bbox_length/2 < point[0] < robot_x + bbox_length/2) and \
                (robot_y - bbox_length/2 < point[1] < robot_y + bbox_length/2):
                    point[4]=omega
            for point in points:
                # Check if point is within the bounding box
                if (robot_x - bbox_length/2 < point[0] < robot_x + bbox_length/2) and \
                (robot_y - bbox_length/2 < point[1] < robot_y + bbox_length/2):
                    print("check confidence was updated")
                    print(point[4])
            #'''
        return 'None'


def main(args=None):
    rclpy.init(args=args)
    color_detector_node = ColorDetectorNode()
    rclpy.spin(color_detector_node)
    color_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

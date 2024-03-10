import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import *
import sensor_msgs_py.point_cloud2 as pcl2
import numpy as np
import std_msgs.msg as std_msgs
from pcl_interfaces.srv import AppendPcl
from pcl_interfaces.srv import PartialMapRequest
import sensor_msgs_py.point_cloud2 as pcl2
import math

class PointCloudSaverNode(Node):
    def __init__(self):
        super().__init__('pointcloud_saver_node')

        self.create_subscription(PointCloud2, '/semantic_pcl', self.pointcloud_callback, 1)
        self.save_service = self.create_service(Trigger, 'save_pointcloud', self.save_pointcloud_callback)
        self.load_and_publish_service = self.create_service(Trigger, 'load_and_publish_pointcloud', self.load_and_publish_pointcloud_callback)
        self.publisher_ = self.create_publisher(PointCloud2, '/loaded_pointcloud', 1)
        # Service for appending pointcloud
        #self.append_pcl = self.create_service(Trigger, 'append_pcl', self.append_pointcloud_callback)
        self.append_pcl_custom = self.create_service(AppendPcl, 'append_pcl', self.append_pointcloud_callback)      # CHANGE
        self.append_pcl_custom = self.create_service(AppendPcl, 'update_pcl', self.update_confidence_callback)      # CHANGE
        self.append_pcl_custom = self.create_service(PartialMapRequest, 'request_partial_map', self.create_partial_map_callback)      # CHANGE
        self.pointcloud_data = None


    def pointcloud_callback(self, msg):
        # Save the PointCloud2 data when it's received
        self.pointcloud_data = msg

    def save_pointcloud_callback(self, request, response):
        
        if self.pointcloud_data is not None:
            # Extract point cloud data
            points = pcl2.read_points(self.pointcloud_data, field_names=("x", "y", "z", "rgb", "confidence", "size"), skip_nans=True)

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
            file.write("# x y z rgb confidence size\n")

            # Write each point to the file
            for point in point_list:
                x, y, z, rgb, confidence, size = point
                file.write(f"{x} {y} {z} {rgb} {confidence} {size}".rstrip()+'\n')

    def load_pointcloud_from_file(self, filename:str)->PointCloud2:
        try:
            # Read PointCloud2 data from the file
            loaded_points = np.loadtxt(filename, comments="#", delimiter=" ", dtype=float)
            if len(loaded_points) == 0:
                self.get_logger().warning("Loaded PointCloud2 data is empty.")
                return None

        except Exception as e:
            self.get_logger().error(f"Error loading PointCloud2 from file: {e}")
            return None
        if loaded_points.ndim == 1:
            self.get_logger().warning("Only one point available. Point will be duplicated")
            loaded_points = np.stack((loaded_points, loaded_points))
         
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
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            # The 4th set of bytes represent colour of pointcloud
            PointField(name='confidence', offset=16, datatype=PointField.FLOAT32, count=1),
            # The 4th set of bytes represent colour of pointcloud
            PointField(name='size', offset=20, datatype=PointField.FLOAT32, count=1)
        ]
        loaded_pointcloud = pcl2.create_cloud(header, fields, loaded_points)
        return loaded_pointcloud
    
    # Historical pcl2
    merged_cloud = []

    def append_pointcloud_callback(self, request:AppendPcl, response):
        #self.get_logger().info("Retriving params sub_signal_mapper", once=True)
        #ros2 service call /append_pcl pcl_interfaces/srv/AppendPcl "{pcl1: pcl1.txt, pcl2: pcl2.txt, pcl_out: pcl3.txt}"

        pcl_filename_1 = request.pcl1
        pcl_filename_2 = request.pcl2
        pcl_out = request.pcl_out

        pcl_out_str = str(pcl_out)

        pcl_1 = self.load_pointcloud_from_file(str(pcl_filename_1))
        pcl_2 = self.load_pointcloud_from_file(str(pcl_filename_2))

        if (pcl_1 is not None) and (pcl_2 is not None):
            # Extract point cloud data        
            points_1 = pcl2.read_points(pcl_1, field_names=("x", "y", "z", "rgb", "confidence","size"), skip_nans=True)
            points_2 = pcl2.read_points(pcl_2, field_names=("x", "y", "z", "rgb", "confidence","size"), skip_nans=True)
            
            appended_points = np.append(points_1, points_2)

            # Convert points to a list for easy manipulation
            point_list = list(appended_points)
            # Save the PointCloud2 to a text file
            self.save_pointcloud_to_file(point_list, pcl_out_str)

            response.success = True
            response.message = 'PointCloud2 saved to file.'
        else:
            response.success = False
            response.message = 'No PointCloud2 data available.'

        return response
    
    def update_confidence_callback(self, request:AppendPcl, response):
        #ros2 service call /update_pcl pcl_interfaces/srv/AppendPcl "{pcl1: pcl1.txt, pcl2: pcl2.txt, pcl_out: pcl3.txt}"
        
        self.standart_conficence = 0.5

        pcl_filename_1 = request.pcl1
        pcl_out = request.pcl_out
        pcl_out_str = str(pcl_out)
        pcl_1 = self.load_pointcloud_from_file(pcl_filename_1)
        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0
        
        current_box_top_left_x = 0
        current_box_top_left_y = 0
        top_left_corner = 0

        nr_of_points = 0

        # if we have less points then squares => we calculate for point in points {else we calculate from squares}
        if pcl_1 is None:
            response.success = False
            #response.message = 'No PointCloud2 data available.'
            return response

        points = pcl2.read_points(pcl_1, field_names=("x", "y", "z", "rgb", "confidence","size"), skip_nans=True)
        # Iterate over the points to find min and max x and y coordinates
        for point in points:
                nr_of_points = nr_of_points + 1
                x, y, _, _, _, _ = point  # Extract x and y coordinates
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)

        # Size of each box in the grid
        box_size = 0.3
        

        # Calculate the number of boxes horizontally and vertically
        num_boxes_x = int((max_x - min_x) / box_size) + 1
        num_boxes_y = int((max_y - min_y) / box_size) + 1
        calculated_points = []

        # Loop through each box in the grid
        
        #response.success = False
        #response.message = 'No PointCloud2 data available.'
        box_size = 0.36
        for i in range(num_boxes_y):
            for j in range(num_boxes_x):
                # Calculate the top-left corner of the current box
                current_box_top_left_x = min_x + j * box_size
                current_box_top_left_y = max_y - i * box_size

                green = 0
                yellow = 0
                red = 0
                total_samples = 0
                meadian = 0
                calculated_points.clear()


                # Print or perform operations with the current box coordinates
                print(f"Box at position ({i}, {j}) has top-left corner at ({current_box_top_left_x}, {current_box_top_left_y})")
                for point in points:
                # Check if point is within the bounding box
                    if (current_box_top_left_x - box_size/2 < point[0] < current_box_top_left_x + box_size/2) and \
                    (current_box_top_left_y - box_size/2 < point[1] < current_box_top_left_y + box_size/2):
                        point[4]=self.standart_conficence
                        
                        top_left_corner = top_left_corner - box_size
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
                    print("Nothin was found for current unit area")
                    
                
                # if is only one sample standart confidence wil be applyed
                elif total_samples== 1:
                    for point in points:
                        # Check if point is within the bounding box
                        if (current_box_top_left_x - box_size/2 < point[0] < current_box_top_left_x + box_size/2) and \
                        (current_box_top_left_y - box_size/2 < point[1] < current_box_top_left_y + box_size/2):
                            point[4]=self.standart_conficence
                            
                            print("One sample was found")
                            
                            response.success = True
                            #response.message = 'PointCloud2 saved to file.'                

                else:

                    meadian = (green * 1 + yellow * 2 + red * 3) / total_samples

                    top_value = 0

                    for point in calculated_points:
                        rgb = point[3]
                        if (16711680.0 == rgb):
                            top_value = top_value + ((3 - meadian) * (3 - meadian))
                        if (8190976.0 == rgb):
                            top_value =  top_value + ((2 - meadian) * (2 - meadian))
                        if (16776960.0 == rgb):
                            top_value =  top_value + ((1 - meadian) * (1 - meadian))

                    omega = math.sqrt(top_value/(total_samples-1))

                    #'''
                    for point in points:
                        # Check if point is within the bounding box
                        if (current_box_top_left_x - box_size/2 < point[0] < current_box_top_left_x + box_size/2) and \
                        (current_box_top_left_y - box_size/2 < point[1] < current_box_top_left_y + box_size/2):
                            point[4]=omega
                            
                    response.success = True
                    #response.message = 'PointCloud2 saved to file.'
                    # Convert points to a list for easy manipulation

        # Convert points to a list for easy manipulation
        point_list = list(points)

        # Save the PointCloud2 to a text file
        self.save_pointcloud_to_file(point_list, pcl_out_str)

        return response       
                        
                        
    def create_partial_map_callback(self, request, response):
        #ros2 service call /update_pcl pcl_interfaces/srv/PartialMapRequest "{pcl1: pcl1.txt, min_x: -1.3, max_x: 1, min_y: -2, max_y: -1, pcl_out: pcl_out.txt}"
        # Extract filenames and bounding box limits from the request
        pcl_filename_1 = request.pcl1
        pcl_out = request.pcl_out
        pcl_out_str = str(pcl_out)
        min_x = request.min_x
        max_x = request.max_x
        min_y = request.min_y
        max_y = request.max_y       
         # Initialize an empty list to hold found points
        found_points = []


        # Attempt to load the first point cloud from file
        try:
            pcl_1 = self.load_pointcloud_from_file(pcl_filename_1)
        except Exception as e:
            # If loading fails, log the exception and set response failure
            self.get_logger().info(f"An rrror occured: {e}")
            response.success = False
            response.message = 'Error loading PointCloud: {}'.format(e)
            return response

        # Check if the point cloud is empty
        if pcl_1 is None:
            self.get_logger().info("No PointCloud2 data available.")
            response.success = False
            response.message = 'No PointCloud2 data available.'
            return response


        # Extract points from the loaded PointCloud2 data
        points = pcl2.read_points(pcl_1, field_names=("x", "y", "z", "rgb", "confidence", "size"), skip_nans=True)

        # Filter points within the specified bounding box
        for point in points:
            if (min_x < point[0] < max_x) and (min_y < point[1] < max_y):
                found_points.append(point)
        
        point_list = list(found_points)

        # Check if no points were found in the requested area
        if len(point_list) == 0:
            self.get_logger().info("No PointCloud2 data found in the requested area.")
            response.success = False
            response.message = 'No PointCloud2 data found in the requested area.'
            return response

        # Save the filtered points to a file
        # Ensure 'save_pointcloud_to_file' handles serialization appropriately
        self.save_pointcloud_to_file(point_list, pcl_out_str)

        # Set success response
        self.get_logger().info(f"PointCloud2 saved to file: {pcl_out_str}")
        response.success = True
        response.message = 'PointCloud2 saved to file: {}'.format(pcl_out_str)
        return response


def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver_node = PointCloudSaverNode()
    rclpy.spin(pointcloud_saver_node)
    pointcloud_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

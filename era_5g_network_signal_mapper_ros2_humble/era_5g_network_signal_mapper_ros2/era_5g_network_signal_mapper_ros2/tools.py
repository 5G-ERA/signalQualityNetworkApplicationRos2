import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import *
import sensor_msgs_py.point_cloud2 as pcl2
import numpy as np
import std_msgs.msg as std_msgs
from pcl_interfaces.srv import AppendPcl
import sensor_msgs_py.point_cloud2 as pcl2

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
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            # The 4th set of bytes represent colour of pointcloud
            PointField(name='confidence', offset=16, datatype=PointField.FLOAT32, count=1),
            # The 4th set of bytes represent colour of pointcloud
            PointField(name='size', offset=20, datatype=PointField.FLOAT32, count=1)
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
        loaded_pointcloud = pcl2.create_cloud_xyz32(header, loaded_points[:, :5])

        return loaded_pointcloud
    
    # Historical pcl2
    merged_cloud = []
    def combine_points(points_1, points_2):

        pass
    def append_pointcloud_callback(self, request:AppendPcl, response):
        #self.get_logger().info("Retriving params sub_signal_mapper", once=True)
        #ros2 service call /append_pcl pcl_interfaces/srv/AppendPcl "{pc1: pcl1.txt, pc2: pcl2.txt, pcl_out: pcl3.txt}"

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
    

    # Function to construct a pointcloud2 object 
    # which will represent the collections of current clouds 
    def construct_pointCloud2(points):
                    # ROS DATATYPE
                    ros_dtype = PointField.FLOAT32
                    # The PointCloud2 message also has a header which specifies which
                    # coordinate frame it is represented in.
                    header = std_msgs.Header()
                    # The fields specify what the bytes represents. The first 4 bytes
                    # represents the x-coordinate, the next 4 the y-coordinate, etc.
                    fields = [
                        PointField(name='x', offset=0, datatype=ros_dtype, count=1),
                        PointField(name='y', offset=4, datatype=ros_dtype, count=1),
                        PointField(name='z', offset=8, datatype=ros_dtype, count=1),
                        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                        PointField(name='confidence', offset=16, datatype=PointField.FLOAT32, count=1),
                        PointField(name='size', offset=20, datatype=PointField.FLOAT32, count=1)
                    ]
                    pcl_msg = pcl2.create_cloud(header, fields, points)
                    pcl_msg.header.stamp = self.get_clock().now().to_msg()
                    pcl_msg.header.frame_id = self.map_frame
                    return pcl_msg
                
    # Creating sub_callback which will receive current cloud of the robot "pcl"
    def sub_callback(pcl):
        # the amount of time to wait until fail receiving both frames
        timeout = Duration(seconds=10)
        # Waiting until transfrom of map_frame and semantic_map_frame is available
        transform = self.tf_buffer.lookup_transform('map', 'semantic_map', rclpy.time.Time(), timeout)
        # Atempt to transform frame "map" to "semantic_map" and add cloud received from "pcl"
        try:
            transformed_cloud = tf2_c.do_transform_cloud(pcl, transform)
        except Exception as ex:
            self.get_logger().info(str(ex))
        # Get the points from the pcl2.
        self.get_logger().info("Publishing transformed_cloud", once=True)
        gen = pcl2.read_points(transformed_cloud, skip_nans=True)
        int_data = list(gen)
        temp_list = []
        for element in int_data:
            mini = []
            for x in element:
                mini.append(x)
            temp_list.append(mini)
        # Append latest pointcloud to merged_cloud
        for y in temp_list:
            merged_cloud.append(y)
        # Recreate the merged pointcloud
        map_pcl = construct_pointCloud2(merged_cloud)
        # Publishing a map created from collection of current semantic cloud of the robot
        # Rate of pubication is rate of "semantic_pcl_sub" minus time of waiting
        # frames for transformation from "self.tf_buffer.lookup_transform...."
        self.semantic_pcl_pub.publish(map_pcl)

    
    

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver_node = PointCloudSaverNode()
    rclpy.spin(pointcloud_saver_node)
    pointcloud_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

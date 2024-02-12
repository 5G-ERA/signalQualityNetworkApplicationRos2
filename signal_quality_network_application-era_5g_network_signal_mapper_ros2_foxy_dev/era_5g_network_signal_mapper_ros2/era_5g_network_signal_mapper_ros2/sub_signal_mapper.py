#!/usr/bin/env python3
import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg as std_msgs
import sensor_msgs_py.point_cloud2 as pcl2
import time
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
#import tf2_sensor_msgs

#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import PyKDL
from tf2_ros import TransformException


# NEED_REVIEWING!!!!

# Follow sign UNCOMENT_HERE


class SubSignalMapper(Node):
    
        def __init__(self):
            super().__init__("pcl2_semantic_mapper")

            self.tf_buffer = tf2_ros.Buffer() 
            self.listener = tf2_ros.TransformListener(self.tf_buffer,self)

            # Declare parameters
            self.declare_parameter("map_frame", rclpy.Parameter.Type.STRING)
            self.declare_parameter("semantic_map_frame", rclpy.Parameter.Type.STRING)

            # Retreive parameters from launch/yaml file 
            self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
            self.semantic_map_frame = self.get_parameter("semantic_map_frame").get_parameter_value().string_value

            # //******************// //******************// //******************// 
            # NEED_REVIEWING!!!!  queue_size need adjusted for publisher
            self.semantic_pcl_pub = self.create_publisher(PointCloud2, '/semantic_pcl',10)
            
            # //******************// //******************// //******************// 

            # Historical pcl2
            merged_cloud = []
            

            # Function to construct a pointcloud2 object
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
                    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
                ]
                pcl_msg = pcl2.create_cloud(header, fields, points)
                pcl_msg.header.stamp = self.get_clock().now().to_msg()
                pcl_msg.header.frame_id = self.map_frame
                return pcl_msg

            #  //******************// //******************// //******************//

            # cloud_out = do_transform_cloud(pcl, trans) # Replacement functionality
            def transform_to_kdl(t : TransformStamped):
                return PyKDL.Frame(
                    PyKDL.Rotation.Quaternion(
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w),
                    PyKDL.Vector(
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z)
                    )
            
            def do_transform(cloud, transform):
                t_kdl = transform_to_kdl(transform)
                points_out = []
                
                for p_in in pcl2.read_points(cloud):
                    p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
                    self.get_logger().info("p_in first: "+str(p_in[3]))
                    #for element in p_in:
                        #self.get_logger().info("p_in first: "+str(p_in))
                        #self.get_logger().info(str(element))
                    '''
                    self.get_logger().info("===========")
                    self.get_logger().info(str(p_out))
                    '''
                    '''
                    if (p_in[3:] == 0):
                        p_in[3:] = 0.0
                    '''
                    #self.get_logger().info("index: "+str(len(p_in)))
                    try:
                        #if (p_in[3] == 16711680):

                        
                       # points_out.append((p_out[0], p_out[1], p_out[2]) + p_in[3:])
                        
                        points_out.append((p_out[0], p_out[1], p_out[2]) + (p_in[3],)) # to be fixed
                    except Exception as e:
                        #self.get_logger().info("no se hizo append")
                        print((p_out[0], p_out[1], p_out[2]) + p_in[3:])
                
                try:
                    #self.get_logger().info("p_in"+str(p_in))
                    res = pcl2.create_cloud(transform.header, cloud.fields, points_out)
                except Exception as e:
                    #self.get_logger().info(str(p_in))
                    #self.get_logger().info("no creada pointcloud")
                    #self.get_logger().info(e)
                    aux = False
                    
                #res = pcl2.create_cloud(transform.header, cloud.fields, [])
                return res
            

            def do_transform_cloud(cloud, transform):
                
                t_kdl = transform_to_kdl(transform)
                points_out = []
                points_in = []
                for p_in in pcl2.read_points(cloud):
                    p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
                    points_out.append(p_out)

                self.get_logger().info("hola"+str(points_out), once=True)
                print("holii"+str(points_out))
                ros_dtype = PointField.FLOAT32
                header = std_msgs.Header()
                fields = [
                    PointField(name='x', offset=0, datatype=ros_dtype, count=1),
                    PointField(name='y', offset=4, datatype=ros_dtype, count=1),
                    PointField(name='z', offset=8, datatype=ros_dtype, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
                ]

                res = pcl2.create_cloud(header, fields, points_out)
                return res    
            #  //******************// //******************// //******************//        


            def sub_callback(pcl):   
                
                self.get_logger().info(f'1', once=True)    
                         
                 
                #tf_buffer = tf2_ros.Buffer()               

                source_frame = self.map_frame
                target_frame = self.semantic_map_frame
                timeout = Duration(seconds=10)  # Create a duration of 4 seconds
                
                # //******************// //******************// //******************// 
                # tf.waitForTransform v1 NEED_REVIEWING!!!!
                '''
                tf_listener = tf2_ros.TransformListener(tf_buffer, self) # broken here.
                while not tf_buffer.can_transform(target_frame, source_frame, timeout):
                    rclpy.spin_once(tf_listener.node)
                # # tf.waitForTransform v2
                tf_buffer.can_transform(source_frame, target_frame, rclpy.time.Time(), timeout)
                    
                '''
                
                #
                # //******************// //******************// //******************// 

                # UNCOMENT_HERE 3 '''
                

                transform = self.tf_buffer.lookup_transform('map', 'semantic_map', rclpy.time.Time(), timeout)
                self.get_logger().info(f'2', once=True) 

                #transformed_cloud = tf2_sensor_msgs.do_transform_cloud(pcl, transform)

                transformed_cloud = do_transform(pcl, transform)
                print("=============")
                print(transformed_cloud)

                # Get the points from the pcl2.
                self.get_logger().info(str(type(transformed_cloud)), once=True)

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
                #merged_cloud.append(temp_list) # no es append, es otra cosa parecida.

                # Recreate the merged pointcloud
                map_pcl = construct_pointCloud2(merged_cloud) 
                self.semantic_pcl_pub.publish(map_pcl)

                # UNCOMENT_HERE 3 simbols
                
                #time.sleep(1.0) # NEED_REVIEWING!!!! maybe delay to be removed
                
            # //******************// //******************// //******************// 
            # NEED_REVIEWING!!!!  buff_size need adjusted for subscriber here is missing
            self.semantic_pcl_sub = self.create_subscription(PointCloud2, "/current_semantic_pcl",sub_callback, qos_profile=1,)
            # //******************// //******************// //******************//


def main(args=None):
    rclpy.init(args=args)
    node = SubSignalMapper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import sensor_msgs_py.point_cloud2 as pcl2
from nav_msgs.srv import GetMap
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import era_5g_network_signal_mapper_ros2.declare_param as paramm

# List of colours from the PointCloud2 that should not be treated as obstacles by the ROS Navigation Stack.
ACCEPTED_COLOURS = []

# Parser of map information; collecting "width" "height" "resolution" of the map; "origin_x" "origin_y" of the robot;
def metadata_manual_parser(data: str)->dict:
    
    w_temp = data.find("width")
    coma_index = 0
    for x in range(w_temp,w_temp+10):
        
        if data[x]==",":
            coma_index = x

    width = int(data[int(w_temp)+6:int(coma_index)])
    print("width: "+str(width))

    h_temp = data.find("height")
    second_comma_index = 0
    for y in range(h_temp,h_temp+15):
        
        if data[y]==",":
            second_comma_index = y

    height = int(data[int(h_temp)+7:int(second_comma_index)])
    print("height: "+str(height))

    origin_x_temp = data.find("x=")
    coma_origin_x_temp = 0
    for z in range(origin_x_temp,origin_x_temp+15):
        
        if data[z]==",":
            coma_origin_x_temp = z

    origin_x = float(data[int(origin_x_temp)+2:int(coma_origin_x_temp)])
    print("origin_x: "+str(origin_x))


    origin_y_temp = data.find("y=")
    coma_origin_y_temp = 0
    for g in range(origin_y_temp,origin_y_temp+15):
        
        if data[g]==",":
            coma_origin_y_temp = g

    origin_y = float(data[int(origin_y_temp)+2:int(coma_origin_y_temp)])
    print("origin_y: "+str(origin_y))

    resolution_temp = data.find("resolution=")
    coma_resolution_temp = 0
    for j in range(resolution_temp,resolution_temp+40):
        
        if data[j]==",":
            coma_resolution_temp = j

    resolution = float(data[int(resolution_temp)+11:int(coma_resolution_temp)])
    print("resolution: "+str(resolution))

    metadata = {"width": width, "height": height, "origin_x": origin_x, "origin_y":origin_y, "resolution": resolution}

    return metadata

class Colour():
    def __init__(self, rgb):
        self.rgb = rgb
        self.binary = int.from_bytes(bytes(rgb[2:0:-1]), byteorder='little', signed=False)

Green = Colour([124, 252, 0])
Red = Colour([255, 0, 0])
Blue = Colour([128, 0, 0])
Yellow = Colour([255, 255, 0])
Orange = Colour([255, 165, 0])
GreenInt = 8190976

ACCEPTED_COLOURS.append(Green.binary)
ACCEPTED_COLOURS.append(GreenInt)

# This class will recreate a new map from given parameteres "origin_x", "origin_y", "resolution", "width", "height";
# And will assign frame_id as "map_frame" from "toMap(self, node, map_data, map_frame: str)" inside this class
class Map(object):
    def __init__(self, origin_x, origin_y, resolution, width, height):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.full((height, width), -1.)

    def toMap(self, node, map_data, map_frame: str):

        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        self.grid_msg = OccupancyGrid()
        # Set up the header.
        time_stamp = Clock().now()
        self.grid_msg.header.stamp = time_stamp.to_msg()
        self.grid_msg.header.frame_id = map_frame
        # .info is a nav_msgs/MapMetaData message.
        self.grid_msg.info.resolution = self.resolution
        self.grid_msg.info.width = self.width
        self.grid_msg.info.height = self.height

        node.get_logger().info('self.origin_x '+str(self.origin_x))
        node.get_logger().info('self.origin_y '+str(self.origin_y))

        # Rotated maps are not supported... quaternion represents no rotation.
        point_map = Point(x=float(self.origin_x ), y=float(self.origin_y), z=0.0)
        quat =  Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
        self.grid_msg.info.origin = Pose(position=point_map, orientation=quat)
        self.grid_msg.data = map_data.map.data
        return self.grid_msg
    
    
class Mapper(Node):
    def __init__(self):
        super().__init__('pcl2_to_costmap')

        # In the self.response will be stored row map data from "/map_server/map"
        self.response = None
        # Retreive parameters from param / launch or params.yaml file / ENV or set default values automaticaly
        self.map_frame: str = paramm.param_set_string(self,'my_map_frame', 'map')       

        # CALL SERVICE MAP
        self.get_logger().info("Reading data of original map...", once=True)
        self.get_logger().info('Waiting for map_server service...')

        # creating custom qos_profile of map
        self.qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Creating request to receive map data from "/map_server/map"
        # "/map_server/map" represent costmap
        client = self.create_client(GetMap, ' self.pcl_sub')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('map_server service not available, waiting...')
        # Create the request message
        request = GetMap.Request()
        # Call the service and get the response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.response = future.result()
            # Process the response here
            # In metadata we store parsed information of map received from "/map_server/map" 
            metadata = metadata_manual_parser(str(self.response))
            self.get_logger().info('Received map data form map server!')
            # You can access the map and metadata using response.map and response.map_metadata
        else:
            self.get_logger().info('Failed to receive map data.')

        # Creating a new identical map received from "/map_server/map"
        self._map = Map(metadata['origin_x'], metadata['origin_y'], metadata['resolution'], metadata['width'], metadata['height'])
        # Assigning frame_id 
        self.grid_msg = self._map.toMap(self, self.response, self.map_frame)

        # Create publishers
        self._map_pub = self.create_publisher(
            msg_type=OccupancyGrid, 
            topic='/map_Semantic',
            qos_profile=self.qos_profile
        )
        # Review and delete or uncomment next line
        #self._map_metadata_pub = self.create_publisher(MapMetaData, '/map_Semantic_metadata', 1)
        # Create a subscriber to receive information of all collection of clouds which represent signal strenght
        self.pcl_sub = self.create_subscription(PointCloud2, '/semantic_pcl', self.pcl_callback, 1)

    # Transformation into into a numpy MaskedArray.
    def occupancygrid_to_numpy(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return np.ma.array(data, mask=data==-1, fill_value=-1)
    
    def pcl_callback(self, pointcloud_msg):
        # pointcloud_msg is all collection of clouds which represent signal strenght
        # pcl_cloud represent a list from all collections of clouds received from signal strenght
        pcl_cloud = list(pcl2.read_points(pointcloud_msg, skip_nans=True))
        # To manipulate the OccupancyGrid and add the pcl2 data, first it needs to be translated into a numpy MaskedArray.
        # On top of the received map " self._map" will be added obstacles from pcl2 if they exist and are in "ACCEPTED_COLOURS"
        np_occupancy = self.occupancygrid_to_numpy(self.grid_msg)
        self.get_logger().info("occupancygrid_to_numpy working", once=True)
        self.get_logger().info("ACCEPTED_COLOURS "+str(ACCEPTED_COLOURS), once=True)
        try:
            for point in pcl_cloud:
                # Only pcl points with colour not in the accepted array will be considered obstacles and added to the grid with prob(100)
                if point[3] not in ACCEPTED_COLOURS:
               
                    self.get_logger().info("point[3] "+str(point[3]), once=True)
                    y = int((point[0] - self._map.origin_x) / self._map.resolution)
                    x = int((point[1] - self._map.origin_y) / self._map.resolution)
                    np_occupancy[x][y] = 100.
            
        except Exception as e:
            print("ERROR: "+str(e))        

        try:
            # Publish combined map with pcl2 as obstacle. 
            self._map_pub.publish(self.grid_msg)
            self.get_logger().info("Publishing semantic map.", once=True)
        
        except Exception as e:
            print("SECOND ERROR: "+str(e))
        

def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

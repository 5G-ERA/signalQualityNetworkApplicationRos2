import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as msg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import sensor_msgs_py.point_cloud2 as pcl2
from nav_msgs.srv import GetMap
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# List of colours from the PointCloud2 that should not be treated as obstacles by the ROS Navigation Stack.
ACCEPTED_COLOURS = []


def metadata_manual_parser(data: str)->dict:
    '''
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

	'''
    metadata = {"width": 1695, "height": 2109, "origin_x": -17.1, "origin_y":-31.6, "resolution": 0.02}

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

ACCEPTED_COLOURS.append(Green.binary)
class Map(object):
    def __init__(self, origin_x=-50, origin_y=-50, resolution=0.02, width=50, height=50):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.full((height, width), -1.)

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        self.grid_msg = OccupancyGrid()
        # Set up the header.
        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.grid_msg.header.frame_id = 'map'
        # .info is a nav_msgs/MapMetaData message.
        self.grid_msg.info.resolution = self.resolution
        self.grid_msg.info.width = self.width
        self.grid_msg.info.height = self.height
        # Rotated maps are not supported... quaternion represents no rotation.
        self.grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0), Quaternion(0, 0, 0, 1))
        # Flatten the numpy array into a list of integers from 0-100.
        self.flat_grid = self.grid.reshape((self.grid.size,))
        self.final = list(np.round(self.flat_grid))
        self.grid_msg.data = self.final
        return self.grid_msg
    
    def toMap(self, node, map_data):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        self.grid_msg = OccupancyGrid()
        # Set up the header.
        time_stamp = Clock().now()
        self.grid_msg.header.stamp = time_stamp.to_msg()
        self.grid_msg.header.frame_id = 'map'
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
        #node.get_logger().info('map_data.map.data '+str(map_data.map.data[:]))
        self.grid_msg.data = map_data.map.data#[:]
        return self.grid_msg
    
    '''
    def toMap(self, map_data):
            """ Return a nav_msgs/OccupancyGrid representation of this map. """
            self.grid_msg = OccupancyGrid()
            # Set up the header.
            time_stamp = Clock().now()
            self.grid_msg.header.stamp = time_stamp.to_msg()
            self.grid_msg.info.origin = Pose(Point(-6.28, -5.16, 0), Quaternion(0, 0, 0, 1))
            self.grid_msg.header.frame_id = 'map'
            # .info is a nav_msgs/MapMetaData message.
            self.grid_msg.info.resolution = 0.05
            self.grid_msg.info.width = 275
            self.grid_msg.info.height = 246
            # Rotated maps are not supported... quaternion represents no rotation.
            self.grid_msg.data = map_data
            return self.grid_msg
    '''
    
class Mapper(Node):
    def __init__(self):
        super().__init__('pcl2_to_costmap')
        map_frame = 'map'
        map_topic = '/map'
        map_topic_metadata = '/robot/map_metadata'
        '''ROS node definition'''

        #self.res = None
        #self.o = None
        #self.grid_msg = None
        #self._map = None

        # Wait for the map_metadata and map topics to become available
        #self.metadata_sub = self.create_subscription(MapMetaData, map_topic_metadata, self.map_metadata_callback, 1)
        #self.metadata_sub

        self.res = 0.05# data.resolution
        self.w = 275 #data.width
        self.h = 246 #data.height
        #self.o = data.origin
        # Create map
        
        #self.grid_msg = self._map.toMap()
        self.response = None
        

        # CALL SERVICE MAP
        self.get_logger().info("Reading data of original map...", once=True)
        self.get_logger().info('Waiting for map_server service...')

        self.qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        client = self.create_client(GetMap, '/map_server/map')
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
            
            #print(str(self.response.map.data[0]))
            metadata = metadata_manual_parser(str(self.response))
            self.get_logger().info('Received map data form map server!')
            # You can access the map and metadata using response.map and response.map_metadata
        else:
            self.get_logger().info('Failed to receive map data.')

        self._map = Map(metadata['origin_x'], metadata['origin_y'], metadata['resolution'], metadata['width'], metadata['height'])
        #self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 1)
        #self.map_sub
        self.grid_msg = self._map.toMap(self, self.response)

        #self._map_metadata_pub.publish(self.grid_msg.info)
        #self._map_pub.publish(self.grid_msg)

        # Create publishers
        self._map_pub = self.create_publisher(
            msg_type=OccupancyGrid, 
            topic='/map_Semantic',
            qos_profile=self.qos_profile
        )
        self._map_metadata_pub = self.create_publisher(MapMetaData, '/map_Semantic_metadata', 1)
        self.pcl_sub = self.create_subscription(PointCloud2, '/semantic_pcl', self.pcl_callback, 1)
        self.get_logger().info("Publishing semantic map.", once=True)
        
    def occupancygrid_to_numpy(self, msg):
        #self.get_logger().info(str(msg))
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return np.ma.array(data, mask=data==-1, fill_value=-1)
    
    def numpy_to_occupancy_grid(self, arr, info=None):
        if not len(arr.shape) == 2:
            raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
            raise TypeError('Array must be of int8s')
        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
            arr = arr.data
        grid.data = arr.ravel()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info = info
        return grid
    
    def pcl_callback(self, pointcloud_msg):
        
        #self.get_logger().info(str(pointcloud_msg), once=True)
        pcl_cloud = list(pcl2.read_points(pointcloud_msg, skip_nans=True))
        # To manipulate the OccupancyGrid and add the pcl2 data, first it needs to be translated into a numpy MaskedArray.
        np_occupancy = self.occupancygrid_to_numpy(self.grid_msg)
        self.get_logger().info("occupancygrid_to_numpy working", once=True)
        self.get_logger().info("ACCEPTED_COLOURS "+str(ACCEPTED_COLOURS), once=True)
        try:
            for point in pcl_cloud:
                #print(point)
                # Only pcl points with colour not in the accepted array will be considered obstacles and added to the grid with prob(100)
                #if point[3] not in ACCEPTED_COLOURS:
                if point[3] !=  8190976: # not in ACCEPTED_COLOURS:
                    self.get_logger().info("point[3] "+str(point[3]), once=True)
                    #y = int((point[0] - self._map.origin_x) / self._map.resolution)
                    #x = int((point[1] + self._map.origin_y*1 - 9) / self._map.resolution)

                    y = int((point[0] - self._map.origin_x) / self._map.resolution)
                    x = int((point[1] - self._map.origin_y) / self._map.resolution)
        

                   # y = int((point[0] - self._map.origin_x) / self._map.resolution)
                   # x = int((point[1]*50 + self._map.origin_y*1 - 40) / self._map.resolution + 11)

                   # y = int((point[0] - self._map.origin_x) / self._map.resolution)
                   # x = int((point[1] + self._map.origin_y) / self._map.resolution)

                    np_occupancy[x][y] = 100.
            
        except Exception as e:
            print("ERROR: "+str(e))
        

        try:
            #print(self.grid_msg.info)
            #map_occupancy_grid = self.numpy_to_occupancy_grid(np_occupancy, self.grid_msg.info)
            #self._map_metadata_pub.publish(self.grid_msg.info)
            self._map_pub.publish(self.grid_msg)
        except Exception as e:
            print("SECOND ERROR: "+str(e))

    
    '''Only called once'''
    def map_metadata_callback(self, data):
        self.res = data.resolution
        self.w = data.width
        self.h = data.height
        self.o = data.origin
        # Create map
        self._map = Map(self.o.position.x, self.o.position.y, self.res, self.w, self.h)
        self.get_logger().info("Reading metadata of original map...", once=True)
        

def main():
    
    try:
        rclpy.init()
        
        m = Mapper()
        
        m.get_logger().info('Waiting for map_server service...')
        client = m.create_client(GetMap, '/map_server/map')
        while not client.wait_for_service(timeout_sec=1.0):
            m.get_logger().info('map_server service not available, waiting...')
        # Create the request message
        request = GetMap.Request()
        # Call the service and get the response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(m, future)
        if future.result() is not None:
            response = future.result()
            # Process the response here
            # print(response)
            m.get_logger().info('Received map data!')
            # You can access the map and metadata using response.map and response.map_metadata
        else:
            m.get_logger().info('Failed to receive map data.')
        
        rclpy.spin(m)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        m.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






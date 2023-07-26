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

# List of colours from the PointCloud2 that should not be treated as obstacles by the ROS Navigation Stack.
ACCEPTED_COLOURS = []

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
    '''
    def toMap(self, map_data):
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
        self.grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                                         Quaternion(0, 0, 0, 1))
        self.grid_msg.data = map_data
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
        self._map = Map(-6.28, -5.16, self.res, self.w, self.h)
        #self.grid_msg = self._map.toMap()
        self.response = None
        

        # CALL SERVICE MAP
        self.get_logger().info("Reading data of original map...", once=True)
        self.get_logger().info('Waiting for map_server service...')
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
            print(self.response)
            self.get_logger().info('Received map data!')
            # You can access the map and metadata using response.map and response.map_metadata
        else:
            self.get_logger().info('Failed to receive map data.')

        #self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 1)
        #self.map_sub
        self.grid_msg = self._map.toMap(self.response)

        #self._map_metadata_pub.publish(self.grid_msg.info)
        #self._map_pub.publish(self.grid_msg)

        # Create publishers
        self._map_pub = self.create_publisher(OccupancyGrid, '/map_Semantic', 1)
        self._map_metadata_pub = self.create_publisher(MapMetaData, '/map_Semantic_metadata', 1)
        self.pcl_sub = self.create_subscription(PointCloud2, '/semantic_pcl', self.pcl_callback, 1)
        self.get_logger().info("Publishing semantic map.", once=True)
        
    def occupancygrid_to_numpy(self, msg):
        self.get_logger().info(str(msg))
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
        try:
            for point in pcl_cloud:
                # Only pcl points with colour not in the accepted array will be considered obstacles and added to the grid with prob(100)
                if point[3] not in ACCEPTED_COLOURS:
                    y = int((point[0] - self._map.origin_x) / self._map.resolution)
                    x = int((point[1]*50 + self._map.origin_y*1 - 40) / self._map.resolution + 11)
                    np_occupancy[x][y] = 100.
            map_occupancy_grid = self.numpy_to_occupancy_grid(np_occupancy, self.grid_msg.info)
            self._map_metadata_pub.publish(self.grid_msg.info)
            self._map_pub.publish(map_occupancy_grid)
        except Exception as e:
            print(e)

    def map_callback(self, msg):
        self.get_logger().info("map callback", once=True)
        try:
            self.grid_msg = self._map.toMap(msg.data)
            self._map_metadata_pub.publish(self.grid_msg.info)
            self._map_pub.publish(self.grid_msg)
        except Exception as a:
            print(a)
    '''Only called once'''
    def map_metadata_callback(self, data):
        self.res = data.resolution
        self.w = data.width
        self.h = data.height
        self.o = data.origin
        # Create map
        self._map = Map(self.o.position.x, self.o.position.y, self.res, self.w, self.h)
        self.get_logger().info("eading metadata of original map...", once=True)
        

def main():
    
    try:
        rclpy.init()
        
        m = Mapper()
        '''
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
            print(response)
            m.get_logger().info('Received map data!')
            # You can access the map and metadata using response.map and response.map_metadata
        else:
            m.get_logger().info('Failed to receive map data.')
        '''
        rclpy.spin(m)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        m.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




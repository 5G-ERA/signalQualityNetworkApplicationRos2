import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped

class ColorChangeRecorder(Node):
    def __init__(self):
        super().__init__('color_change_recorder')
        self.color_subscriber = self.create_subscription(String, '/pcl_colour', self.color_callback, 10)
        self.position_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.position_callback, 10)
        self.coordinates_edge_green = []
        self.coordinates_edge_yellow = []
        self.previous_color = None
        self.current_color = None
        self.circle_center_publisher = self.create_publisher(Float32MultiArray, '/circle_center', 10)
        self.signal_data_publisher = self.create_publisher(Float32MultiArray, '/signal_data_reconstruction', 10)
        #self.center = None
        self.green_center = None
        self.yelow_center = None
        self.green_radius = 0.0
        self.yellow_radius = 0.0


    def publish_estimated_sqm(self):
        # Publish circle center data
        circle_center = Float32MultiArray()
        center = [0.0,0.0]
        if self.yellow_center == None:
            center = self.green_center
        elif self.green_center == None:
            center = self.yellow_center
        
        else:
            mid_x = (self.yellow_center[0] + self.green_center[0])/2
            mid_y = (self.yellow_center[1] + self.green_center[1])/2
            center = [mid_x, mid_y]

        circle_center.data = center
        self.circle_center_publisher.publish(circle_center)
        self.get_logger().info(f'Published circle center: {circle_center.data}')

        # Publish signal data reconstruction
        signal_data = Float32MultiArray()
        signal_data.data = [self.green_radius, self.yellow_radius]
        self.signal_data_publisher.publish(signal_data)
        self.get_logger().info(f'Published signal data: {signal_data.data}')


    def color_callback(self, msg):
        self.previous_color = self.current_color
        self.current_color = msg.data
        if self.previous_color is not None and self.previous_color != self.current_color:
            if (self.previous_color == 'YELLOW' and self.current_color == 'GREEN') or \
               (self.previous_color == 'GREEN' and self.current_color == 'YELLOW'):
                self.add_coordinate_green()
        
        if self.previous_color is not None and self.previous_color != self.current_color:
            if (self.previous_color == 'YELLOW' and self.current_color == 'RED') or \
               (self.previous_color == 'RED' and self.current_color == 'YELLOW'):
                self.add_coordinate_yellow()

    def position_callback(self, msg):
        self.robot_position = msg.pose.pose.position

    def add_coordinate_green(self):
        x = self.robot_position.x
        y = self.robot_position.y
        self.coordinates_edge_green.append((x, y))
        self.get_logger().info(f'Added edge green coordinate: ({x}, {y})')
        if len(self.coordinates_edge_green) == 3:
            self.get_logger().info('Detected three color change coordinates of green. Stopping the robot.')
            #for coordinate in self.coordinates_edge_green:
                #self.get_logger().info(f'Coordinate: {coordinate}')
            green_center, green_radius = self.calculate_center(self.coordinates_edge_green)
            self.get_logger().info(f'Added edge green coordinate center: ({green_center}, radius {green_radius})')
            self.green_center = green_center
            self.green_radius = green_radius
            if self.yellow_radius != 0:
                yellow_radius = self.yellow_radius
                self.yellow_radius = yellow_radius - self.green_radius

            self.publish_estimated_sqm()

    def add_coordinate_yellow(self):
        x = self.robot_position.x
        y = self.robot_position.y
        self.coordinates_edge_yellow.append((x, y))
        self.get_logger().info(f'Added edge yellow coordinate: ({x}, {y})')
        if len(self.coordinates_edge_yellow) == 3:
            self.get_logger().info('Detected three color change coordinates of yellow. Stopping the robot.')
            #for coordinate in self.coordinates_edge_yellow:
                #self.get_logger().info(f'Coordinate: {coordinate}')
            # Implement code to stop the robot here

            yellow_center, yellow_radius = self.calculate_center(self.coordinates_edge_yellow)

            self.yellow_center = yellow_center
            self.get_logger().info(f'Added edge yellow coordinate center: ({yellow_center}, radius {yellow_radius})')

            self.yellow_radius = yellow_radius
            if self.green_radius != 0:
                yellow_radius = self.yellow_radius
                self.yellow_radius = yellow_radius - self.green_radius

            self.publish_estimated_sqm()

    '''
    
    z'''
    def calculate_center(self, coordinates_edge: list):
        A = coordinates_edge[0]
        B = coordinates_edge[1]
        C = coordinates_edge[2]

        #A = (5.8365559577941895, 0.976704478263855)
        #B = (1.2366442680358887, 2.427051067352295)
        #C = (0.7705655097961426, 2.007391929626465)


        # Calculate the midpoints of AB and BC
        mid_AB = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
        mid_BC = ((B[0] + C[0]) / 2, (B[1] + C[1]) / 2)

        # Calculate the slopes of AB and BC
        slope_AB = (B[1] - A[1]) / (B[0] - A[0])
        slope_BC = (C[1] - B[1]) / (C[0] - B[0])

        # Calculate the perpendicular slopes
        perp_slope_AB = -1 / slope_AB
        perp_slope_BC = -1 / slope_BC

        # Define the functions of the perpendicular bisectors
        def perp_bisector(m, midpoint):
            # y = mx + c
            c = midpoint[1] - m * midpoint[0]
            return m, c

        m_AB, c_AB = perp_bisector(perp_slope_AB, mid_AB)
        m_BC, c_BC = perp_bisector(perp_slope_BC, mid_BC)

        # Find the intersection of the two perpendicular bisectors
        # m1*x + c1 = m2*x + c2
        # x = (c2 - c1) / (m1 - m2)
        x_center = (c_BC - c_AB) / (m_AB - m_BC)
        y_center = m_AB * x_center + c_AB
        center = (x_center, y_center)

        # Calculate the radius of the circle
        radius = np.sqrt((A[0] - center[0])**2 + (A[1] - center[1])**2)

        return center, radius
    
def main(args=None):
    rclpy.init(args=args)
    node = ColorChangeRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

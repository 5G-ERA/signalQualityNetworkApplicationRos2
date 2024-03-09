#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import random
from datetime import datetime
import threading

class MapExplorationClient(Node):
    def __init__(self, map_width, map_height,exploration_duration):
        super().__init__('map_exploration_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_width = map_width
        self.map_height = map_height
        # Set a duration for the map exploration phase, e.g., 300 seconds (5 minutes)
        self.exploration_duration = exploration_duration
        self.get_logger().info(f"Map_width: {self.map_width}", once = True)
        self.get_logger().info(f"Map_height: {self.map_height}", once = True)
        self.get_logger().info(f"Random exploration duration: {self.exploration_duration}", once = True)

        self.map_finished_publisher = self.create_publisher(Bool, '/map_finished', 10)
        self.exploration_timer = None
        self.do_not_continue = False

    def publish_map_finished(self, is_finished):
        msg = Bool()
        msg.data = is_finished
        self.map_finished_publisher.publish(msg)

    def exploration_timer_callback(self):        
        # Stop the timer if it's no longer needed
        if self.exploration_timer:
            self.exploration_timer.cancel()
            self.do_not_continue = True

    def start_exploration_timer(self):
        self.start_time = datetime.now()
        print("Start time is: ", self.start_time)
        self.exploration_timer = self.create_timer(self.exploration_duration, self.exploration_timer_callback)
        self.publish_map_finished(False)  # Indicate exploration is starting

    def movebase_client(self):
        self.start_exploration_timer()

        while rclpy.ok():
            if not self.client.wait_for_server(timeout_sec=10.0):
                self.get_logger().info('Action server not available, waiting...')
                continue
            if self.do_not_continue:
                self.get_logger().info('Exploration phase finished.')
                self.publish_map_finished(True)  # Indicate exploration is finished
                current_date_and_time = datetime.now()
                print("Finish time is: ", current_date_and_time)
                total_duration = current_date_and_time - self.start_time

                print("Total duration ", total_duration)
                        
                return  # implicitly, this is the same as saying `return None`

            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = random.uniform(-self.map_width, self.map_width)
            goal_msg.pose.position.y = random.uniform(-self.map_height, self.map_height)
            goal_msg.pose.orientation.w = 1.0
            print("goal_msg.pose.position.x")
            print(goal_msg.pose.position.x)
            print("goal_msg.pose.position.y")
            print(goal_msg.pose.position.y)

            send_goal_future = self.client.send_goal_async(NavigateToPose.Goal(pose=goal_msg))
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle:
                self.get_logger().info("Goal was rejected by server")
                continue

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)

            result = get_result_future.result().result
            if result:
                self.get_logger().info('Goal execution done!')

                current_date_and_time = datetime.now()
                print("Finish goal time is: ", current_date_and_time)
                self.publish_map_finished(False)
            else:
                self.get_logger().info('Goal execution failed! Setting new goal...')

def main(args=None):
    rclpy.init(args=args)
    map_exploration_client = MapExplorationClient(map_width=2.24, map_height=2.24, exploration_duration = 120)
    try:
        map_exploration_client.movebase_client()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS client libraries
        map_exploration_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

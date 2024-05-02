import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pcl_interfaces.srv import EdgeSwitchover

class ColorSubscriber(Node):
    def __init__(self):
        super().__init__('color_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/current_color',
            self.color_callback,
            10)
        self.previous_color = ""

    def color_callback(self, msg):
        current_color = msg.data
        if current_color == "green" and self.previous_color == "yellow":
            self.call_edge_switchover_service()
        self.previous_color = current_color

    def call_edge_switchover_service(self):
        client = self.create_client(EdgeSwitchover, '/edge_switchover')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        request = EdgeSwitchover.Request()
        request.action_plan_id = "c7c24b5b-cc4f-42a5-aab9-9f999a7e5457"
        request.action_id = "f6c90588-8e10-4a04-b612-f16440547855"
        request.destination = "MiddlewareLutonWorkstation"
        request.destination_type = "Edge"
        request.bearer = "eyJhbGciOiJodHRwOi8vd3d3LnczLm9yZy8yMDAxLzA0L3htbGRzaWctbW9yZSNobWFjLXNoYTI1NiIsInR5cCI6IkpXVCJ9.eyJodHRwOi8vc2NoZW1hcy54bWxzb2FwLm9yZy93cy8yMDA1LzA1L2lkZW50aXR5L2NsYWltcy9uYW1lIjoiYWQyMGYyNTQtZGMzYi00MDZkLTlmMTUtYjczY2NkNDdlODY3IiwiaHR0cDovL3NjaGVtYXMubWljcm9zb2Z0LmNvbS93cy8yMDA4LzA2L2lkZW50aXR5L2NsYWltcy9yb2xlIjoiYWRtaW4iLCJqdGkiOiJlNjE2MThhNi05NjI5LTQ0MWYtYjJiOS00YTA0ZTkyOTAyNzEiLCJleHAiOjE3MTQ2NjEzMDYsImlzcyI6InJlZGlzaW50ZXJmYWNlSXNzdWVyIiwiYXVkIjoicmVkaXNpbnRlcmZhY2VBdWRpZW5jZSJ9.OOoc-nALc_FK65UloDvdWKVvkOsl3xI3HunbOHYaQjU"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call successful')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    color_subscriber = ColorSubscriber()
    rclpy.spin(color_subscriber)
    color_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

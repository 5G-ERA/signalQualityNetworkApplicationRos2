import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import *
import sensor_msgs_py.point_cloud2 as pcl2
import numpy as np
import std_msgs.msg as std_msgs
import sensor_msgs_py.point_cloud2 as pcl2
from pcl_interfaces.srv import EdgeSwitchover
import math
import requests
import json
from std_msgs.msg import String



base_url = "http://edge1.net:31000/"



class PointCloudSaverNode(Node):
    def __init__(self):
        super().__init__('auto_edge_swithchover_node')

        self.create_subscription(String, '/current_color', self.current_color_callback, 1)
        #self.save_service = self.create_service(Trigger, 'save_pointcloud', self.save_pointcloud_callback)
        #self.load_and_publish_service = self.create_service(Trigger, 'load_and_publish_pointcloud', self.load_and_publish_pointcloud_callback)
        #self.publisher_ = self.create_publisher(PointCloud2, '/loaded_pointcloud', 1)
        # Service for appending pointcloud
        #self.append_pcl = self.create_service(Trigger, 'append_pcl', self.append_pointcloud_callback)
        #self.append_pcl_custom = self.create_service(EdgeSwitchover, 'edge_switchover', self.edge_switchover_callback)      # CHANGE
        self.client = self.create_client(EdgeSwitchover, '/edge_switchover')
        self.pointcloud_data = None
        self.httpResponse = None
        self.current_colourr = "YELLOW"
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/edge_switchover service not available, waiting again...')
        self.req = EdgeSwitchover.Request()


        task_id = "4ff6c1d3-ba52-4993-8d78-c46352a6ef0c"# old task id: 6897fb8e-a2bb-4b79-ac6e-97b0c305798b
        robot_id = "8f384ffb-5599-419b-805a-206633f5f2f5"
        self.token = self.get_middleware_token()
        #get all plans and filter by task_id and robot_id
        action_plans_json = self.get_action_plan_id(self.token)

        self.action_plan_id = self.get_action_plan_id_by_task_and_robot(action_plans_json, task_id, robot_id)
        
        print("ActionPlanId")
        print(self.action_plan_id)

        self.action_id = self.get_action_id(self.token, task_id)
        self.url = "http://edge2.net:31000/task/plan/switchover"



    def current_color_callback(self, msg):


        # Check if msg.data is "GREEN" or "YELLOW"
        if msg.data == "GREEN" or msg.data == "YELLOW":
            # If msg.data has changed, update self.current_colourr and print it
            if self.current_colourr != msg.data:
                self.current_colourr = msg.data
                print(self.current_colourr)
                # Call service to switch
                # Add your service call logic here
                if msg.data == "YELLOW":
                    #Edge 1

                    print("Here1!")
                    print(self.action_id)

                    url = "http://edge1.net:31000/task/plan/switchover"
                    self.send_request(self.action_plan_id, self.action_id, "MiddlewareLutonWorkstationTwo","Edge",self.token, url_edge=url)
                    print("switch to: MiddlewareLutonWorkstationTwo")
                    print("url in current_collor_callback:")
                    print(self.url)
                    print("url in current_collor_callback:")
                    print("Here1!")
                if msg.data == "GREEN":
                    #Edge 2
                    print("Here2!")
                    print(self.action_id)

                    url = "http://edge2.net:31000/task/plan/switchover"
                    self.send_request(self.action_plan_id,self.action_id, "MiddlewareLutonWorkstation","Edge",self.token, url_edge=url)
                    print("switch to: MiddlewareLutonWorkstation")
                    print("url in current_collor_callback:")
                    print(self.url)
                    print("url in current_collor_callback:")


                    print("Here2!")

            

    def send_request(self,action_plan_id, action_id,destination, destination_type, bearer, url_edge):
        self.req.action_plan_id = action_plan_id
        self.req.action_id = action_id
        self.req.destination = destination
        self.req.destination_type = destination_type
        self.req.bearer = bearer
        self.req.url_edge = url_edge
        self.future = self.client.call_async(self.req)

    def edge_switchover_callback(self, msg, response):

        try:
            
            actionPlanId = msg.action_plan_id
            self.actionId = msg.action_id
            actionId = self.actionId
            destination = msg.destination #Name
            destinationType = msg.destination_type # edge or cloud
            bearerToken = msg.bearer
            Bearer = 'Bearer ' + bearerToken


            payload = json.dumps({
            "actionPlanId": actionPlanId,
            "actionId": actionId,
            "destination": destination,
            "destinationType": destinationType
            })
            headers = {
            'Content-Type': 'application/json',
            'Accept': 'text/plain',
            'Authorization': Bearer
            }
            print("edge_switchover_call start")

            print(self.url)

            httpResponse = requests.request("POST", self.url, headers=headers, data=payload)
            print("edge_switchover_call finish")


            if httpResponse.status_code == 200:
                response.success = True
                response.message = 'Successfull switched'
            else: 

                response.success = False
                response.message = 'Unsuccessfull switched'

            return response

        except Exception as e:
            response.success = False
            response.message = 'Unsuccessfull switched'
            return response
    
    def get_middleware_token(self):

        url = base_url + "login"

        payload = json.dumps({
            "username": "middleware",
            "password": "middleware"
        })
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json'  # Assuming the response is in JSON format
        }

        try:
            response = requests.post(url, headers=headers, data=payload)
            response.raise_for_status()  # Raises an exception for 4XX or 5XX errors
            token = response.json().get('token')  # Extract token from response
            #print(token)
            return token
        except requests.RequestException as e:
            print(f"Request failed: {e}")
            return None
        
    def get_action_plan_id(self, token:str):


        url = base_url + "data/action/plan"

        payload = {}
        headers = {
        'Accept': 'text/plain',
        'Authorization': 'Bearer ' + token
        }

        response = requests.request("GET", url, headers=headers, data=payload)

        #print(response.text)
        
        return response.json()
    
    def get_action_id(self, token:str, task_id):


        url = base_url + "data/task/" + task_id + "/full"

        payload = {}
        headers = {
        'Accept': 'text/plain',
        'Authorization': 'Bearer ' + token
        }

        response = requests.request("GET", url, headers=headers, data=payload)

        #print(response.text)
        response_json = response.json()

        action_id = response_json["actionSequence"][0]["id"]
        print("ActionId:")

        print(action_id)
        return action_id
        

    def get_action_plan_id_by_task_and_robot(self, action_plans, task_id, robot_id):
        """
        Filters the action plans to find the one matching the given task ID and robot ID.
        
        Parameters:
        - action_plans (list): A list of dictionaries, each representing an action plan.
        - task_id (str): The task ID to filter by.
        - robot_id (str): The robot ID to filter by.
        
        Returns:
        - str: The actionPlanId of the matching action plan, or None if no match is found.
        """
        for plan in action_plans:
            if plan['taskId'] == task_id and plan['robotId'] == robot_id:
                return plan['id']
        return None




def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver_node = PointCloudSaverNode()
    rclpy.spin(pointcloud_saver_node)
    if pointcloud_saver_node.future.done():
            try:
                response = pointcloud_saver_node.future.result()
            except Exception as e:
                pointcloud_saver_node.get_logger().info('Service call failed %r' % (e,))
            else:
                pointcloud_saver_node.get_logger().info('Service call succeeded')
    pointcloud_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/bin/sh

# Source ros
source /opt/ros/humble/setup.bash
# Export unique ROS_DOMAIN_ID for testing on same machine
export ROS_DOMAIN_ID=22
# Export NetApp address of interface
# export NETAPP_ADDRESS=http://192.168.50.124:5896
# export NETAPP_ADDRESS=http://192.168.0.43:5896
# Set TOPIC_LIST environment variable (input, images)

export USE_MIDDLEWARE=true
export MIDDLEWARE_ADDRESS=middleware.local:30010
export MIDDLEWARE_USER=ad20f254-dc3b-406d-9f15-b73ccd47e867
export MIDDLEWARE_PASSWORD=middleware
export MIDDLEWARE_TASK_ID=c1e383cd-61f5-40d7-abe1-0796919ed8b0
export MIDDLEWARE_ROBOT_ID=68013a37-7027-4b30-8c04-701debb0ba44
export TOPICS_TO_SERVER='[{"name":"/amcl_pose","type":"geometry_msgs/msg/PoseWithCovarianceStamped","qos":{"durability":"TRANSIENT_LOCAL","reliability":"RELIABLE","history":"KEEP_LAST","depth":1}}]'
export TOPICS_FROM_SERVER='[{"name": "/semantic_pcl","type": "sensor_msgs/msg/PointCloud2","compression": "lz4"}]'

python3 -m era_5g_relay_network_application.client

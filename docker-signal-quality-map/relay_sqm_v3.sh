#!/bin/sh

# Check if the library is installed, if not, install it
if ! python3 -c "import era_5g_relay_network_application" &> /dev/null; then
    echo "The era-5g-relay-network-application library is not installed. Installing it..."
    sudo pip install era-5g-relay-network-application
fi

#SOURCE THE ROS INSTALL
source /opt/ros/humble/setup.bash

export USE_MIDDLEWARE=true
export MIDDLEWARE_ADDRESS=address
export MIDDLEWARE_USER=ad20f254-dc3b-406d-9f15-b73ccd47e867
export MIDDLEWARE_PASSWORD=password
export MIDDLEWARE_TASK_ID=c1e383cd-61f5-40d7-abe1-0796919ed8b0
export MIDDLEWARE_ROBOT_ID=300c719a-1c06-4500-a13a-c2e20592b273
# TOPICS TO SEND FROM ROBOT TO NETWORK APPLICATION
export TOPICS_TO_SERVER='[{"name":"/amcl_pose","type":"geometry_msgs/msg/PoseWithCovarianceStamped","qos":{"durability":"TRANSIENT_LOCAL","reliability":"RELIABLE","history":"KEEP_LAST","depth":1}},{"name":"/pcl_colour","type":"std_msgs/msg/String"}]'
# TOPICS TO SEND FROM NETWORK APPLICATION BACK TO ROBOT
export TOPICS_FROM_SERVER='[{"name": "/semantic_pcl","type": "sensor_msgs/msg/PointCloud2"}]'
# RUN RELAY CLIENT
python3 -m era_5g_relay_network_application.client

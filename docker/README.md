# signal_quality_network_application_ros2 docker

This package allows the robot to create semantic maps that can later be translated into occupancy grid maps merged with the main map topic. Semantic interpretations are easy to configure as a list of colors to be considered "virtual" obstacles in the new map. I.E Low quality area of radio 5G signal. This in term will make the nav2 package avoid the blind spots. This application is very useful for cloud based mobile robots that offload computational resources to the cloud and want to keep QoS and QoE specific metrics using docker

To launch the semantic mapping, which creates a pointcloud2 around the robot boundingbox, run:
# Requirements and setup:
This code was tested in ROS 2 Humble and Foxy. Copy the content of this repository by using either git clone or manual download.

```
# LAUNCH SIMULATION 
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

to launch both services (signal_mapper and sub_signal_mapper) at the same time detached run:
```
docker compose up -d
```

to run each service (signal_mapper and sub_signal_mapper) separate open terminal from coresponding folder(ex signal_mapper) and run:
```
docker compose up
```

To create images for container:
from folder signal_mapper run 
```
docker build . -t signal-mapper:1.0
```
from folder sub_signal_mapper run 
```
docker build . -t sub-signal-mapper:1.0
```

# signal_quality_network_application_ros2

This package allows the robot to create semantic maps that can later be translated into occupancy grid maps merged with the main map topic. Semantic interpretations are easy to configure as a list of colors to be considered "virtual" obstacles in the new map. I.E Low quality area of radio 5G signal. This in term will make the nav2 package avoid the blind spots. This application is very useful for cloud based mobile robots that offload computational resources to the cloud and want to keep QoS and QoE specific metrics.

To launch the semantic mapping, which creates a pointcloud2 around the robot boundingbox, run:
# Requirements and setup:
This code was tested in ROS 2 Humble and Foxy. Copy the content of this repository by using either git clone or manual download and paste it to your /src folder in your ros2 workspace. Then follow by building the ws.

```
colcon build
```

Make sure to source both your workspace afterwards and the ros2 installation under /opt. If you dont do so then ros2 launch will fail as it will not find the package.

## Introduction: 
```
ros2 launch era_5g_network_signal_mapper_ros2 era_5g_network_signal_mapper_ros2.launch.py
```
![image](https://github.com/5G-ERA/signal_quality_network_application_ros2/assets/26432703/19411486-743c-4fe0-a5e6-3dd4fcd7e37b)


You may configure beforehand if you so wish, the boundingbox of the robot according to its geometry. This example assumes a RobotNik [SummitXL](https://robotnik.eu/products/mobile-robots/summit-xl-en-2/) geometry. 

 Under ros node **signal_mapper.py** modify the following variables.

    * height = 0.3
    * lenght = 0.3
    * lamba = 0.05
    
Lamba is the amount of points to be created along the height and lenght. It may stay at 0.05 to look like the pictures presented in this tutorial.

## Control semantic color publications:

The ros node signal mapper subscribes to a color topic and it will change the color of the current pointcloud depending on the latest recived data of the colors.

For the easiest interaction, you can use a ros node to manully change the colors according to your preferences. 

```
ros2 run era_5g_network_signal_mapper_ros2 colour_pub
```

For our experiments, we created a ros node (publisher.py) that reads from a database and publises the color topic. This database was influx and received everysecond a 5G signal coming from a Teltonika 5G router.

## Costmap translation:

You can either run this node at the begining of your semantic mapping process or after it is finished. This node will subscribe to the current /map topic from SLAM and merge it with a translation to occupancyGrid of the pointclouds with the colors that are considered obstacles. This obstacles color list is defined at the begining of the node a python list and expects a color class object with its rgb representation. If you wish to add extra colors to the ones defined by default, make sure to expand upon signal_mapper.py and include in the subscriber the proper rgb translations.

```
ros2 run era_5g_network_signal_mapper_ros2 costmap_translate
```
![image](https://github.com/5G-ERA/signal_quality_network_application_ros2/assets/26432703/ce22ea4a-9359-41b4-aa77-47287ddc5ae5)


## Saving the new map and load it to map_server:
```
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/test/maps/semantic-04-08-2023
```

To laod the map in RVIZ to be used by nav2 and amcl:
```
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/robot/robot_ws/src/test/maps/ote_indoor/semantic-04-08-2023.yaml}"
```
##  To save and load the pointcloud map

You may use rosbags for this purpose, so you can save a semantic map if you still dont want to do the map translation.
```
ros2 bag record --all --compression-mode file --compression-format zstd
```
For reply, use ros2 bag play and the name of your rosbag file.

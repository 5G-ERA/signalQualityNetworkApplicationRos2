# ROS 2 Semantic map package using 5G radio signal quality.

This package allows the robot to create semantic maps that can later be translated into occupancy grid maps merged with the main map topic (from your SLAM algorithm). Semantic interpretations are easy to configure as a list of colors to be considered "virtual" obstacles in the new map. I.E Low quality area of radio 5G signal. This in term will make the ROS2 nav2 package avoid the blind spots. This application is very useful for cloud based mobile robots that offload computational resources to the cloud and want to keep QoS and QoE specific metrics.

Experimental research was done with RobotNik's [SummitXL](https://robotnik.eu/products/mobile-robots/summit-xl-en-2/) and conducted with a real 5G test-bed. Results can be found following the IROS paper publication.

```
A. Lendinez et al., "Enhancing 5G-Enabled Robots Autonomy by Radio-Aware Semantic Maps," *2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Detroit, MI, USA, 2023, pp. 6267-6272, doi: [10.1109/IROS55552.2023.10342279](https://doi.org/10.1109/IROS55552.2023.10342279).
Keywords: Wireless communication; 5G mobile communication; Semantics; Surfaces; Reliability; Mobile robots; Task analysis;
```
Video depiction of real operational tests: 

[![Demo Video](http://img.youtube.com/vi/CMcDZyFyge8/0.jpg)](https://www.youtube.com/watch?v=CMcDZyFyge8)

Following is a list of features that this package provides:

|   | Features                     | Scripts                                                                                                    | Service call                                              | Publishes topics                    | Subscribed topics                             |
|---|------------------------------|------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------|-------------------------------------|-----------------------------------------------|
| 1 | Edge Switchover              | client_service_auto_switch_param_hardcoded client_service_auto_switchover_action switchover_service_server | /edge_switchover pcl_interfaces/srv/EdgeSwitchover        | -                                   | /current_color                                |
| 2 | Get current color            | current_color service_server_semantic_map.py (previous tools.py)                                           | -                                                         | -                                   | -                                             |
| 3 | Publsih color                | colours_red_green_yellow colours_multy                                                                     | -                                                         | -                                   | -                                             |
| 4 | Create semantic map          | signal_mapper sub_signal_mapper tf2_custom                                                                 | -                                                         | /current_semantic_pcl /semantic_pcl | /pcl_colour  /amcl_pose /current_semantic_pcl |
| 5 | Costmap translation          | costmap_translate                                                                                          | GetMap, /map_server/map                                   | /map_Semantic                       | -                                             |
| 6 | Confidence layer calculation | -                                                                                                          | -                                                         | -                                   | -                                             |
| 7 | Load map                     | service_server_semantic_map                                                                                | /load_and_publish_pointcloud std_srvs/Trigger             | -                                   | -                                             |
| 8  | Save map                     | service_server_semantic_map                                                                                | /save_pointcloud std_srvs/Trigger                         | -                                   | -                                             |
| 9  | Request area of interest     | service_server_semantic_map                                                                                | /request_partial_map pcl_interfaces/srv/PartialMapRequest | -                                   | -                                             |
| 10  | Merge two semantic maps      | service_server_semantic_map                                                                                | /append_pcl pcl_interfaces/srv/AppendPcl                  | -                                   | -                                             |

## 1) Installation and requirements:

This code was tested in ROS 2 **Humble**. Copy the content of this repository by using either git clone or manual download and paste it to your /src folder in your ros2 workspace. The only dependency is tf2-kdl which you can install follwing:

```bash
sudo apt-get install ros-<ros_version>-tf2-kdl
```

Then build the ws.

```
colcon build
```

Make sure to source both your workspace afterwards and the ros2 installation under /opt. If you dont do so then ros2 will fail to run the package.

## 2) Setup & simulations: 
To run this package, you will need a simulation or real robot running ROS2. Semantic mapping is done after SLAM. The package needs:

* A map under the frame **/map**.
* **/amcl_pose topic** to track robot position. (It can also use TF for robot base_frame and it can be run during SLAM). 
* The frame **/base_footprint** or your remapping to your frame corresponding to the lower center of the robot.

The turtlebot3 House simulation was used for testing & development purpose and it works out-of-the-box. Follow the [tutorial](
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) from turtlebot Humble to install. If you encounter problems while running this simulation in Gazebo with Humble, please try this command before running in the terminal for your simulation launch.

```
source /usr/share/gazebo-11/setup.bash
```


## 3) Run the package: 

You can run all basic features with this launch file. (Make sure to source the workspace beforehand).

```
ros2 launch era_5g_network_signal_mapper_ros2 era_5g_network_signal_mapper_ros2.launch.py
```
![image](https://github.com/5G-ERA/signal_quality_network_application_ros2/assets/26432703/19411486-743c-4fe0-a5e6-3dd4fcd7e37b)

For demo purpose, a ROS 2 node is avaiable for controlling the color publication. It will change the color of the current pointcloud (PCL) around the robot.

```
ros2 run era_5g_network_signal_mapper_ros2 colours_multy
```

For our experiments, we created a ros node (publisher.py) that reads from a database and publises the color topic. This database was influx and received everysecond a 5G signal coming from a Teltonika 5G router. We are assesing only RSRP values for the color management.

## 4) Costmap translation and navigation using semantic maps:

 This node will subscribe to the current /map topic from SLAM and merge it with a translation to occupancyGrid of the pointclouds with the colors that are considered obstacles (the semantic map specific color areas). This obstacles color list is defined at the begining of the node as a python list and expects a color class object with its rgb representation. If you wish to add extra colors to the ones defined by default, make sure to expand upon signal_mapper.py and include in the subscriber the proper rgb translations.

```
ros2 run era_5g_network_signal_mapper_ros2 costmap_translate
```
![image](https://github.com/5G-ERA/signal_quality_network_application_ros2/assets/26432703/ce22ea4a-9359-41b4-aa77-47287ddc5ae5)


## 5) Saving the new occupancyGrid map and load it with map_server:
```
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/test/maps/semantic-04-08-2023
```

To laod the map in RVIZ to be used by nav2 and amcl:
```
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/robot/robot_ws/src/test/maps/ote_indoor/semantic-04-08-2023.yaml}"
```


## 6) Saving and loading semantic map:

If you are already running the launch file, the semantic map service server should be already running. If not, spin it up by using:

```
cd era_5g_network_signal_mapper_ros2"
```
From this location run in terminal to run node for starting service server:

```
source /opt/ros/humble/setup.bash
```

```
python3 service_server_semantic_map.py
```

To save the semantic map to a file use:

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

```
ros2 service call /save_pointcloud std_srvs/Trigger "{}"
```
To load sematic pointcloud from file run:
```
ros2 service call /load_and_publish_pointcloud std_srvs/Trigger "{}"
```
And select topic: "/loaded_pointcloud"

**At the momment this service call does not support custom naming for the saved file. It needs to be done manually.**


## 7) Get current colour of the semantic map:

This feature allows the robot to get the current color of the semantic map where it is currently located. 
In order t use this feature the pakage must be run before the pointcloud is loaded either via saving and loading semantic poitcloud as a static file or running signal_mapper package.

from termianl:
```
ros2 run era_5g_network_signal_mapper_ros2 current_color
```

The topic "/current_color" will publish current color of the location of the robot

## 8) Switchover using CROP Middleware

CROP Middleware is an open source system for orchestration and deployment of network applications for ROS. It enables cloud based robotics as an easy way. A robot can call a switch over from one edge/cloud location to another.
Manual service call example. (It requiers launch file to spin switchover_service_server.py or manual ros2 run)

```
ros2 service call /edge_switchover pcl_interfaces/srv/EdgeSwitchover "{action_plan_id: 6c182717-af5e-459f-a1a5-bd2cfb363e47,action_id: 6c182717-af5e-459f-a1a5-bd2cfb363e47,destination: name_of_dest_middleware,destination_type: edge, bearer: 6c182717}"
```

The switchover can be automatically done by the robot considering the semantic network quality map. First load your semantic map using the service call. The witchover is done when robot changes from one color to another.

```
ros2 run era_5g_network_signal_mapper_ros2 client_service_auto_switchover_action
```


## 9) Addendum: 

* ROS2 Bag for recording with compresion.
```
ros2 bag record --all --compression-mode file --compression-format zstd
```

* ROS 1 Package also available at github [repository](https://github.com/5G-ERA/signal_quality_network_application).

* CROP Middleware and documentation available at github [repository](https://github.com/5G-ERA/docs). 

## 10) Acknoledgements:

*This work was sponsored by the 5G-ERA EU Project with funding of the European Union's Horizon 2020 Research and Innovation programme under grant agreement No. 101016681.*

Visit the 5G ERA project website for more information.
https://5g-era.eu.

